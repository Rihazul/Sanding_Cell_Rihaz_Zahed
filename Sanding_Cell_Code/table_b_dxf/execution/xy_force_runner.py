from __future__ import annotations

import time

from typing import Any

from Server_Better_V2 import (
    clear_stop,
    communicate,
    putForceZplus,
    releaseForce,
    request_stop,
    stop_requested,
    turn_vibration_off,
    turn_vibration_on,
)

from .common import (
    TableBDxfRobotExecutionError,
    TableBDxfRobotStopRequested,
    log as _log,
    raise_if_stop_requested as _raise_if_stop_requested,
    stop_active_motion as _stop_active_motion,
)
from .joint_safety import JOINT_LIMITS_DEG, read_current_joints
from .motion_config import (
    force_fast_approach_mm,
    force_sanding_overshoot_mm,
    orientation,
    preheight_z_mm,
    robot_speed,
    sanding_speed,
    table_b_ucs,
    tcp_for_physical_tool,
)
from ..tool_reach import TOOL_REACH, reach_span_at_y


def _pose_from_xy(xy: list[float], z_mm: float, rxyz: list[float]) -> list[float]:
    if len(xy) < 2:
        raise TableBDxfRobotExecutionError(f"Invalid XY point: {xy}")
    return [float(xy[0]), float(xy[1]), float(z_mm), rxyz[0], rxyz[1], rxyz[2]]


def _path_points(step: dict[str, Any]) -> list[list[float]]:
    points = step.get("robot_base_points") or []
    cleaned: list[list[float]] = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        xy = [float(point[0]), float(point[1])]
        if cleaned and abs(cleaned[-1][0] - xy[0]) < 0.001 and abs(cleaned[-1][1] - xy[1]) < 0.001:
            continue
        cleaned.append(xy)
    if len(cleaned) < 2:
        raise TableBDxfRobotExecutionError(
            f"Path {step.get('path_id')} has fewer than 2 executable robot-base points."
        )
    return cleaned


def _points_close(a: list[float], b: list[float], tolerance_mm: float = 0.001) -> bool:
    return abs(a[0] - b[0]) <= tolerance_mm and abs(a[1] - b[1]) <= tolerance_mm


def _is_closed_path(step: dict[str, Any], points: list[list[float]]) -> bool:
    if bool(step.get("closed") or step.get("is_closed") or step.get("is_loop")):
        return True
    return len(points) > 2 and _points_close(points[0], points[-1])


def _closed_cycle_points(points: list[list[float]]) -> list[list[float]]:
    loop_points = [list(point) for point in points]
    if not _points_close(loop_points[0], loop_points[-1]):
        loop_points.append(list(loop_points[0]))
    return loop_points[1:]


def _open_cycle_points(points: list[list[float]], cycle_index: int) -> list[list[float]]:
    # Odd cycles continue forward from start to end; even cycles return end to start.
    if cycle_index % 2 == 1:
        return points[1:]
    return list(reversed(points[:-1]))


def _cycle_points(points: list[list[float]], is_closed: bool, cycle_index: int) -> list[list[float]]:
    if is_closed:
        return _closed_cycle_points(points)
    return _open_cycle_points(points, cycle_index)


def _motion_queue_flush_every(config: dict[str, Any]) -> int:
    value = None
    if isinstance(config, dict):
        value = (config.get("door") or {}).get("tableBDxfMotionFlushEveryPoints")
        if value is None:
            value = (config.get("settings") or {}).get("tableBDxfMotionFlushEveryPoints")
    try:
        flush_every = int(value) if value is not None else 8
    except (TypeError, ValueError):
        flush_every = 8
    return max(0, flush_every)


def _simultaneous_j7_robot_enabled(config: dict[str, Any]) -> bool:
    if not isinstance(config, dict):
        return True
    value = (config.get("door") or {}).get("tableBDxfSimultaneousJ7Robot")
    if value is None:
        value = (config.get("settings") or {}).get("tableBDxfSimultaneousJ7Robot")
    if value is None:
        return True
    if isinstance(value, str):
        return value.strip().lower() not in ("0", "false", "no", "off")
    return bool(value)


def _joint_transition_margin_deg(config: dict[str, Any]) -> float:
    value = None
    if isinstance(config, dict):
        motion_cfg = config.get("tableBDxfMotion")
        if isinstance(motion_cfg, dict):
            value = motion_cfg.get("jointTransitionLimitMarginDeg")
        if value is None:
            value = (config.get("door") or {}).get("tableBDxfJointTransitionLimitMarginDeg")
        if value is None:
            value = (config.get("settings") or {}).get("tableBDxfJointTransitionLimitMarginDeg")
    try:
        return max(0.0, float(value)) if value is not None else 10.0
    except (TypeError, ValueError):
        return 10.0


def _should_wait_for_motion_point(point_index: int, total_points: int, flush_every: int) -> bool:
    if point_index >= total_points:
        return True
    return flush_every > 0 and point_index % flush_every == 0


def _low_y_limit_for_tool(physical_tool: int) -> float | None:
    spec = TOOL_REACH.get(f"tool_{physical_tool}") or {}
    try:
        return float(spec.get("rect_below_y"))
    except (TypeError, ValueError):
        return None


def _read_current_coord_pose(cps: Any, tcp: str, ucs: str) -> list[float] | None:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result)
    except Exception:
        ret = -1
    if ret == 0 and len(result) >= 12:
        try:
            return [float(result[i]) for i in range(6, 12)]
        except (TypeError, ValueError):
            return None
    return None


def _requires_j7_idle_before_prepoint(
    physical_tool: int,
    current_pose: list[float] | None,
) -> bool:
    """If the arm is currently low/bottom, do not move it while J7 is travelling."""
    low_y_limit = _low_y_limit_for_tool(physical_tool)
    if low_y_limit is None:
        return False
    if current_pose is None or len(current_pose) < 2:
        return True
    return float(current_pose[1]) < low_y_limit


def _requires_high_negative_x_escape_before_low_y(
    physical_tool: int,
    current_pose: list[float] | None,
    target_xy: list[float],
) -> bool:
    """Avoid direct high negative-X to low-Y moves that can fold J3."""
    low_y_limit = _low_y_limit_for_tool(physical_tool)
    if low_y_limit is None or current_pose is None or len(current_pose) < 2 or len(target_xy) < 2:
        return False
    current_x = float(current_pose[0])
    current_y = float(current_pose[1])
    target_y = float(target_xy[1])
    return current_y >= low_y_limit and current_x < 0.0 and target_y < low_y_limit


def _read_current_joints_for_transition(cps: Any, config: dict[str, Any]) -> list[float] | None:
    try:
        return read_current_joints(cps)
    except TableBDxfRobotExecutionError as exc:
        _log(
            config,
            "[TableB DXF Robot] joint transition guard could not read joints; using non-simultaneous J7. %s",
            exc,
        )
        return None


def _joint_transition_guard(
    cps: Any,
    config: dict[str, Any],
) -> tuple[bool, list[float] | None, str]:
    """Return whether J7 should finish before arm motion because a joint is near a limit."""
    joints = _read_current_joints_for_transition(cps, config)
    if joints is None:
        return True, None, "joint_read_failed"

    margin = _joint_transition_margin_deg(config)
    risks: list[str] = []
    for index, value in enumerate(joints[:6]):
        low, high = JOINT_LIMITS_DEG[index]
        safe_low = low + margin
        safe_high = high - margin
        if value <= safe_low or value >= safe_high:
            risks.append(f"J{index + 1}={value:.1f} outside transition window [{safe_low:.1f}, {safe_high:.1f}]")
    if risks:
        return True, joints, "; ".join(risks)
    return False, joints, "ok"


def _is_negative_x_in_low_y_zone(physical_tool: int, x: float, y: float) -> bool:
    low_y_limit = _low_y_limit_for_tool(physical_tool)
    return low_y_limit is not None and float(y) < low_y_limit and float(x) < 0.0


def _assert_robot_base_points_reachable(step: dict[str, Any], physical_tool: int, points: list[list[float]]) -> None:
    """Final execution guard against stale or unsafe approved points."""
    reach_tool = f"tool_{physical_tool}"
    for point in points:
        x, y = float(point[0]), float(point[1])
        if _is_negative_x_in_low_y_zone(physical_tool, x, y):
            low_y_limit = _low_y_limit_for_tool(physical_tool)
            raise TableBDxfRobotExecutionError(
                f"Path {step.get('path_id')} is unsafe for Tool {physical_tool}: "
                f"robot-base point=({x:.1f}, {y:.1f}) has negative X in the low-Y zone "
                f"(Y < {float(low_y_limit):.1f}). Regenerate/approve with a legal 7th-axis stop."
            )
        span = reach_span_at_y(reach_tool, y)
        if span is None:
            raise TableBDxfRobotExecutionError(
                f"Path {step.get('path_id')} has unreachable Y={y:.1f} for Tool {physical_tool}."
            )
        lo, hi = span
        if x < lo - 1e-6 or x > hi + 1e-6:
            raise TableBDxfRobotExecutionError(
                f"Path {step.get('path_id')} is unsafe for Tool {physical_tool}: "
                f"robot-base point=({x:.1f}, {y:.1f}) outside local reach [{lo:.1f}, {hi:.1f}]. "
                "Preview and approve the toolpath again with a legal 7th-axis stop."
            )


def _wait_force_stabilized(cps, config, force_goal_n, contact_threshold_n):
    """After contact is detected, wait until the contact force HOLDS near the commanded goal
    before the toolpath motion starts.

    putForceZplus returns as soon as force crosses the (low) contact threshold, but the
    controller is still ramping to the full force and the contact is still settling. If the
    toolpath MoveL starts then, the tool skims/hovers at the path start while force ramps.
    Here we poll the Z force and require it to be at/above the hold level for a few
    consecutive reads. Tunable via config.door: forceHoldFractionOfGoal (0.85),
    forceHoldStableReads (3), forceHoldPollSec (0.02), forceHoldTimeoutSec (2.0).
    """
    door = config.get("door", {}) if isinstance(config, dict) else {}
    def _f(key, default):
        try:
            return float(door.get(key, default))
        except (TypeError, ValueError):
            return float(default)
    poll_s = max(0.005, _f("forceHoldPollSec", 0.02))
    timeout_s = max(0.1, _f("forceHoldTimeoutSec", 2.0))
    hold_fraction = min(1.0, max(0.1, _f("forceHoldFractionOfGoal", 0.85)))
    try:
        stable_needed = max(1, int(door.get("forceHoldStableReads", 3)))
    except (TypeError, ValueError):
        stable_needed = 3
    goal = abs(float(force_goal_n))
    # Hold target: most of the goal force, but never below the contact threshold.
    hold_level = max(abs(float(contact_threshold_n)), hold_fraction * goal) if goal > 0 else abs(float(contact_threshold_n))

    def _z_force():
        result = []
        try:
            nret = cps.HRIF_ReadFTCabData(0, 0, result)
        except Exception:
            return None
        if nret != 0 or not isinstance(result, (list, tuple)) or len(result) < 3:
            return None
        try:
            return abs(float(result[2]))  # Z axis force
        except (TypeError, ValueError):
            return None

    start = time.time()
    stable = 0
    while True:
        if stop_requested():
            return
        fz = _z_force()
        if fz is None:
            time.sleep(poll_s)
            # If we cannot read force, do not block the path; a short dwell then proceed.
            if time.time() - start >= min(0.3, timeout_s):
                return
            continue
        if fz >= hold_level:
            stable += 1
            if stable >= stable_needed:
                if isinstance(config, dict) and config.get("logger"):
                    config["logger"].info("[forceHold] force stabilized at %.2fN (goal %.2fN)", fz, goal)
                return
        else:
            stable = 0
        if time.time() - start >= timeout_s:
            if isinstance(config, dict) and config.get("logger"):
                config["logger"].warning("[forceHold] force not confirmed held (last %.2fN, target %.2fN) after %.2fs; continuing.", fz, hold_level, timeout_s)
            return
        time.sleep(poll_s)

def _wait_until_arm_settled(cps, config):
    """Block until the arm TCP position stops changing, independent of the motion-done flag.

    At high speed that flag can report done while the arm is still coasting from
    deceleration; starting force control then makes the tool hover. So we poll the actual
    TCP position and require it stable for a few reads before force-on. Tunable via
    config.door: forceSettlePollSec, forceSettleStableReads, forceSettleThresholdMm,
    forceSettleTimeoutSec.
    """
    door = config.get("door", {}) if isinstance(config, dict) else {}
    def _f(key, default):
        try:
            return float(door.get(key, default))
        except (TypeError, ValueError):
            return float(default)
    poll_s = max(0.005, _f("forceSettlePollSec", 0.02))
    threshold_mm = max(0.01, _f("forceSettleThresholdMm", 0.2))
    timeout_s = max(0.1, _f("forceSettleTimeoutSec", 1.5))
    try:
        stable_needed = max(1, int(door.get("forceSettleStableReads", 3)))
    except (TypeError, ValueError):
        stable_needed = 3

    def _tcp_xyz():
        result = []
        try:
            nret = cps.HRIF_ReadActPos(0, 0, result)
        except Exception:
            return None
        if nret != 0 or not isinstance(result, (list, tuple)) or len(result) < 9:
            return None
        try:
            return (float(result[6]), float(result[7]), float(result[8]))
        except (TypeError, ValueError):
            return None

    start = time.time()
    prev = _tcp_xyz()
    stable = 0
    while True:
        if stop_requested():
            return
        time.sleep(poll_s)
        cur = _tcp_xyz()
        if cur is None or prev is None:
            time.sleep(poll_s)
            return
        moved = ((cur[0]-prev[0])**2 + (cur[1]-prev[1])**2 + (cur[2]-prev[2])**2) ** 0.5
        prev = cur
        if moved <= threshold_mm:
            stable += 1
            if stable >= stable_needed:
                return
        else:
            stable = 0
        if time.time() - start >= timeout_s:
            if isinstance(config, dict) and config.get("logger"):
                config["logger"].warning(
                    "[forceSettle] arm not confirmed settled after %.2fs (last move %.3f mm); continuing.",
                    timeout_s, moved,
                )
            return


def _table_b_motion_value(config: dict[str, Any], key: str, default: float) -> float:
    value = None
    if isinstance(config, dict):
        motion_cfg = config.get("tableBDxfMotion")
        if isinstance(motion_cfg, dict):
            value = motion_cfg.get(key)
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


def _force_search_linear_velocity(config: dict[str, Any], physical_tool: int) -> float:
    if physical_tool == 1:
        return _table_b_motion_value(config, "tool1SearchLinearVelocity", 5.0)
    return _table_b_motion_value(config, "searchLinearVelocity", 5.0)


def _contact_force_threshold(config: dict[str, Any], physical_tool: int) -> float | None:
    if physical_tool != 1:
        return None
    # Tool 1 is soft/lightweight. This only decides when Python exits the
    # contact-search loop; putForceZplus still sends the operator force as the
    # controller goal, so it does not cap sanding force.
    return _table_b_motion_value(config, "tool1ContactForceThresholdN", 2.0)


def _low_y_escape_margin_mm(config: dict[str, Any]) -> float:
    return max(0.0, _table_b_motion_value(config, "lowYEscapeMarginMm", 25.0))


def _escape_low_y_during_j7(
    cps: Any,
    config: dict[str, Any],
    *,
    physical_tool: int,
    current_pose: list[float] | None,
    axis7_position: float | None,
    pre_z: float,
    rxyz: list[float],
    tcp: str,
    ucs: str,
) -> bool:
    """Move out of the bottom reach constraint while J7 is travelling.

    A destination point may be legal at high Y with negative local X, but the linear
    transition from a bottom pose can briefly command negative X while Y is still low. Keep
    local X non-negative during the low-Y escape, then reposition to the real target after
    the arm is above the constrained zone and J7 is idle.
    """
    low_y_limit = _low_y_limit_for_tool(physical_tool)
    if low_y_limit is None or axis7_position is None or current_pose is None or len(current_pose) < 3:
        return False

    current_x = float(current_pose[0])
    current_y = float(current_pose[1])
    current_z = float(current_pose[2])
    if current_y >= low_y_limit:
        return False

    escape_x = max(0.0, current_x)
    escape_y = low_y_limit + _low_y_escape_margin_mm(config)
    _log(
        config,
        "[TableB DXF Robot] low-Y escape during J7: current=(%.1f, %.1f, %.1f) escape=(%.1f, %.1f, %.1f) j7=%.1f",
        current_x,
        current_y,
        current_z,
        escape_x,
        escape_y,
        pre_z,
        float(axis7_position),
    )

    if current_z > pre_z + 0.5:
        communicate(
            cps=cps,
            config=config,
            point=[current_x, current_y, pre_z, rxyz[0], rxyz[1], rxyz[2]],
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=False,
        )
    j7_result = communicate(
        cps=cps,
        config=config,
        point=None,
        tcp=tcp,
        ucs=ucs,
        seventh=axis7_position,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=False,
        require_seventh_ok=True,
    )
    if j7_result is None:
        raise TableBDxfRobotExecutionError("7th-axis async move failed during low-Y escape.")
    communicate(
        cps=cps,
        config=config,
        point=[escape_x, escape_y, pre_z, rxyz[0], rxyz[1], rxyz[2]],
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=True,
    )
    return True


def _escape_high_negative_x_during_j7(
    cps: Any,
    config: dict[str, Any],
    *,
    physical_tool: int,
    current_pose: list[float] | None,
    axis7_position: float | None,
    pre_z: float,
    rxyz: list[float],
    tcp: str,
    ucs: str,
) -> bool:
    """Move from top negative-X to X=0 before descending into the low-Y zone."""
    low_y_limit = _low_y_limit_for_tool(physical_tool)
    if low_y_limit is None or axis7_position is None or current_pose is None or len(current_pose) < 3:
        return False

    current_x = float(current_pose[0])
    current_y = float(current_pose[1])
    current_z = float(current_pose[2])
    if current_y < low_y_limit or current_x >= 0.0:
        return False

    escape_y = max(current_y, low_y_limit + _low_y_escape_margin_mm(config))
    _log(
        config,
        "[TableB DXF Robot] high negative-X escape during J7: current=(%.1f, %.1f, %.1f) escape=(0.0, %.1f, %.1f) j7=%.1f",
        current_x,
        current_y,
        current_z,
        escape_y,
        pre_z,
        float(axis7_position),
    )

    if current_z > pre_z + 0.5:
        communicate(
            cps=cps,
            config=config,
            point=[current_x, current_y, pre_z, rxyz[0], rxyz[1], rxyz[2]],
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=False,
        )
    j7_result = communicate(
        cps=cps,
        config=config,
        point=None,
        tcp=tcp,
        ucs=ucs,
        seventh=axis7_position,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=False,
        require_seventh_ok=True,
    )
    if j7_result is None:
        raise TableBDxfRobotExecutionError("7th-axis async move failed during high negative-X escape.")
    communicate(
        cps=cps,
        config=config,
        point=[0.0, escape_y, pre_z, rxyz[0], rxyz[1], rxyz[2]],
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=True,
    )
    return True


def _lift_to_preheight_no_force(
    cps: Any,
    config: dict[str, Any],
    current_xy: list[float],
    pre_z: float,
    rxyz: list[float],
    tcp: str,
    ucs: str,
    physical_tool: int,
) -> None:
    """On STOP, lift the tool straight up to the pre-height (-17) with NO force.

    Force control must already be OFF (stop_active_motion disables it) so this is a pure
    position move that releases the surface immediately — no pressure, no dragging, no
    marks. We lift straight up from the tool's ACTUAL XY (falling back to the last commanded
    XY), changing only Z, so there is no lateral scrub across the surface. Best-effort: never
    raise, so stop cleanup always completes even if the controller rejects the move.

    Tool 2 is out of scope (it is not run by this runner), so this is only reached for
    Tools 1/3/4.
    """
    if physical_tool not in (1, 3, 4):
        return
    logger = config.get("logger") if isinstance(config, dict) else None
    try:
        # Read the tool's ACTUAL XY/Z. A mid-move stop halts the tool between commanded
        # points, so the last commanded current_xy is not exactly where the tool is; lifting
        # from the actual XY makes the retract a true straight-up move with no lateral scrub
        # across the surface. We also use actual Z to ensure we only move AWAY from the
        # surface: on Table B +Z is toward the surface, so lifting means going to a
        # LESS-positive Z (pre_z). If the tool is already at/above pre_z, skip so we never
        # command a move back down into the surface.
        actual_xy = None
        actual_z = None
        result = []
        try:
            if cps.HRIF_ReadActPos(0, 0, result) == 0 and len(result) >= 9:
                actual_xy = [float(result[6]), float(result[7])]
                actual_z = float(result[8])
        except Exception:
            actual_xy = None
            actual_z = None
        if actual_z is not None and actual_z <= pre_z + 0.5:
            # Already at or above the retract height; nothing to lift.
            if logger:
                logger.info(
                    "[TableB DXF Robot] stop-lift skipped: already at/above pre-height "
                    "(actual Z=%.2f, pre_z=%.2f).",
                    actual_z, pre_z,
                )
            return
        # Prefer the actual XY (straight-up lift); fall back to the last commanded XY if the
        # position read failed.
        lift_xy = actual_xy if actual_xy is not None else current_xy
        lift_pose = _pose_from_xy(lift_xy, pre_z, rxyz)
        if logger:
            logger.info(
                "[TableB DXF Robot] stop-lift: retracting tool to pre-height %.2f mm with no force.",
                pre_z,
            )
        # communicate() and its wait loop both bail on a latched stop flag, so a MoveL issued
        # while stop is active would be a no-op. The lift IS the intended stop action, so we
        # clear the flag only for the duration of this single retract move, then re-latch it in
        # the finally so the caller's "stopped" semantics (raise + cleanup) are preserved.
        # Motion is already halted and force control already OFF at this point.
        clear_stop()
        try:
            communicate(
                cps=cps,
                config=config,
                point=lift_pose,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed(config),
                velocity_profile="robotspeed",
                wait=True,
            )
        finally:
            request_stop()
    except Exception as exc:
        if logger:
            logger.warning("[TableB DXF Robot] stop-lift failed (continuing): %s", exc)


def run_force_xy_path(cps: Any, config: dict[str, Any], step: dict[str, Any]) -> None:
    """Run one approved XY path with Table B Z+ force control.

    The DXF planner owns XY/J7 reachability. This runner only converts each
    robot-base XY point into the robot's 6D Cartesian command:
    [X, Y, Z, 180, 0, 0].
    """
    physical_tool = int(step.get("physical_tool") or 0)
    if physical_tool == 2:
        raise TableBDxfRobotExecutionError(
            "Tool 2 Table B DXF execution is not implemented in the XY force runner."
        )
    if physical_tool not in (1, 3, 4):
        raise TableBDxfRobotExecutionError(f"Unsupported physical tool: {physical_tool}")

    points = _path_points(step)
    _assert_robot_base_points_reachable(step, physical_tool, points)
    is_closed = _is_closed_path(step, points)
    tcp = tcp_for_physical_tool(config, physical_tool)
    ucs = table_b_ucs(config)
    rxyz = orientation(config)
    pre_z = preheight_z_mm(config)
    # Sanding-point Z is commanded BELOW the surface (pre_z + overshoot; +Z is toward the
    # surface on Table B) so the trajectory pushes the tool INTO the surface and force
    # control merely caps the pressure. Commanding the old Z=pre_z (above the surface) let
    # the position setpoint pull the tool up faster than the Z force loop could correct at
    # fast XY speed, so it hovered. pre_z itself stays above the surface for the pre-approach
    # and the lift-off; only the in-contact sanding points use the overshot Z.
    sanding_overshoot_mm = force_sanding_overshoot_mm(config)
    force_control_z = pre_z + sanding_overshoot_mm
    # Fast pre-approach: the arm arrives at pre_z in free space, then force control
    # crawls the WHOLE remaining air gap to the surface at the slow search velocity.
    # That gap varies per toolpath (surface height + residual settle), so at a fixed
    # search speed the contact time swings randomly. We close most of the gap with a
    # fast MoveL to (pre_z + fast_approach) — +Z is toward the surface on Table B —
    # then let force control search only the last few mm. Consistent, still safe.
    fast_approach_mm = force_fast_approach_mm(config)
    fast_approach_z = pre_z + fast_approach_mm
    force = float(step.get("recipe", {}).get("force") or 0)
    cycles = int(step.get("recipe", {}).get("cycle") or 1)
    axis7_position = step.get("axis7_position_mm")
    flush_every = _motion_queue_flush_every(config)
    simultaneous_j7_robot = _simultaneous_j7_robot_enabled(config)
    current_pose = _read_current_coord_pose(cps, tcp, ucs)
    low_y_j7_idle_before_prepoint = _requires_j7_idle_before_prepoint(physical_tool, current_pose)
    high_negative_x_escape_before_low_y = _requires_high_negative_x_escape_before_low_y(
        physical_tool, current_pose, points[0]
    )
    joint_j7_idle_before_prepoint, current_joints, joint_guard_reason = _joint_transition_guard(cps, config)
    j7_idle_before_prepoint = (
        low_y_j7_idle_before_prepoint
        or high_negative_x_escape_before_low_y
        or joint_j7_idle_before_prepoint
    )
    current_y = current_pose[1] if current_pose and len(current_pose) >= 2 else None
    current_j3 = current_joints[2] if current_joints and len(current_joints) >= 3 else None

    if force <= 0:
        raise TableBDxfRobotExecutionError(
            f"Path {step.get('path_id')} has invalid force: {force}"
        )
    if cycles <= 0:
        return

    _raise_if_stop_requested(cps, config, "before path start")

    _log(
        config,
        "[TableB DXF Robot] path=%s tool=%s points=%s cycles=%s mode=%s flushEvery=%s simultaneousJ7=%s j7IdleBeforePrepoint=%s lowYGuard=%s highNegXGuard=%s jointGuard=%s currentY=%s currentJ3=%s force=%.1f j7=%s",
        step.get("path_id"),
        physical_tool,
        len(points),
        cycles,
        "loop" if is_closed else "pingpong",
        flush_every,
        simultaneous_j7_robot,
        j7_idle_before_prepoint,
        low_y_j7_idle_before_prepoint,
        high_negative_x_escape_before_low_y,
        joint_guard_reason,
        "None" if current_y is None else f"{float(current_y):.1f}",
        "None" if current_j3 is None else f"{float(current_j3):.1f}",
        force,
        axis7_position,
    )

    start_pre_pose = _pose_from_xy(points[0], pre_z, rxyz)
    if axis7_position is not None and simultaneous_j7_robot and not j7_idle_before_prepoint:
        j7_result = communicate(
            cps=cps,
            config=config,
            point=None,
            tcp=tcp,
            ucs=ucs,
            seventh=axis7_position,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True,
        )
        if j7_result is None:
            raise TableBDxfRobotExecutionError(
                f"7th-axis async move failed for path {step.get('path_id')}."
            )
        communicate(
            cps=cps,
            config=config,
            point=start_pre_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=True,
        )
    else:
        # Non-simultaneous / same-7th-axis / low-Y toolpath switch. Previously this bundled
        # seventh=axis7 into the pre-pose MoveL. When J7 was already at that station (a
        # same-station switch), the bundled move's "done" state could settle on J7 (already
        # idle) rather than the arm, so at high speed force control started while the arm was
        # still moving and it hovered searching from the wrong point. Mirror the working J7
        # branch: position J7 first only if it must move, THEN a DEDICATED arm-only MoveL to
        # the pre-pose (seventh=-1, wait=True) so the arm fully settles before force control.
        #
        # If the current TCP pose is already in the low-Y/bottom constrained zone, or a
        # joint is near its physical limit, async J7 travel can fold the arm into a limit.
        # Park the rail first only in those guarded cases; normal high-Y transitions can
        # remain simultaneous.
        j7_positioned_by_escape = False
        if axis7_position is not None and low_y_j7_idle_before_prepoint:
            j7_positioned_by_escape = _escape_low_y_during_j7(
                cps,
                config,
                physical_tool=physical_tool,
                current_pose=current_pose,
                axis7_position=axis7_position,
                pre_z=pre_z,
                rxyz=rxyz,
                tcp=tcp,
                ucs=ucs,
            )
        elif axis7_position is not None and high_negative_x_escape_before_low_y:
            j7_positioned_by_escape = _escape_high_negative_x_during_j7(
                cps,
                config,
                physical_tool=physical_tool,
                current_pose=current_pose,
                axis7_position=axis7_position,
                pre_z=pre_z,
                rxyz=rxyz,
                tcp=tcp,
                ucs=ucs,
            )
        if axis7_position is not None and not j7_positioned_by_escape:
            communicate(
                cps=cps,
                config=config,
                point=None,
                tcp=tcp,
                ucs=ucs,
                seventh=axis7_position,
                speed=robot_speed(config),
                velocity_profile="robotspeed",
                wait=True,
                require_seventh_ok=True,
            )
        communicate(
            cps=cps,
            config=config,
            point=start_pre_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=False,
        )

    vibration_on = False
    force_applied = False
    current_xy = points[0]
    stopped = False
    try:
        _raise_if_stop_requested(cps, config, "before force search")
        # Confirm the arm has truly stopped at the pre-pose before force control, so a fast
        # robotspeed approach that still reports motion-done can't make the tool hover.
        _wait_until_arm_settled(cps, config)
        # Fast pre-approach: close most of the (variable) air gap at full robot speed with a
        # plain MoveL straight down to a fixed standoff above the surface, wait=True. This
        # leaves the slow 7 mm/s force search only the last few mm to crawl, so contact time
        # is consistent instead of randomly long. Skipped when disabled (fast_approach_mm=0).
        if fast_approach_mm > 0:
            fast_result = communicate(
                cps=cps,
                config=config,
                point=_pose_from_xy(points[0], fast_approach_z, rxyz),
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed(config),
                velocity_profile="robotspeed",
                wait=True,
                require_seventh_ok=False,
            )
            if fast_result is None:
                raise TableBDxfRobotExecutionError(
                    f"Fast pre-approach move failed for path {step.get('path_id')}."
                )
            # Make sure the fast move has fully stopped before the slow search begins, so a
            # coasting arm can't make force control start from the wrong point again.
            _wait_until_arm_settled(cps, config)
        ok = putForceZplus(
            cps=cps,
            force=force,
            tcp=tcp,
            ucs=ucs,
            config=config,
            search_linear_velocity=_force_search_linear_velocity(config, physical_tool),
            contact_force_threshold=_contact_force_threshold(config, physical_tool),
        )
        if not ok:
            raise TableBDxfRobotExecutionError(
                f"Force search failed for path {step.get('path_id')}."
            )
        force_applied = True
        # Force control only crossed the contact threshold; wait for it to HOLD near the full
        # commanded force before the toolpath moves, so the tool doesn't skim while force ramps.
        _wait_force_stabilized(
            cps, config, force, _contact_force_threshold(config, physical_tool) or force
        )

        vibration_on = bool(turn_vibration_on(cps))
        if not vibration_on:
            raise TableBDxfRobotExecutionError(
                f"Vibration did not turn on for path {step.get('path_id')}."
            )

        for cycle_index in range(1, cycles + 1):
            _log(
                config,
                "[TableB DXF Robot] path=%s cycle=%s/%s",
                step.get("path_id"),
                cycle_index,
                cycles,
            )
            cycle_points = _cycle_points(points, is_closed, cycle_index)
            for point_index, xy in enumerate(cycle_points, start=1):
                _raise_if_stop_requested(cps, config, "during sanding path")
                wait_for_point = _should_wait_for_motion_point(
                    point_index, len(cycle_points), flush_every
                )
                communicate(
                    cps=cps,
                    config=config,
                    point=_pose_from_xy(xy, force_control_z, rxyz),
                    tcp=tcp,
                    ucs=ucs,
                    seventh=-1,
                    speed=sanding_speed(config),
                    velocity_profile="sandingspeed",
                    speed_mode="linear",
                    wait=wait_for_point,
                )
                current_xy = xy
    except TableBDxfRobotStopRequested:
        stopped = True
        raise
    finally:
        if vibration_on:
            turn_vibration_off(cps)
        if force_applied:
            releaseForce(cps, config, wait_for_blending=False)
        if stopped or stop_requested():
            # CNC-style stop: halt motion, turn OFF force control, then lift the tool straight
            # up to the pre-height (-17) with NO force, so it releases the surface immediately
            # and never drags or leaves marks. From -17 the operator can re-send the task or
            # home. Force control is disabled inside _stop_active_motion BEFORE this lift, so
            # the retract MoveL is a pure position move (no pressure into the surface).
            _stop_active_motion(cps, config, "after stop cleanup")
            _lift_to_preheight_no_force(
                cps, config, current_xy, pre_z, rxyz, tcp, ucs, physical_tool
            )
            if not stopped:
                raise TableBDxfRobotStopRequested("Table B DXF operation stopped by user.")
        else:
            lift_pose = _pose_from_xy(current_xy, pre_z, rxyz)
            communicate(
                cps=cps,
                config=config,
                point=lift_pose,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed(config),
                velocity_profile="robotspeed",
                wait=True,
            )
