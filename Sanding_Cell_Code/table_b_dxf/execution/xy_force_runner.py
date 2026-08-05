from __future__ import annotations

from typing import Any

from Server_Better_V2 import (
    communicate,
    putForceZplus,
    releaseForce,
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
from .motion_config import (
    orientation,
    preheight_z_mm,
    robot_speed,
    sanding_speed,
    table_b_ucs,
    tcp_for_physical_tool,
)


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


def _should_wait_for_motion_point(point_index: int, total_points: int, flush_every: int) -> bool:
    if point_index >= total_points:
        return True
    return flush_every > 0 and point_index % flush_every == 0


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
    is_closed = _is_closed_path(step, points)
    tcp = tcp_for_physical_tool(config, physical_tool)
    ucs = table_b_ucs(config)
    rxyz = orientation(config)
    pre_z = preheight_z_mm(config)
    force_control_z = pre_z
    force = float(step.get("recipe", {}).get("force") or 0)
    cycles = int(step.get("recipe", {}).get("cycle") or 1)
    axis7_position = step.get("axis7_position_mm")
    flush_every = _motion_queue_flush_every(config)
    simultaneous_j7_robot = _simultaneous_j7_robot_enabled(config)

    if force <= 0:
        raise TableBDxfRobotExecutionError(
            f"Path {step.get('path_id')} has invalid force: {force}"
        )
    if cycles <= 0:
        return

    _raise_if_stop_requested(cps, config, "before path start")

    _log(
        config,
        "[TableB DXF Robot] path=%s tool=%s points=%s cycles=%s mode=%s flushEvery=%s simultaneousJ7=%s force=%.1f j7=%s",
        step.get("path_id"),
        physical_tool,
        len(points),
        cycles,
        "loop" if is_closed else "pingpong",
        flush_every,
        simultaneous_j7_robot,
        force,
        axis7_position,
    )

    start_pre_pose = _pose_from_xy(points[0], pre_z, rxyz)
    if axis7_position is not None and simultaneous_j7_robot:
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
        # Non-simultaneous / same-7th-axis toolpath switch. Previously this bundled
        # seventh=axis7 into the pre-pose MoveL. When J7 was already at that station (a
        # same-station switch), the bundled move's "done" state could settle on J7 (already
        # idle) rather than the arm, so at high speed force control started while the arm was
        # still moving and it hovered searching from the wrong point. Mirror the working J7
        # branch: position J7 first only if it must move, THEN a DEDICATED arm-only MoveL to
        # the pre-pose (seventh=-1, wait=True) so the arm fully settles before force control.
        if axis7_position is not None:
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
            _stop_active_motion(cps, config, "after stop cleanup")
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
