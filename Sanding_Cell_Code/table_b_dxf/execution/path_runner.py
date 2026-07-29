from __future__ import annotations

from typing import Any

from Server_Better_V2 import (
    _get_tool_in_hand,
    communicate,
    getTool11,
    keepTool11,
    moveOnlyJ6r,
    putForceZplus,
    releaseForce,
    stop_requested,
    turn_vibration_off,
    turn_vibration_on,
)

from .motion_config import (
    orientation,
    preheight_z_mm,
    robot_speed,
    sanding_speed,
    table_b_ucs,
    tcp_for_physical_tool,
)


class TableBDxfRobotExecutionError(RuntimeError):
    """Raised when a Table B DXF robot path cannot be executed safely."""


class TableBDxfRobotStopRequested(TableBDxfRobotExecutionError):
    """Raised to abort the whole Table B DXF run after operator Stop."""


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


def _log(config: dict[str, Any], message: str, *args: Any) -> None:
    logger = config.get("logger") if isinstance(config, dict) else None
    if logger:
        logger.info(message, *args)


def _stop_active_motion(cps: Any, config: dict[str, Any], reason: str) -> None:
    _log(config, "[TableB DXF Robot] stop requested: %s", reason)
    try:
        cps.HRIF_GrpStop(0, 0)
    except Exception:
        pass
    try:
        result = []
        cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
    except Exception:
        pass
    try:
        cps.HRIF_SetForceControlState(0, 0, 0)
    except Exception:
        pass
    try:
        turn_vibration_off(cps)
    except Exception:
        pass
    try:
        cps.HRIF_SetBoxDO(0, 7, 0)  # tool spin off
    except Exception:
        pass
    try:
        cps.HRIF_SetBoxDO(0, 3, 0)  # laser off
    except Exception:
        pass


def _raise_if_stop_requested(cps: Any, config: dict[str, Any], reason: str) -> None:
    if stop_requested():
        _stop_active_motion(cps, config, reason)
        raise TableBDxfRobotStopRequested("Table B DXF operation stopped by user.")


TOOL_1_3_JOINT_CORRECTIONS: dict[tuple[int, int], dict[str, float]] = {
    (1, 3): {"J1": 30.0, "J6": -90.0},
    (3, 1): {"J1": -30.0, "J6": 90.0},
}


def _normalize_tool_in_hand(raw_tool: Any) -> int | None:
    if raw_tool == 0:
        return None
    try:
        tool = int(raw_tool)
    except (TypeError, ValueError) as error:
        raise TableBDxfRobotExecutionError(
            f"Unable to read mounted tool state: {raw_tool!r}."
        ) from error
    if tool in (1, 2, 3, 4):
        return tool
    raise TableBDxfRobotExecutionError(
        f"Unknown mounted tool state from CI sensors: {raw_tool!r}."
    )


def _read_tool_in_hand(cps: Any) -> int | None:
    return _normalize_tool_in_hand(_get_tool_in_hand(cps))


def _j7_reference_tool(tool: int | None, fallback_tool: int = 3) -> int:
    # J7-only commands still require a TCP/UCS argument, but Tool 2 has no DXF
    # force-runner TCP. Use a supported reference TCP; the command only moves rail.
    if tool in (1, 3, 4):
        return int(tool)
    return int(fallback_tool if fallback_tool in (1, 3, 4) else 3)


def _apply_tool_transition_correction(
    cps: Any, config: dict[str, Any], current_tool: int, requested_tool: int
) -> None:
    correction = TOOL_1_3_JOINT_CORRECTIONS.get((current_tool, requested_tool))
    if not correction:
        return
    _log(
        config,
        "[TableB DXF Robot] Tool %s -> Tool %s joint correction J1=%+.1f J6=%+.1f",
        current_tool,
        requested_tool,
        correction["J1"],
        correction["J6"],
    )
    moveOnlyJ6r(
        cps,
        correction["J6"],
        config,
        J1=correction["J1"],
        wait=True,
    )


def _ensure_tool_in_hand(
    cps: Any, config: dict[str, Any], requested_tool: int, mounted_tool: int | None
) -> int:
    _raise_if_stop_requested(cps, config, "before tool-state read")
    sensor_tool = _read_tool_in_hand(cps)
    if mounted_tool is not None and sensor_tool != mounted_tool:
        _log(
            config,
            "[TableB DXF Robot] mounted-tool memory %s differs from CI sensor %s; using sensor",
            mounted_tool,
            sensor_tool,
        )
    current_tool = sensor_tool
    if current_tool == requested_tool:
        _log(config, "[TableB DXF Robot] Tool %s already mounted", requested_tool)
        return requested_tool

    _raise_if_stop_requested(cps, config, "before tool change")

    # All tool changes start with the rail at the calibrated tool station.
    _return_j7_to_zero(cps, config, _j7_reference_tool(current_tool, requested_tool))

    if current_tool is not None:
        _log(config, "[TableB DXF Robot] dropping Tool %s before picking Tool %s", current_tool, requested_tool)
        keepTool11(
            cps,
            toolNumber=current_tool,
            config=config,
            goToSafe=False,
            startFromSafe=True,
        )
        dropped_state = _read_tool_in_hand(cps)
        if dropped_state is not None:
            raise TableBDxfRobotExecutionError(
                f"Tool {current_tool} drop was not confirmed; CI still reports Tool {dropped_state}."
            )
        _apply_tool_transition_correction(cps, config, current_tool, requested_tool)
        start_from_safe = False
    else:
        _log(config, "[TableB DXF Robot] no tool mounted; picking Tool %s", requested_tool)
        start_from_safe = True

    success = getTool11(
        cps,
        toolNumber=requested_tool,
        config=config,
        startFromSafe=start_from_safe,
        exitToSafe=True,
    )
    if not success:
        raise TableBDxfRobotExecutionError(f"Tool {requested_tool} pick failed.")
    picked_state = _read_tool_in_hand(cps)
    if picked_state != requested_tool:
        raise TableBDxfRobotExecutionError(
            f"Tool {requested_tool} pick was not confirmed; CI reports {picked_state}."
        )
    return requested_tool


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
        communicate(
            cps=cps,
            config=config,
            point=start_pre_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=axis7_position if axis7_position is not None else -1,
            speed=robot_speed(config),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=axis7_position is not None,
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


def _return_j7_to_zero(cps: Any, config: dict[str, Any], physical_tool: int) -> None:
    _raise_if_stop_requested(cps, config, "before J7 return-to-zero")
    tcp = tcp_for_physical_tool(config, physical_tool)
    ucs = table_b_ucs(config)
    _log(config, "[TableB DXF Robot] returning J7 to 0 mm after tool %s batch", physical_tool)
    result = communicate(
        cps=cps,
        config=config,
        point=None,
        tcp=tcp,
        ucs=ucs,
        seventh=0,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=True,
    )
    if result is None:
        raise TableBDxfRobotExecutionError(
            f"Failed to return J7 to 0 mm after tool {physical_tool} batch."
        )


def _move_arm_to_task_home(cps: Any, config: dict[str, Any], reason: str) -> None:
    _raise_if_stop_requested(cps, config, f"before arm home move ({reason})")
    _log(config, "[TableB DXF Robot] moving 6-axis arm directly to safePoint: %s", reason)
    result = communicate(
        cps=cps,
        config=config,
        point=config["point"]["safePoint"],
        tcp=config["coords"]["tcpDefault"],
        ucs=config["coords"]["ucsDefault"],
        seventh=-1,
        speed=robot_speed(config),
        velocity_profile="robotspeed",
        wait=True,
    )
    if result is None:
        raise TableBDxfRobotExecutionError("Failed to move 6-axis arm to safePoint.")


def _return_to_home_with_tool(cps: Any, config: dict[str, Any], physical_tool: int) -> None:
    """Return the rail and arm to home/safe while keeping the mounted tool."""
    _move_arm_to_task_home(cps, config, "before J7 return")
    _return_j7_to_zero(cps, config, physical_tool)
    _move_arm_to_task_home(cps, config, "after J7 return")

def run_robot_plan(cps: Any, config: dict[str, Any], plan: dict[str, Any]) -> dict[str, Any]:
    """Execute the resolved Table B DXF run plan, including tool changes."""
    if cps is None:
        raise TableBDxfRobotExecutionError("A connected CPS instance is required for real execution.")

    executed_steps = 0
    mounted_tool: int | None = None
    for batch in plan.get("batches") or []:
        _raise_if_stop_requested(cps, config, "before tool batch")
        physical_tool = int(batch.get("physical_tool") or 0)
        if physical_tool == 2:
            raise TableBDxfRobotExecutionError(
                "Tool 2 DXF execution is not implemented yet. Disable side/edge outside for this run."
            )
        if physical_tool not in (1, 3, 4):
            raise TableBDxfRobotExecutionError(f"Unsupported physical tool batch: {physical_tool}")

        _log(config, "[TableB DXF Robot] === TOOL %s BATCH START ===", physical_tool)
        mounted_tool = _ensure_tool_in_hand(cps, config, physical_tool, mounted_tool)
        _move_arm_to_task_home(cps, config, f"before tool {physical_tool} operation")

        for step in batch.get("steps") or []:
            _raise_if_stop_requested(cps, config, "before next path")
            run_force_xy_path(cps, config, step)
            executed_steps += 1

        _return_to_home_with_tool(cps, config, physical_tool)
        _log(config, "[TableB DXF Robot] === TOOL %s BATCH COMPLETE ===", physical_tool)

    _log(config, "[TableB DXF Robot] run complete; keeping mounted tool %s at J7=0", mounted_tool)
    return {"executed_steps": executed_steps, "tool_kept_in_hand": mounted_tool}


