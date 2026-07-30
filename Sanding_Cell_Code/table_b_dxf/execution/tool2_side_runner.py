from __future__ import annotations

from collections.abc import Callable
from typing import Any, NamedTuple

from Server_Better_V2 import (
    communicate,
    moveOnlyJ6r,
    putForceXminus,
    putForceXplus,
    putForceYminus1,
    putForceYplus1,
    releaseForce,
    turn_vibration_off,
    turn_vibration_on,
    waitForBlending,
)

from .common import (
    TableBDxfRobotExecutionError,
    TableBDxfRobotStopRequested,
    log as _log,
    raise_if_stop_requested as _raise_if_stop_requested,
    stop_active_motion as _stop_active_motion,
)
from .motion_config import robot_speed, sanding_speed
from .tool2_recovery import clear_tool2_recovery_state, record_tool2_recovery_state
from .tool2_side_geometry import (
    TOOL2_APPROACH_OUTWARD_MM,
    TOOL2_CONTACT_Z_MM,
    TOOL2_LEFT_PREFORCE_LOCAL_X_MM,
    tool2_lift_z_for_side,
    split_tool2_bottom_segments,
    tool2_left_axis7_position,
    tool2_top_local_x_span_at_y,
)


TOOL2_OPERATION_SIDE = "side"
TOOL2_OPERATION_EDGE = "edgeOutside"
TOOL2_EDGE_RY_DEG = -22.0
TOOL2_EDGE_APPROACH_OUTWARD_MM = 5.0
TOOL2_BOTTOM_SIDE_EXIT_Y_MM = 80.0
TOOL2_BOTTOM_SIDE_EXIT_Z_MM = -80.0
TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM = 100.0

TOOL2_SIDE_RZ_DEG: dict[str, float] = {
    "right": 0.0,
    "bottom": 90.0,
    "left": 180.0,
    "top": -90.0,
}

# Explicit wrist direction for side changes. Positive J6 is clockwise on this cell.
TOOL2_SIDE_TRANSITION_J6_DEG: dict[tuple[str, str], float] = {
    ("bottom", "top"): 180.0,
    ("top", "bottom"): -180.0,
    ("top", "right"): -270.0,
    ("right", "top"): 270.0,
    ("bottom", "right"): -90.0,
    ("right", "bottom"): 90.0,
    ("bottom", "left"): 90.0,
    ("left", "bottom"): -90.0,
    ("left", "top"): 90.0,
    ("top", "left"): -90.0,
}

# Tool 2 sands the door thickness. The force direction is tied to the physical
# side being sanded, not to the XY surface force runner used by tools 1/3/4.
TOOL2_SIDE_FORCE_FUNCTIONS: dict[str, Callable[..., Any]] = {
    "right": putForceXplus,
    "bottom": putForceYplus1,
    "left": putForceXminus,
    "top": putForceYminus1,
}

TOOL2_SIDE_FORCE_NAMES: dict[str, str] = {
    side: force_func.__name__
    for side, force_func in TOOL2_SIDE_FORCE_FUNCTIONS.items()
}


class _Tool2HorizontalSegment(NamedTuple):
    side: str
    axis7: float
    local_start_x: float
    local_end_x: float
    y: float


class _Tool2Position(NamedTuple):
    side: str | None
    x: float
    y: float
    rz: float


def tool2_side_rz(side: str) -> float:
    try:
        return TOOL2_SIDE_RZ_DEG[side]
    except KeyError as error:
        raise TableBDxfRobotExecutionError(f"Unknown Tool 2 side for RZ: {side!r}") from error


def tool2_side_force_function(side: str) -> Callable[..., Any]:
    try:
        return TOOL2_SIDE_FORCE_FUNCTIONS[side]
    except KeyError as error:
        raise TableBDxfRobotExecutionError(f"Unknown Tool 2 side for force control: {side!r}") from error


def _tool2_tcp(config: dict[str, Any]) -> str:
    coords = config.get("coords") or {}
    tcp = coords.get("tcpSideTool") or coords.get("tcptool2plane2")
    if not tcp:
        raise TableBDxfRobotExecutionError("Missing Tool 2 TCP config: coords.tcpSideTool or coords.tcptool2plane2")
    return str(tcp)


def _tool2_ucs(config: dict[str, Any]) -> str:
    try:
        return str(config["coords"]["ucsTable2"])
    except KeyError as error:
        raise TableBDxfRobotExecutionError("Missing Tool 2 UCS config: coords.ucsTable2") from error


def _pose(x: float, y: float, z: float, rz: float, *, ry: float = 0.0) -> list[float]:
    return [float(x), float(y), float(z), 0.0, float(ry), float(rz)]


def _lift_z_for_transition(previous_position: _Tool2Position | None, next_side: str) -> float:
    if next_side == "bottom":
        return tool2_lift_z_for_side("bottom")
    if previous_position is not None and previous_position.side == "bottom" and next_side == "left":
        return tool2_lift_z_for_side("bottom")
    return tool2_lift_z_for_side(next_side)


def _is_edge_operation(operation_mode: str) -> bool:
    return operation_mode == TOOL2_OPERATION_EDGE


def _sanding_pose(operation_mode: str, x: float, y: float, z: float, rz: float) -> list[float]:
    ry = TOOL2_EDGE_RY_DEG if _is_edge_operation(operation_mode) else 0.0
    return _pose(x, y, z, rz, ry=ry)


def _tool2_outward_offset_for_operation(operation_mode: str) -> float:
    return TOOL2_EDGE_APPROACH_OUTWARD_MM if _is_edge_operation(operation_mode) else TOOL2_APPROACH_OUTWARD_MM


def _horizontal_y_for_operation(side: str, y: float, operation_mode: str) -> float:
    offset = _tool2_outward_offset_for_operation(operation_mode)
    if side == "bottom":
        return -offset
    if side == "top":
        return y - TOOL2_APPROACH_OUTWARD_MM + offset
    return y


def _vertical_x_for_operation(side: str, x: float, operation_mode: str) -> float:
    offset_delta = TOOL2_APPROACH_OUTWARD_MM - _tool2_outward_offset_for_operation(operation_mode)
    if side == "right":
        return -_tool2_outward_offset_for_operation(operation_mode)
    if side == "left":
        return x - offset_delta
    return x


def _mark_tool2_recovery(side: str, pose: list[float]) -> None:
    record_tool2_recovery_state(side, pose, active=True)


def _left_side_bottom_lift_y(side: str, end_y: float) -> float | None:
    if side == "left" and abs(float(end_y)) <= 1e-6:
        return TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM
    return None


def _move(
    cps: Any,
    config: dict[str, Any],
    point: list[float] | None,
    *,
    seventh: float = -1,
    speed: float | None = None,
    velocity_profile: str = "robotspeed",
    wait: bool = True,
    speed_mode: str | None = None,
    require_seventh_ok: bool = False,
) -> None:
    result = communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=_tool2_tcp(config),
        ucs=_tool2_ucs(config),
        seventh=seventh,
        speed=robot_speed(config) if speed is None else speed,
        velocity_profile=velocity_profile,
        wait=wait,
        speed_mode=speed_mode,
        require_seventh_ok=require_seventh_ok,
    )
    if result is None:
        raise TableBDxfRobotExecutionError(f"Tool 2 communicate failed for point={point} seventh={seventh}.")


def _move_axis7_and_lifted_prepoint(
    cps: Any,
    config: dict[str, Any],
    *,
    axis7: float,
    prepoint: list[float],
) -> None:
    """Move J7 and arm together, then block at the lifted prepoint before contact.

    The J7 command is asynchronous. The following wait=True prepoint MoveL acts as
    the force/contact barrier because communicate() waits for pending J7 idle.
    """
    _move(
        cps,
        config,
        None,
        seventh=axis7,
        velocity_profile="robotspeed",
        wait=False,
        require_seventh_ok=True,
    )
    _move(
        cps,
        config,
        prepoint,
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=True,
    )


def _transition_mid_y(y_total: float | None) -> float | None:
    if y_total is None:
        return None
    return max(0.0, float(y_total) / 2.0)


def _prepare_bottom_side_exit_before_j6(
    cps: Any,
    config: dict[str, Any],
    previous_position: _Tool2Position,
    next_side: str,
    y_total: float | None = None,
) -> None:
    if previous_position.side != "bottom" or next_side not in {"top", "left"}:
        return
    rz = TOOL2_SIDE_RZ_DEG["bottom"]
    x = previous_position.x
    y = previous_position.y
    clearance_y = _transition_mid_y(y_total) if next_side == "top" else None
    if clearance_y is None:
        clearance_y = TOOL2_BOTTOM_SIDE_EXIT_Y_MM
    _log(
        config,
        "[TableB DXF Tool2] bottom exit before %s: prepoint Z=0, lift Z=-50, clearance Y=%.3f Z=-80 at x=%.3f",
        next_side,
        clearance_y,
        x,
    )
    _move(cps, config, _pose(x, y, TOOL2_CONTACT_Z_MM, rz), velocity_profile="robotspeed", wait=True)
    _move(cps, config, _pose(x, y, tool2_lift_z_for_side("bottom"), rz), velocity_profile="robotspeed", wait=True)
    _move(
        cps,
        config,
        _pose(x, clearance_y, TOOL2_BOTTOM_SIDE_EXIT_Z_MM, rz),
        velocity_profile="robotspeed",
        wait=True,
    )


def _apply_side_transition_j6(
    cps: Any,
    config: dict[str, Any],
    previous_position: _Tool2Position | None,
    next_side: str,
    y_total: float | None = None,
) -> None:
    if previous_position is None or previous_position.side is None or previous_position.side == next_side:
        return
    j6_delta = TOOL2_SIDE_TRANSITION_J6_DEG.get((previous_position.side, next_side))
    if j6_delta is None or abs(j6_delta) <= 1e-6:
        return
    if previous_position.side == "top" and next_side == "bottom":
        mid_y = _transition_mid_y(y_total)
        if mid_y is not None:
            _log(
                config,
                "[TableB DXF Tool2] top exit before bottom: travel to mid Y=%.3f at Z=%.3f before J6",
                mid_y,
                tool2_lift_z_for_side("top"),
            )
            _move(
                cps,
                config,
                _pose(previous_position.x, mid_y, tool2_lift_z_for_side("top"), TOOL2_SIDE_RZ_DEG["top"]),
                velocity_profile="robotspeed",
                wait=True,
            )
    _prepare_bottom_side_exit_before_j6(cps, config, previous_position, next_side, y_total)
    _log(
        config,
        "[TableB DXF Tool2] side transition %s -> %s using J6 delta %.1f",
        previous_position.side,
        next_side,
        j6_delta,
    )
    moveOnlyJ6r(cps, j6_delta, config, wait=True)


def _apply_force_and_vibration(cps: Any, config: dict[str, Any], side: str, force: float) -> None:
    force_func = tool2_side_force_function(side)
    ok = force_func(
        cps=cps,
        force=force,
        tcp=_tool2_tcp(config),
        ucs=_tool2_ucs(config),
        config=config,
    )
    # Existing force helpers return None on normal success. Treat only explicit
    # False as failure so the sanding MoveL can start after force contact.
    if ok is False:
        raise TableBDxfRobotExecutionError(f"Tool 2 {side} force search failed using {force_func.__name__}.")
    if not turn_vibration_on(cps):
        raise TableBDxfRobotExecutionError(f"Tool 2 {side} vibration did not turn on.")


def _release_force_and_vibration(cps: Any, config: dict[str, Any], side: str) -> None:
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    releaseForce(cps=cps, config=config)
    _raise_if_stop_requested(cps, config, f"after Tool 2 {side} segment")


def _force_ready_at_contact(
    cps: Any,
    config: dict[str, Any],
    side: str,
    operation_mode: str,
    x: float,
    y: float,
    rz: float,
) -> list[float]:
    # All travel/placement uses RY=0. Edge bends only after the contact XY/Z is reached.
    base_contact = _pose(x, y, TOOL2_CONTACT_Z_MM, rz, ry=0.0)
    _move(cps, config, base_contact, velocity_profile="robotspeed", wait=True)
    if _is_edge_operation(operation_mode):
        edge_contact = _pose(x, y, TOOL2_CONTACT_Z_MM, rz, ry=TOOL2_EDGE_RY_DEG)
        _move(cps, config, edge_contact, velocity_profile="robotspeed", wait=True)
        _mark_tool2_recovery(side, edge_contact)
        return edge_contact
    _mark_tool2_recovery(side, base_contact)
    return base_contact


def _bounds_from_steps(steps: list[dict[str, Any]]) -> tuple[float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for step in steps:
        for point in step.get("viewer_points") or step.get("points") or []:
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                xs.append(float(point[0]))
                ys.append(float(point[1]))
    if not xs or not ys:
        raise TableBDxfRobotExecutionError("Tool 2 batch has no side points to infer door size.")
    return max(xs) - min(xs), max(ys) - min(ys)


def _run_horizontal_segment(
    cps: Any,
    config: dict[str, Any],
    force: float,
    segment: _Tool2HorizontalSegment,
    operation_mode: str,
    lift_z: float | None = None,
    *,
    enter_from_contact: bool = False,
    lift_after: bool = True,
    backoff_before_entry: bool = False,
    reverse_path: bool = False,
) -> _Tool2Position:
    side = segment.side
    lift_z = tool2_lift_z_for_side(side) if lift_z is None else float(lift_z)
    rz = TOOL2_SIDE_RZ_DEG[side]
    y = _horizontal_y_for_operation(side, segment.y, operation_mode)
    side_safe_y = _horizontal_y_for_operation(side, segment.y, TOOL2_OPERATION_SIDE)
    path_start_x = segment.local_end_x if reverse_path else segment.local_start_x
    path_end_x = segment.local_start_x if reverse_path else segment.local_end_x
    _log(
        config,
        "[TableB DXF Tool2] %s %s segment j7=%.3f x %.3f -> %.3f y=%.3f enter_from_contact=%s lift_after=%s",
        operation_mode,
        side,
        segment.axis7,
        path_start_x,
        path_end_x,
        y,
        enter_from_contact,
        lift_after,
    )
    if enter_from_contact:
        _move(
            cps,
            config,
            None,
            seventh=segment.axis7,
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True,
        )
        if backoff_before_entry:
            _move(
                cps,
                config,
                _pose(path_start_x, side_safe_y, TOOL2_CONTACT_Z_MM, rz),
                velocity_profile="robotspeed",
                wait=True,
                require_seventh_ok=True,
            )
        _move(
            cps,
            config,
            _pose(path_start_x, y, TOOL2_CONTACT_Z_MM, rz),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=True,
        )
    else:
        _move_axis7_and_lifted_prepoint(
            cps,
            config,
            axis7=segment.axis7,
            prepoint=_pose(path_start_x, y, lift_z, rz),
        )
    _force_ready_at_contact(cps, config, side, operation_mode, path_start_x, y, rz)
    _apply_force_and_vibration(cps, config, side, force)
    end_pose = _sanding_pose(operation_mode, path_end_x, y, TOOL2_CONTACT_Z_MM, rz)
    _mark_tool2_recovery(side, end_pose)
    _log(config, "[TableB DXF Tool2] %s %s sanding move to %s", operation_mode, side, end_pose)
    _move(
        cps,
        config,
        end_pose,
        speed=sanding_speed(config),
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )
    _release_force_and_vibration(cps, config, side)
    exit_y = y
    if _is_edge_operation(operation_mode):
        exit_y = side_safe_y
        safe_exit_pose = _pose(path_end_x, exit_y, TOOL2_CONTACT_Z_MM, rz, ry=0.0)
        _move(
            cps,
            config,
            safe_exit_pose,
            velocity_profile="robotspeed",
            wait=True,
        )
        _mark_tool2_recovery(side, safe_exit_pose)
    elif not lift_after:
        safe_exit_pose = _pose(path_end_x, side_safe_y, TOOL2_CONTACT_Z_MM, rz, ry=0.0)
        _move(
            cps,
            config,
            safe_exit_pose,
            velocity_profile="robotspeed",
            wait=True,
        )
        _mark_tool2_recovery(side, safe_exit_pose)
    if lift_after:
        _move(cps, config, _pose(path_end_x, exit_y, TOOL2_CONTACT_Z_MM, rz), velocity_profile="robotspeed", wait=True)
        _move(
            cps,
            config,
            _pose(path_end_x, exit_y, lift_z, rz),
            velocity_profile="robotspeed",
            wait=True,
        )
    return _Tool2Position(side, path_end_x, exit_y, rz)


def _run_horizontal_segment_operations(
    cps: Any,
    config: dict[str, Any],
    force_by_mode: dict[str, float],
    operation_modes: list[str],
    segment: _Tool2HorizontalSegment,
    previous_position: _Tool2Position | None = None,
    *,
    next_side: str | None = None,
    y_total: float | None = None,
) -> _Tool2Position:
    if previous_position is None and segment.side == "top":
        _log(config, "[TableB DXF Tool2] initial homing bottom-orientation -> top using J6 +180")
        moveOnlyJ6r(cps, 180.0, config, wait=True)
    _apply_side_transition_j6(cps, config, previous_position, segment.side, y_total)
    lift_z = _lift_z_for_transition(previous_position, segment.side)
    enter_from_contact = bool(previous_position and previous_position.side == "bottom" and segment.side == "bottom")
    keep_contact_after = segment.side == "bottom" and next_side == "bottom"
    last_position: _Tool2Position | None = None
    for index, operation_mode in enumerate(operation_modes):
        has_next_same_pass_operation = index + 1 < len(operation_modes)
        enter_from_contact_for_operation = (enter_from_contact and index == 0) or index > 0
        lift_after_operation = False if has_next_same_pass_operation else not keep_contact_after
        last_position = _run_horizontal_segment(
            cps,
            config,
            force_by_mode[operation_mode],
            segment,
            operation_mode,
            lift_z=lift_z,
            enter_from_contact=enter_from_contact_for_operation,
            lift_after=lift_after_operation,
            backoff_before_entry=index > 0 and _is_edge_operation(operation_mode),
            reverse_path=index > 0 and _is_edge_operation(operation_mode),
        )
    if last_position is None:
        raise TableBDxfRobotExecutionError("Tool 2 horizontal segment has no selected operations.")
    return last_position


def _run_vertical_side(
    cps: Any,
    config: dict[str, Any],
    force: float,
    *,
    side: str,
    axis7: float,
    x: float,
    start_y: float,
    end_y: float,
    operation_mode: str,
    lift_z: float | None = None,
    enter_from_contact: bool = False,
    lift_after: bool = True,
    backoff_before_entry: bool = False,
    reverse_path: bool = False,
) -> _Tool2Position:
    rz = TOOL2_SIDE_RZ_DEG[side]
    lift_z = tool2_lift_z_for_side(side) if lift_z is None else float(lift_z)
    base_x = x
    x = _vertical_x_for_operation(side, base_x, operation_mode)
    side_safe_x = _vertical_x_for_operation(side, base_x, TOOL2_OPERATION_SIDE)
    path_start_y = end_y if reverse_path else start_y
    path_end_y = start_y if reverse_path else end_y
    _log(
        config,
        "[TableB DXF Tool2] %s %s side j7=%.3f y %.3f -> %.3f enter_from_contact=%s lift_after=%s",
        operation_mode,
        side,
        axis7,
        start_y,
        end_y,
        enter_from_contact,
        lift_after,
    )
    if enter_from_contact:
        _move(
            cps,
            config,
            None,
            seventh=axis7,
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True,
        )
        if backoff_before_entry:
            _move(
                cps,
                config,
                _pose(side_safe_x, path_start_y, TOOL2_CONTACT_Z_MM, rz),
                velocity_profile="robotspeed",
                wait=True,
                require_seventh_ok=True,
            )
        _move(
            cps,
            config,
            _pose(x, path_start_y, TOOL2_CONTACT_Z_MM, rz),
            velocity_profile="robotspeed",
            wait=True,
            require_seventh_ok=True,
        )
    else:
        _move_axis7_and_lifted_prepoint(
            cps,
            config,
            axis7=axis7,
            prepoint=_pose(x, path_start_y, lift_z, rz),
        )
    _force_ready_at_contact(cps, config, side, operation_mode, x, path_start_y, rz)
    _apply_force_and_vibration(cps, config, side, force)
    end_pose = _sanding_pose(operation_mode, x, path_end_y, TOOL2_CONTACT_Z_MM, rz)
    _mark_tool2_recovery(side, end_pose)
    _log(config, "[TableB DXF Tool2] %s %s sanding move to %s", operation_mode, side, end_pose)
    _move(
        cps,
        config,
        end_pose,
        speed=sanding_speed(config),
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )
    _release_force_and_vibration(cps, config, side)
    exit_x = x
    if _is_edge_operation(operation_mode):
        exit_x = side_safe_x
        safe_exit_pose = _pose(exit_x, path_end_y, TOOL2_CONTACT_Z_MM, rz, ry=0.0)
        _move(cps, config, safe_exit_pose, velocity_profile="robotspeed", wait=True)
        _mark_tool2_recovery(side, safe_exit_pose)
    elif not lift_after:
        safe_exit_pose = _pose(side_safe_x, path_end_y, TOOL2_CONTACT_Z_MM, rz, ry=0.0)
        _move(cps, config, safe_exit_pose, velocity_profile="robotspeed", wait=True)
        _mark_tool2_recovery(side, safe_exit_pose)
    if lift_after:
        _move(cps, config, _pose(exit_x, path_end_y, TOOL2_CONTACT_Z_MM, rz), velocity_profile="robotspeed", wait=True)
        lift_y = _left_side_bottom_lift_y(side, path_end_y)
        if lift_y is not None:
            _move(cps, config, _pose(exit_x, lift_y, TOOL2_CONTACT_Z_MM, rz), velocity_profile="robotspeed", wait=True)
            _move(cps, config, _pose(exit_x, lift_y, lift_z, rz), velocity_profile="robotspeed", wait=True)
            return _Tool2Position(side, exit_x, lift_y, rz)
        _move(cps, config, _pose(exit_x, path_end_y, lift_z, rz), velocity_profile="robotspeed", wait=True)
    return _Tool2Position(side, exit_x, path_end_y, rz)


def _run_right_side(
    cps: Any,
    config: dict[str, Any],
    force: float,
    y_total: float,
    operation_mode: str,
    lift_z: float | None = None,
    *,
    enter_from_contact: bool = False,
    lift_after: bool = True,
    backoff_before_entry: bool = False,
    reverse_path: bool = False,
) -> _Tool2Position:
    return _run_vertical_side(
        cps,
        config,
        force,
        side="right",
        axis7=0.0,
        x=-15.0,
        start_y=y_total,
        end_y=0.0,
        operation_mode=operation_mode,
        lift_z=lift_z,
        enter_from_contact=enter_from_contact,
        lift_after=lift_after,
        backoff_before_entry=backoff_before_entry,
        reverse_path=reverse_path,
    )


def _run_left_side(
    cps: Any,
    config: dict[str, Any],
    force: float,
    x_total: float,
    y_total: float,
    operation_mode: str,
    lift_z: float | None = None,
    *,
    start_from_top: bool = False,
    enter_from_contact: bool = False,
    lift_after: bool = True,
    backoff_before_entry: bool = False,
    reverse_path: bool = False,
) -> _Tool2Position:
    start_y = y_total if start_from_top else 0.0
    end_y = 0.0 if start_from_top else y_total
    return _run_vertical_side(
        cps,
        config,
        force,
        side="left",
        axis7=tool2_left_axis7_position(x_total),
        x=TOOL2_LEFT_PREFORCE_LOCAL_X_MM,
        start_y=start_y,
        end_y=end_y,
        operation_mode=operation_mode,
        lift_z=lift_z,
        enter_from_contact=enter_from_contact,
        lift_after=lift_after,
        backoff_before_entry=backoff_before_entry,
        reverse_path=reverse_path,
    )


def _run_right_side_operations(
    cps: Any,
    config: dict[str, Any],
    force_by_mode: dict[str, float],
    operation_modes: list[str],
    y_total: float,
    previous_position: _Tool2Position | None = None,
) -> _Tool2Position:
    _apply_side_transition_j6(cps, config, previous_position, "right")
    lift_z = _lift_z_for_transition(previous_position, "right")
    last_position: _Tool2Position | None = None
    for index, operation_mode in enumerate(operation_modes):
        has_next_same_pass_operation = index + 1 < len(operation_modes)
        last_position = _run_right_side(
            cps,
            config,
            force_by_mode[operation_mode],
            y_total,
            operation_mode,
            lift_z=lift_z,
            enter_from_contact=index > 0,
            lift_after=not has_next_same_pass_operation,
            backoff_before_entry=index > 0 and _is_edge_operation(operation_mode),
            reverse_path=index > 0 and _is_edge_operation(operation_mode),
        )
    if last_position is None:
        raise TableBDxfRobotExecutionError("Tool 2 right side has no selected operations.")
    return last_position


def _run_left_side_operations(
    cps: Any,
    config: dict[str, Any],
    force_by_mode: dict[str, float],
    operation_modes: list[str],
    x_total: float,
    y_total: float,
    previous_position: _Tool2Position | None = None,
) -> _Tool2Position:
    _apply_side_transition_j6(cps, config, previous_position, "left")
    lift_z = _lift_z_for_transition(previous_position, "left")
    start_from_top = bool(previous_position and previous_position.side == "top")
    last_position: _Tool2Position | None = None
    for index, operation_mode in enumerate(operation_modes):
        has_next_same_pass_operation = index + 1 < len(operation_modes)
        last_position = _run_left_side(
            cps,
            config,
            force_by_mode[operation_mode],
            x_total,
            y_total,
            operation_mode,
            lift_z=lift_z,
            start_from_top=start_from_top,
            enter_from_contact=index > 0,
            lift_after=not has_next_same_pass_operation,
            backoff_before_entry=index > 0 and _is_edge_operation(operation_mode),
            reverse_path=index > 0 and _is_edge_operation(operation_mode),
        )
    if last_position is None:
        raise TableBDxfRobotExecutionError("Tool 2 left side has no selected operations.")
    return last_position


def _split_tool2_bottom_segments_for_traversal(x_total_mm: float) -> list[_Tool2HorizontalSegment]:
    return [
        _Tool2HorizontalSegment("bottom", axis7, 0.0, local_x_end, -15.0)
        for axis7, local_x_end in split_tool2_bottom_segments(x_total_mm)
    ]


def _split_tool2_top_segments_increasing(x_total_mm: float, y_total_mm: float) -> list[_Tool2HorizontalSegment]:
    x_total = max(0.0, float(x_total_mm))
    y = float(y_total_mm) + 15.0
    span = tool2_top_local_x_span_at_y(float(y_total_mm))
    if x_total <= 0.0 or span is None:
        return []

    local_min, local_max = span
    if local_max <= local_min:
        return []

    segments: list[_Tool2HorizontalSegment] = []
    start = 0.0
    first_axis7 = 0.0
    if local_min <= 0.0 <= local_max:
        first_end = min(x_total, first_axis7 + local_max)
        if first_end > start + 1e-6:
            segments.append(_Tool2HorizontalSegment("top", first_axis7, start - first_axis7, first_end - first_axis7, y))
            start = first_end

    while start < x_total - 1e-6:
        axis7 = start - local_min
        end = min(x_total, axis7 + local_max)
        if end <= start + 1e-6:
            break
        segments.append(_Tool2HorizontalSegment("top", axis7, start - axis7, end - axis7, y))
        start = end
    return segments


def _segments_by_station(segments: list[_Tool2HorizontalSegment], tolerance_mm: float = 1.0) -> list[tuple[float, list[_Tool2HorizontalSegment]]]:
    ordered = sorted(segments, key=lambda segment: (segment.axis7, 0 if segment.side == "bottom" else 1))
    stations: list[tuple[float, list[_Tool2HorizontalSegment]]] = []
    for segment in ordered:
        if stations and abs(stations[-1][0] - segment.axis7) <= tolerance_mm:
            stations[-1][1].append(segment)
        else:
            stations.append((segment.axis7, [segment]))
    return stations


def _reverse_horizontal_segment(segment: _Tool2HorizontalSegment) -> _Tool2HorizontalSegment:
    return _Tool2HorizontalSegment(
        segment.side,
        segment.axis7,
        segment.local_end_x,
        segment.local_start_x,
        segment.y,
    )


def _ordered_station_segments(
    station_segments: list[_Tool2HorizontalSegment],
    last_position: _Tool2Position | None,
) -> list[_Tool2HorizontalSegment]:
    if len(station_segments) <= 1:
        return station_segments

    by_side = {segment.side: segment for segment in station_segments}
    if last_position and last_position.side == "bottom":
        order = ["bottom", "top"]
    elif last_position and last_position.side == "top":
        order = ["top", "bottom"]
    else:
        order = ["top", "bottom"]
    return [by_side[side] for side in order if side in by_side]


def _next_station_first_side(
    stations: list[tuple[float, list[_Tool2HorizontalSegment]]],
    next_station_index: int,
    last_position: _Tool2Position | None,
) -> str | None:
    if next_station_index >= len(stations):
        return None
    ordered = _ordered_station_segments(stations[next_station_index][1], last_position)
    if not ordered:
        return None
    return ordered[0].side


def _next_side_in_sequence(
    sequence: list[_Tool2HorizontalSegment],
    index: int,
    stations: list[tuple[float, list[_Tool2HorizontalSegment]]],
    next_station_index: int,
    current_side: str,
) -> str | None:
    if index + 1 < len(sequence):
        return sequence[index + 1].side
    assumed_position = _Tool2Position(current_side, 0.0, 0.0, TOOL2_SIDE_RZ_DEG[current_side])
    return _next_station_first_side(stations, next_station_index, assumed_position)


def _run_tool2_station_traversal(
    cps: Any,
    config: dict[str, Any],
    force_by_mode: dict[str, float],
    x_total: float,
    y_total: float,
    operation_modes: list[str],
) -> None:
    bottom_segments = _split_tool2_bottom_segments_for_traversal(x_total)
    top_segments = _split_tool2_top_segments_increasing(x_total, y_total)
    if not bottom_segments and not top_segments:
        raise TableBDxfRobotExecutionError("Tool 2 has no reachable top/bottom side segments.")

    _log(
        config,
        "[TableB DXF Tool2] operations=%s station traversal bottom_segments=%s top_segments=%s",
        "+".join(operation_modes),
        len(bottom_segments),
        len(top_segments),
    )

    stations = _segments_by_station([*bottom_segments, *top_segments])
    # Start from the actual homing pose. Do not inject a fake previous side,
    # otherwise startup performs an extra side-transition before the first pass.
    last_position: _Tool2Position | None = None
    right_done = False
    left_done = False

    for station_index, (axis7, station_segments) in enumerate(stations):
        _raise_if_stop_requested(cps, config, f"before Tool 2 {'+'.join(operation_modes)} station {station_index + 1}")
        ordered_segments = _ordered_station_segments(station_segments, last_position)

        if not right_done and abs(axis7) <= 1.0:
            # At J7=0, run the reachable top split back toward X=0 so the tool
            # ends near the right-side transition before sanding top->bottom.
            top_sequence = [s for s in ordered_segments if s.side == "top"]
            for index, segment in enumerate(top_sequence):
                last_position = _run_horizontal_segment_operations(
                    cps,
                    config,
                    force_by_mode,
                    operation_modes,
                    _reverse_horizontal_segment(segment),
                    last_position,
                    next_side=_next_side_in_sequence(top_sequence, index, stations, station_index + 1, "top"),
                    y_total=y_total,
                )
            last_position = _run_right_side_operations(cps, config, force_by_mode, operation_modes, y_total, last_position)
            right_done = True
            bottom_sequence = [s for s in ordered_segments if s.side == "bottom"]
            for index, segment in enumerate(bottom_sequence):
                last_position = _run_horizontal_segment_operations(
                    cps,
                    config,
                    force_by_mode,
                    operation_modes,
                    segment,
                    last_position,
                    next_side=_next_side_in_sequence(bottom_sequence, index, stations, station_index + 1, "bottom"),
                    y_total=y_total,
                )
            continue

        is_final_station = station_index == len(stations) - 1
        if is_final_station and not left_done:
            # Finish near the left side with the closest horizontal segment first, then left side, then the remaining segment.
            bottom_first = last_position is None or last_position.side == "bottom"
            first_side = "bottom" if bottom_first else "top"
            second_side = "top" if bottom_first else "bottom"
            first_sequence = [s for s in ordered_segments if s.side == first_side]
            for index, segment in enumerate(first_sequence):
                last_position = _run_horizontal_segment_operations(
                    cps,
                    config,
                    force_by_mode,
                    operation_modes,
                    segment,
                    last_position,
                    next_side=_next_side_in_sequence(first_sequence, index, stations, station_index + 1, first_side),
                    y_total=y_total,
                )
            last_position = _run_left_side_operations(cps, config, force_by_mode, operation_modes, x_total, y_total, last_position)
            left_done = True
            second_sequence = [s for s in ordered_segments if s.side == second_side]
            for index, segment in enumerate(second_sequence):
                last_position = _run_horizontal_segment_operations(
                    cps,
                    config,
                    force_by_mode,
                    operation_modes,
                    segment,
                    last_position,
                    next_side=_next_side_in_sequence(second_sequence, index, stations, station_index + 1, second_side),
                    y_total=y_total,
                )
            continue

        for index, segment in enumerate(ordered_segments):
            last_position = _run_horizontal_segment_operations(
                cps,
                config,
                force_by_mode,
                operation_modes,
                segment,
                last_position,
                next_side=_next_side_in_sequence(ordered_segments, index, stations, station_index + 1, segment.side),
                y_total=y_total,
            )

    if not right_done:
        _run_right_side_operations(cps, config, force_by_mode, operation_modes, y_total, last_position)
    if not left_done:
        _run_left_side_operations(cps, config, force_by_mode, operation_modes, x_total, y_total, last_position)


def _force_for_steps(steps: list[dict[str, Any]]) -> float:
    for step in steps:
        try:
            force = float((step.get("recipe") or {}).get("force") or 0.0)
        except (TypeError, ValueError):
            force = 0.0
        if force > 0:
            return force
    raise TableBDxfRobotExecutionError("Tool 2 batch has no valid force value.")


def _operation_mode_for_step(step: dict[str, Any]) -> str:
    tool = str(step.get("tool") or "")
    if tool == "tool_2_edgeOutside":
        return TOOL2_OPERATION_EDGE
    return TOOL2_OPERATION_SIDE


def _operation_groups(steps: list[dict[str, Any]]) -> list[tuple[str, list[dict[str, Any]]]]:
    groups: dict[str, list[dict[str, Any]]] = {
        TOOL2_OPERATION_SIDE: [],
        TOOL2_OPERATION_EDGE: [],
    }
    for step in steps:
        groups[_operation_mode_for_step(step)].append(step)
    # Physical order for combined Tool 2 work: side first, then edge on the same pass.
    return [(mode, groups[mode]) for mode in (TOOL2_OPERATION_SIDE, TOOL2_OPERATION_EDGE) if groups[mode]]


def _run_tool2_sequence(cps: Any, config: dict[str, Any], steps: list[dict[str, Any]], operation_mode: str) -> int:
    force = _force_for_steps(steps)
    x_total, y_total = _bounds_from_steps(steps)
    _log(
        config,
        "[TableB DXF Tool2] %s start x_total=%.3f y_total=%.3f force=%.1f outward=%.1f edge_ry=%.1f",
        operation_mode,
        x_total,
        y_total,
        force,
        TOOL2_APPROACH_OUTWARD_MM,
        TOOL2_EDGE_RY_DEG if _is_edge_operation(operation_mode) else 0.0,
    )

    _raise_if_stop_requested(cps, config, f"before Tool 2 {operation_mode} batch")
    _run_tool2_station_traversal(cps, config, {operation_mode: force}, x_total, y_total, [operation_mode])
    return len(steps)


def run_tool2_side_batch(cps: Any, config: dict[str, Any], steps: list[dict[str, Any]]) -> int:
    """Run selected Tool 2 side/thickness operations.

    `tool_2_side` keeps RY=0 throughout sanding. `tool_2_edgeOutside` travels
    with RY=0, bends to RY=-22 only after the contact XY/Z is reached, applies
    force/vibration, sands the pass with RY=-22, then resets RY=0 before moving.
    """
    if not steps:
        return 0

    executed = 0
    try:
        groups = _operation_groups(steps)
        if len(groups) > 1:
            operation_modes = [mode for mode, _grouped_steps in groups]
            force_by_mode = {mode: _force_for_steps(grouped_steps) for mode, grouped_steps in groups}
            x_total, y_total = _bounds_from_steps(steps)
            _log(
                config,
                "[TableB DXF Tool2] combined operations=%s x_total=%.3f y_total=%.3f forces=%s",
                "+".join(operation_modes),
                x_total,
                y_total,
                force_by_mode,
            )
            _raise_if_stop_requested(cps, config, "before combined Tool 2 batch")
            _run_tool2_station_traversal(cps, config, force_by_mode, x_total, y_total, operation_modes)
            executed = len(steps)
        else:
            for operation_mode, grouped_steps in groups:
                executed += _run_tool2_sequence(cps, config, grouped_steps, operation_mode)
        clear_tool2_recovery_state()
    except TableBDxfRobotStopRequested:
        raise
    except Exception:
        _stop_active_motion(cps, config, "Tool 2 side/edge batch error")
        raise

    return executed


def run_tool2_side_path(cps: Any, config: dict[str, Any], step: dict[str, Any]) -> None:
    """Compatibility wrapper for callers that still pass one Tool 2 step."""
    run_tool2_side_batch(cps, config, [step])
