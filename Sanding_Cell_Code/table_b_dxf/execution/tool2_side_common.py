"""Shared Tool 2 (Table B) primitives: constants, pose helpers, and the move wrapper.

Split out of tool2_side_runner.py. Contains no side-transition or pass logic --
only the values and helpers that every Tool 2 module builds on.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, NamedTuple

from Server_Better_V2 import (
    communicate,
    putForceXminus,
    putForceXplus,
    putForceYminus1,
    putForceYplus1,
)

from .common import TableBDxfRobotExecutionError
from .motion_config import robot_speed
from .tool2_recovery import record_tool2_recovery_state
from .tool2_side_geometry import (
    TOOL2_APPROACH_OUTWARD_MM,
    TOOL2_CONTACT_Z_MM,
    TOOL2_LEFT_PREFORCE_LOCAL_X_MM,
    Tool2HorizontalSegment as _Tool2HorizontalSegment,
    tool2_lift_z_for_side,
    tool2_left_axis7_position,
)


TOOL2_OPERATION_SIDE = "side"
TOOL2_OPERATION_EDGE = "edgeOutside"
TOOL2_EDGE_RY_DEG = -22.0
TOOL2_EDGE_APPROACH_OUTWARD_MM = 5.0
# Bottom travel line: the tool rides on the EDGE prepoint (Y=-5, RY=-22) between
# bottom passes and holds it while the 7th axis moves.
TOOL2_BOTTOM_TRAVEL_OUTWARD_MM = 5.0
# Right side end used to pick the right -> bottom entry route.
TOOL2_RIGHT_BOTTOM_DETOUR_Y_MM = 200.0
# Top -> right handoff, x=0 case. Both values are taken verbatim from the
# operator's hand-calculated point list: the corner is approached at X=-15 and
# the right edge is entered at X=+15.
TOOL2_TOP_RIGHT_CORNER_X_MM = -15.0
# Right side is at local X=-15; the tool must not cross the edge to +15.
TOOL2_TOP_RIGHT_ENTRY_X_MM = -15.0
# Bottom prepoint Y used by the right -> bottom handoff in that same list.
TOOL2_BOTTOM_PREPOINT_Y_MM = -5.0
# Mid-Y waypoint used when crossing between bottom and top. The arm reaches this
# point FIRST and only then turns the wrist, so the tool is never rotating while
# it translates across the door. Slightly deeper than the normal -68 travel
# height for extra clearance during the 180 turn.
TOOL2_MID_TRANSITION_Z_MM = -75.0
TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM = 100.0

TOOL2_SIDE_RZ_DEG: dict[str, float] = {
    "right": 0.0,
    "bottom": 90.0,
    "left": 180.0,
    "top": -90.0,
}

# Coordinate RZ is the source of truth for Tool 2 side orientation.
# Relative J6 is used only to force the wrist to rotate through the safe direction.
TOOL2_SIDE_TRANSITION_J6_DEG: dict[tuple[str, str], float] = {
    ("bottom", "top"): 180.0,
    ("top", "bottom"): -180.0,
    ("top", "right"): 90.0,
    ("right", "top"): -90.0,
    ("bottom", "right"): -90.0,
    ("right", "bottom"): 90.0,
    ("bottom", "left"): 90.0,
    ("left", "bottom"): -90.0,
    ("left", "top"): 90.0,
    ("top", "left"): -90.0,
}

def _assert_j6_map_is_safe() -> None:
    """Fail at import if the wrist map could walk J6 toward its limits.

    J6 is bounded [-360, 360] and every side change is commanded as a RELATIVE
    move, so an inconsistent map would accumulate instead of oscillating. Three
    properties keep the wrist near zero:

      1. every delta is +-90 or +-180 -- never the long way round (270),
      2. a pair and its reverse cancel exactly, so out-and-back returns to the
         same joint value rather than drifting one revolution,
      3. the delta matches the coordinate RZ change (mod 360), so the joint move
         and the pose command agree and the wrist is not turned twice.

    Violating (3) is what produced the earlier top -> right double rotation.
    """
    for (source, target), delta in TOOL2_SIDE_TRANSITION_J6_DEG.items():
        if abs(delta) not in (90.0, 180.0):
            raise TableBDxfRobotExecutionError(
                f"Tool 2 J6 map {source}->{target}={delta:+.0f} is not a +-90/+-180 turn."
            )
        reverse = TOOL2_SIDE_TRANSITION_J6_DEG.get((target, source))
        if reverse is not None and abs(delta + reverse) > 1e-6:
            raise TableBDxfRobotExecutionError(
                f"Tool 2 J6 map {source}->{target}={delta:+.0f} and reverse {reverse:+.0f} do not cancel; "
                "repeated crossings would walk J6 toward its limit."
            )
        rz_delta = TOOL2_SIDE_RZ_DEG[target] - TOOL2_SIDE_RZ_DEG[source]
        if abs(((delta - rz_delta) + 180.0) % 360.0 - 180.0) > 1e-6:
            raise TableBDxfRobotExecutionError(
                f"Tool 2 J6 map {source}->{target}={delta:+.0f} disagrees with RZ change {rz_delta:+.0f}; "
                "the wrist would rotate twice."
            )


_assert_j6_map_is_safe()


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


class _Tool2Position(NamedTuple):
    side: str | None
    x: float
    y: float
    z: float
    rz: float
    axis7: float | None = None




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
    """Safe travel height for a side transition.

    All sides share one safe height, so the destination side's value is always
    correct. Bottom -> left used to force bottom's (then shallower) height, which
    left the tool travelling the long 7th-axis move closer to the door than every
    other transition.
    """
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


def _bottom_travel_y() -> float:
    return -TOOL2_BOTTOM_TRAVEL_OUTWARD_MM


def _bottom_travel_pose(x: float, z: float, rz: float) -> list[float]:
    return _pose(x, _bottom_travel_y(), z, rz, ry=TOOL2_EDGE_RY_DEG)


def _vertical_x_for_operation(side: str, x: float, operation_mode: str) -> float:
    offset_delta = TOOL2_APPROACH_OUTWARD_MM - _tool2_outward_offset_for_operation(operation_mode)
    if side == "right":
        return -_tool2_outward_offset_for_operation(operation_mode)
    if side == "left":
        return x - offset_delta
    return x


# Door height for the active cycle, so stop recovery can place the top side.
_ACTIVE_Y_TOTAL_MM: float | None = None


def _set_active_y_total(y_total: float | None) -> None:
    """Remember door height for the cycle so stop recovery can place the top side."""
    global _ACTIVE_Y_TOTAL_MM
    _ACTIVE_Y_TOTAL_MM = float(y_total) if y_total is not None else None


def _active_y_total() -> float | None:
    """Read the cycle's door height. Must be a call, not an import: importing the
    module-level value would snapshot None before the cycle sets it."""
    return _ACTIVE_Y_TOTAL_MM


_ACTIVE_FRAME: tuple[str | None, str | None] = (None, None)


def _set_active_frame(tcp: str | None, ucs: str | None) -> None:
    """Remember the TCP/UCS in force so recovery state records the right frame."""
    global _ACTIVE_FRAME
    _ACTIVE_FRAME = (tcp, ucs)


def _mark_tool2_recovery(side: str, pose: list[float]) -> None:
    tcp, ucs = _ACTIVE_FRAME
    record_tool2_recovery_state(
        side, pose, active=True, y_total=_ACTIVE_Y_TOTAL_MM, tcp=tcp, ucs=ucs
    )


def _left_side_bottom_lift_y(side: str, end_y: float) -> float | None:
    if side == "left" and abs(float(end_y)) <= 1e-6:
        return TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM
    return None


def _same_pose(a: list[float] | None, b: list[float] | None, tol: float = 1e-6) -> bool:
    """True when two 6-DOF poses are identical within tolerance."""
    if a is None or b is None or len(a) < 6 or len(b) < 6:
        return False
    return all(abs(float(a[i]) - float(b[i])) <= tol for i in range(6))


def _move_unless_already_there(
    cps: Any,
    config: dict[str, Any],
    point: list[float],
    current: list[float] | None,
    **kwargs: Any,
) -> None:
    """Issue a repositioning move only when it would actually change the pose.

    Exit paths re-command the pose the sanding pass just finished at, which the
    controller executes as a zero-length move. Skipping those removes the visible
    stutter without touching any move that genuinely repositions the tool.
    """
    if _same_pose(point, current):
        return
    _move(cps, config, point, **kwargs)


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
    tcp = _tool2_tcp(config)
    ucs = _tool2_ucs(config)
    _set_active_frame(tcp, ucs)
    result = communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=tcp,
        ucs=ucs,
        seventh=seventh,
        speed=robot_speed(config) if speed is None else speed,
        velocity_profile=velocity_profile,
        wait=wait,
        speed_mode=speed_mode,
        require_seventh_ok=require_seventh_ok,
    )
    if result is None:
        raise TableBDxfRobotExecutionError(f"Tool 2 communicate failed for point={point} seventh={seventh}.")

