"""Tool 2 (Table B) side-to-side transition geometry.

Every move the tool makes BETWEEN sanding passes lives here: how it leaves one
side, how the wrist rotates, and how it arrives at the next. Split out of
tool2_side_runner.py because this is the layer that governs travel cost and is
where the door-strike bugs occur.

Each side pair has its own exit helper so a change to one crossing cannot
silently alter another:
    bottom -> top/left   _prepare_bottom_side_exit_before_j6
    top    -> right      _prepare_top_side_exit_before_right
    right  -> bottom     _prepare_right_side_exit_before_bottom
The J6/RZ sweep itself, plus J7 overlap, is _apply_side_transition_j6.
"""

from __future__ import annotations

from typing import Any

from .common import TableBDxfRobotExecutionError
from .joint_safety import guarded_move_only_j6r
from .tool2_recovery import record_tool2_recovery_state
from .tool2_side_geometry import (
    TOOL2_TOP_RECT_LOCAL_X,
    tool2_straight_leg_is_reachable,
)
from .tool2_side_common import (
    TOOL2_APPROACH_OUTWARD_MM,
    TOOL2_CONTACT_Z_MM,
    TOOL2_EDGE_RY_DEG,
    TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM,
    TOOL2_BOTTOM_PREPOINT_Y_MM,
    TOOL2_MID_TRANSITION_Z_MM,
    TOOL2_RIGHT_BOTTOM_DETOUR_Y_MM,
    TOOL2_TOP_RIGHT_CORNER_X_MM,
    TOOL2_TOP_RIGHT_ENTRY_X_MM,
    TOOL2_SIDE_RZ_DEG,
    TOOL2_SIDE_TRANSITION_J6_DEG,
    _Tool2Position,
    _active_y_total,
    _bottom_travel_pose,
    _bottom_travel_y,
    _move,
    _move_unless_already_there,
    _pose,
    tool2_lift_z_for_side,
)
from .common import log as _log


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


def _move_axis7_and_tool2_guarded_prepoint(
    cps: Any,
    config: dict[str, Any],
    *,
    axis7: float,
    prepoint: list[float],
    previous_position: _Tool2Position | None,
    next_side: str,
    y_total: float | None,
    axis7_already_started: bool = False,
) -> None:
    """Move J7 and Tool 2 arm to a prepoint without cutting through the reach notch.

    Tool 2 has the same practical problem as the XY-plane tools: a direct MoveL
    between a low-Y bottom pose and a high-Y negative-X top pose can briefly enter
    the low-Y/negative-X area and fold the arm. Route those two crossings through
    local X=0 at a safe Y/Z, then continue to the real prepoint.
    """
    if previous_position is None or previous_position.side is None or len(prepoint) < 6:
        if axis7_already_started:
            _move(cps, config, prepoint, velocity_profile="robotspeed", wait=True, require_seventh_ok=True)
        else:
            _move_axis7_and_lifted_prepoint(cps, config, axis7=axis7, prepoint=prepoint)
        return

    target_x = float(prepoint[0])
    target_y = float(prepoint[1])
    target_z = float(prepoint[2])
    target_rz = float(prepoint[5])
    safe_y = _transition_mid_y(y_total)
    if safe_y is None:
        safe_y = max(TOOL2_LEFT_BOTTOM_LIFT_CLEARANCE_Y_MM, previous_position.y, target_y)
    safe_z = min(float(previous_position.z), target_z, tool2_lift_z_for_side("top"))

    # bottom -> top reaches the mid-Y waypoint before the wrist turns, but that
    # waypoint sits at X>=0 while a top prepoint can be deeply negative. Going
    # straight there would be one long diagonal across the reach notch, so the
    # crossing is still split at local X=0 at the safe height.
    bottom_to_top_negative_x = (
        previous_position.side == "bottom"
        and next_side == "top"
        and target_x < 0.0
    )
    if not bottom_to_top_negative_x:
        if axis7_already_started:
            _move(cps, config, prepoint, velocity_profile="robotspeed", wait=True, require_seventh_ok=True)
        else:
            _move_axis7_and_lifted_prepoint(cps, config, axis7=axis7, prepoint=prepoint)
        return

    # Climb inside the low rectangle at an X that is legal there, THEN move in X
    # once above the step. Going straight to the target X would cut the step
    # corner and leave the envelope even though both ends are inside it.
    climb_x = min(max(0.0, float(previous_position.x)), TOOL2_TOP_RECT_LOCAL_X[1])
    if not tool2_straight_leg_is_reachable((climb_x, float(previous_position.y)), (climb_x, safe_y)):
        climb_x = 0.0
    intermediate = _pose(climb_x, safe_y, safe_z, target_rz)
    if not tool2_straight_leg_is_reachable((climb_x, safe_y), (target_x, target_y)):
        # Still cutting a corner: step X at the safe Y before continuing.
        _log(
            config,
            "[TableB DXF Tool2] reach: %s -> %s leg would leave the envelope; stepping X at Y=%.1f first",
            previous_position.side,
            next_side,
            safe_y,
        )
    reason = "bottom -> top negative-X"

    _log(
        config,
        "[TableB DXF Tool2] guarded reach transition %s: j7=%.3f via %s before prepoint %s",
        reason,
        axis7,
        intermediate,
        prepoint,
    )
    if not axis7_already_started:
        _move(
            cps,
            config,
            None,
            seventh=axis7,
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True,
        )
    # Do not require J7 idle yet: this waypoint exists specifically so the arm
    # can travel safely while the rail is moving.
    _move(
        cps,
        config,
        intermediate,
        velocity_profile="robotspeed",
        wait=True,
        require_seventh_ok=False,
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
    lift_z = tool2_lift_z_for_side("bottom")
    _log(
        config,
        "[TableB DXF Tool2] bottom exit before %s: hold Y=%.1f RY=%.1f, contact Z=0 -> lift Z=%.1f at x=%.3f",
        next_side,
        _bottom_travel_y(),
        TOOL2_EDGE_RY_DEG,
        lift_z,
        x,
    )
    # Bottom is the special side: it retracts off Y=0 and keeps RY=-22 all the way
    # through the transition, unlike top/left/right which back off 15 mm at RY=0.
    # The wrist rotation safety guard reads live joints before moving J6, so the
    # final lifted pose must be reached (wait=True) first.
    if previous_position.z > -5.0:
        _move(cps, config, _bottom_travel_pose(x, TOOL2_CONTACT_Z_MM, rz), velocity_profile="robotspeed", wait=True)
    if next_side == "top":
        # Going to top means a 180 wrist turn. At the normal bottom lift height the
        # tilted tool is still close enough to strike the door as it swings, so
        # climb to a deeper clearance at Y=0 (still RY=-22) and turn from there.
        _move(
            cps,
            config,
            # Stay on the bottom travel line in Y (clear of the door); only Z
            # goes deeper. Y=0 would put the tool back at the door edge, which is
            # exactly where the 180 turn then strikes it.
            _bottom_travel_pose(x, tool2_lift_z_for_side("top"), rz),
            velocity_profile="robotspeed",
            wait=True,
        )
        return
    _move(cps, config, _bottom_travel_pose(x, lift_z, rz), velocity_profile="robotspeed", wait=True)


def _prepare_top_side_exit_before_right(
    cps: Any,
    config: dict[str, Any],
    previous_position: _Tool2Position,
    y_total: float | None,
) -> None:
    """Top -> right handoff, per the operator's point list.

    Two cases, by where the top pass ended:

    ended at x = X (away from origin), lift and cross at height:
        1) [x, y_total+15,   0, 0, 0, -90]   push off the door
        2) [x, y_total+15, -68, 0, 0, -90]
        3) [-15, y_total,  -68, 0, 0,   0]
        4) [-15, y_total,    0, 0, 0,   0]

    ended at x = 0, stay at contact height the whole way -- deliberately NOT
    going to Z=-68 for this crossing:
        1) [-15, y_total+15, 0, 0, 0, -90]   push off the door
        2) [-15, y_total+15, 0, 0, 0,   0]
        3) [ 15, y_total,    0, 0, 0,   0]

    The RZ change is commanded as a coordinate value here; the caller's J6 sweep
    is suppressed for this pair so the wrist is not turned twice.
    """
    if y_total is None:
        return
    top_rz = TOOL2_SIDE_RZ_DEG["top"]
    right_rz = TOOL2_SIDE_RZ_DEG["right"]
    top_y = float(y_total) + TOOL2_APPROACH_OUTWARD_MM
    lift_z = tool2_lift_z_for_side("top")
    x = float(previous_position.x)
    ends_at_origin = abs(x) <= 1e-6

    _log(
        config,
        "[TableB DXF Tool2] top -> right exit from x=%.3f (%s)",
        x,
        "x=0: flat crossing at contact height" if ends_at_origin else "x=X: lift and cross at Z=%.0f" % lift_z,
    )

    if ends_at_origin:
        # 1) push off the door, still at top orientation
        _move(cps, config, _pose(TOOL2_TOP_RIGHT_CORNER_X_MM, top_y, TOOL2_CONTACT_Z_MM, top_rz),
              velocity_profile="robotspeed", wait=True)
        # 2) rotate to the right orientation on the same point
        _move(cps, config, _pose(TOOL2_TOP_RIGHT_CORNER_X_MM, top_y, TOOL2_CONTACT_Z_MM, right_rz),
              velocity_profile="robotspeed", wait=True)
        # 3) slide onto the right standoff at contact height -- no Z excursion.
        #    Force control closes the remaining gap; do NOT command the tool into
        #    the door here.
        _move(cps, config, _pose(TOOL2_TOP_RIGHT_ENTRY_X_MM, float(y_total), TOOL2_CONTACT_Z_MM, right_rz),
              velocity_profile="robotspeed", wait=True)
        return

    # 1) push off the door at contact height
    _move(cps, config, _pose(x, top_y, TOOL2_CONTACT_Z_MM, top_rz),
          velocity_profile="robotspeed", wait=True)
    # 2) lift
    _move(cps, config, _pose(x, top_y, lift_z, top_rz),
          velocity_profile="robotspeed", wait=True)
    # 3) cross to the right edge at height, already at right RZ
    _move(cps, config, _pose(-TOOL2_APPROACH_OUTWARD_MM, float(y_total), lift_z, right_rz),
          velocity_profile="robotspeed", wait=True)
    # 4) descend to contact
    _move(cps, config, _pose(-TOOL2_APPROACH_OUTWARD_MM, float(y_total), TOOL2_CONTACT_Z_MM, right_rz),
          velocity_profile="robotspeed", wait=True)


def _right_finished_at_top(previous_position: _Tool2Position, y_total: float | None) -> bool:
    """True when the right pass ended at the top-right corner rather than right-bottom."""
    if y_total is None:
        return False
    return abs(float(previous_position.y) - float(y_total)) < abs(float(previous_position.y))


def _prepare_right_side_exit_before_bottom(
    cps: Any,
    config: dict[str, Any],
    previous_position: _Tool2Position,
    y_total: float | None,
) -> None:
    """Right -> bottom handoff, per the operator's point list.

    Two cases, by which end of the right edge the pass finished on. Both rotate
    the wrist IN PLACE at a clear standoff (RZ first, then RY) before travelling
    to the bottom line, rather than moving first and rotating at the destination.

    ended at [0, y_total] (top-right):
        1) [-15, y_total,   0, 0,   0,  0]   push off the door
        2) [-15, y_total, -68, 0,   0,  0]
        3) [-15, y_total, -68, 0,   0, 90]   RZ turn, in place
        4) [  0,      -5, -68, 0, -22, 90]
        5) [  0,      -5,   0, 0, -22, 90]   edge starts force here

    ended at [0, 0] (right-bottom):
        1) [-15,   0,   0, 0,   0,  0]       push off the door
        2) [-15, 200, -68, 0,   0,  0]
        3) [-15, 200, -68, 0,   0, 90]       RZ turn, in place
        4) [-15, 200, -68, 0, -22, 90]       RY tilt, in place
        5) [  0,  -5,   0, 0, -22, 90]       edge starts force here

    Side sanding continues from there to [0, -15, 0, 0, 0, 90]; that final step
    belongs to the pass entry, not this helper. The caller's J6 sweep is
    suppressed for this pair because RZ is commanded here.
    """
    right_rz = TOOL2_SIDE_RZ_DEG["right"]
    bottom_rz = TOOL2_SIDE_RZ_DEG["bottom"]
    lift_z = tool2_lift_z_for_side("right")
    x = -TOOL2_APPROACH_OUTWARD_MM
    finished_at_top = _right_finished_at_top(previous_position, y_total)

    _log(
        config,
        "[TableB DXF Tool2] right -> bottom exit from %s corner (y=%.3f)",
        "top-right" if finished_at_top else "right-bottom",
        previous_position.y,
    )

    if finished_at_top and y_total is not None:
        corner_y = float(y_total)
        # 1) push off the door at contact height
        _move(cps, config, _pose(x, corner_y, TOOL2_CONTACT_Z_MM, right_rz),
              velocity_profile="robotspeed", wait=True)
        # 2) lift
        _move(cps, config, _pose(x, corner_y, lift_z, right_rz),
              velocity_profile="robotspeed", wait=True)
        # 3) RZ turn in place
        _move(cps, config, _pose(x, corner_y, lift_z, bottom_rz),
              velocity_profile="robotspeed", wait=True)
        # 4) travel to the bottom prepoint, tilted
        _move(cps, config, _pose(0.0, TOOL2_BOTTOM_PREPOINT_Y_MM, lift_z, bottom_rz, ry=TOOL2_EDGE_RY_DEG),
              velocity_profile="robotspeed", wait=True)
        # 5) descend to contact
        _move(cps, config, _pose(0.0, TOOL2_BOTTOM_PREPOINT_Y_MM, TOOL2_CONTACT_Z_MM, bottom_rz, ry=TOOL2_EDGE_RY_DEG),
              velocity_profile="robotspeed", wait=True)
        return

    # right-bottom corner
    # 1) push off the door at contact height
    _move(cps, config, _pose(x, 0.0, TOOL2_CONTACT_Z_MM, right_rz),
          velocity_profile="robotspeed", wait=True)
    # 2) climb to the raised standoff
    _move(cps, config, _pose(x, TOOL2_RIGHT_BOTTOM_DETOUR_Y_MM, lift_z, right_rz),
          velocity_profile="robotspeed", wait=True)
    # 3) RZ turn in place
    _move(cps, config, _pose(x, TOOL2_RIGHT_BOTTOM_DETOUR_Y_MM, lift_z, bottom_rz),
          velocity_profile="robotspeed", wait=True)
    # 4) RY tilt in place, then travel to the bottom prepoint AT HEIGHT. Going
    #    straight from [x, 200, -68] to [0, -5, 0] descends 68 mm while crossing
    #    205 mm in Y, so the tool reaches contact height while still over the
    #    door and strikes the surface. Arrive above the prepoint first.
    _move(cps, config, _pose(x, TOOL2_RIGHT_BOTTOM_DETOUR_Y_MM, lift_z, bottom_rz, ry=TOOL2_EDGE_RY_DEG),
          velocity_profile="robotspeed", wait=True)
    _move(cps, config, _pose(0.0, TOOL2_BOTTOM_PREPOINT_Y_MM, lift_z, bottom_rz, ry=TOOL2_EDGE_RY_DEG),
          velocity_profile="robotspeed", wait=True)
    # 5) straight down onto the bottom prepoint
    _move(cps, config, _pose(0.0, TOOL2_BOTTOM_PREPOINT_Y_MM, TOOL2_CONTACT_Z_MM, bottom_rz, ry=TOOL2_EDGE_RY_DEG),
          velocity_profile="robotspeed", wait=True)


def _apply_bottom_tilt_after_rotation(
    cps: Any,
    config: dict[str, Any],
    x: float,
) -> None:
    """Tilt to RY=-22 once RZ is already 90, on the bottom travel line."""
    _move(
        cps,
        config,
        _pose(x, _bottom_travel_y(), tool2_lift_z_for_side("bottom"), TOOL2_SIDE_RZ_DEG["bottom"], ry=TOOL2_EDGE_RY_DEG),
        velocity_profile="robotspeed",
        wait=True,
    )


def _safe_to_start_j7_during_tool2_side_transition(previous_position: _Tool2Position | None) -> bool:
    if previous_position is None or previous_position.side is None:
        return False
    # Table B Tool 2 uses negative Z as lifted/safe. Do not overlap J7 with any
    # wrist rotation unless the tool is already clear of the door surface.
    safe_z = tool2_lift_z_for_side(previous_position.side)
    return float(previous_position.z) <= float(safe_z) + 0.5


def _apply_side_transition_j6(
    cps: Any,
    config: dict[str, Any],
    previous_position: _Tool2Position | None,
    next_side: str,
    y_total: float | None = None,
    next_axis7: float | None = None,
) -> bool:
    if previous_position is None or previous_position.side is None or previous_position.side == next_side:
        return False
    j6_delta = TOOL2_SIDE_TRANSITION_J6_DEG.get((previous_position.side, next_side))
    if j6_delta is None or abs(j6_delta) <= 1e-6:
        return False
    started_axis7 = False
    rz_already_commanded = False
    if previous_position.side == "top" and next_side == "right":
        _prepare_top_side_exit_before_right(cps, config, previous_position, y_total)
        # RZ was commanded as a coordinate above; a J6 sweep here would turn the
        # wrist a second time.
        rz_already_commanded = True
    if previous_position.side == "right" and next_side == "bottom":
        _prepare_right_side_exit_before_bottom(cps, config, previous_position, y_total)
        rz_already_commanded = True
    if previous_position.side == "top" and next_side == "bottom":
        mid_y = _transition_mid_y(y_total)
        if mid_y is not None:
            lift_z = tool2_lift_z_for_side("top")
            _log(
                config,
                "[TableB DXF Tool2] top exit before bottom: normalize to X=0 then travel to mid Y=%.3f at Z=%.3f before J6",
                mid_y,
                lift_z,
            )
            if previous_position.x < 0.0:
                # Blocking: if this blends into the following mid-Y move the two
                # become one diagonal from deep negative X down across the step,
                # which leaves the reach envelope.
                _move(
                    cps,
                    config,
                    _pose(0.0, previous_position.y, lift_z, TOOL2_SIDE_RZ_DEG["top"]),
                    velocity_profile="robotspeed",
                    wait=True,
                )
            _move(
                cps,
                config,
                _pose(0.0, mid_y, TOOL2_MID_TRANSITION_Z_MM, TOOL2_SIDE_RZ_DEG["top"]),
                velocity_profile="robotspeed",
                wait=True,
            )
    bottom_exit_prepared = previous_position.side == "bottom" and next_side in {"top", "left"}
    _prepare_bottom_side_exit_before_j6(cps, config, previous_position, next_side, y_total)
    if previous_position.side == "bottom" and next_side == "top":
        mid_y = _transition_mid_y(y_total)
        if mid_y is not None:
            # Reach the mid-Y waypoint FIRST, still at the bottom orientation and
            # tilt, then turn the wrist there. Rotating during the traverse is
            # what struck the door.
            _log(
                config,
                "[TableB DXF Tool2] bottom -> top: travel to mid Y=%.3f at Z=%.1f before the J6 turn",
                mid_y,
                TOOL2_MID_TRANSITION_Z_MM,
            )
            # Normalise X at the mid-Y waypoint. Keeping the bottom pass's X here
            # would leave a single diagonal from far-positive X to a deep-negative
            # top prepoint after the turn -- the crossing the reach envelope does
            # not allow. Stepping X to 0 first splits it into two safe legs.
            mid_x = 0.0 if float(previous_position.x) > 0.0 else float(previous_position.x)
            _move(
                cps,
                config,
                _pose(
                    mid_x,
                    mid_y,
                    TOOL2_MID_TRANSITION_Z_MM,
                    TOOL2_SIDE_RZ_DEG["bottom"],
                    ry=TOOL2_EDGE_RY_DEG,
                ),
                velocity_profile="robotspeed",
                wait=True,
            )
    if (
        next_axis7 is not None
        and previous_position.axis7 is not None
        and abs(float(previous_position.axis7) - float(next_axis7)) > 1.0
        and (
            _safe_to_start_j7_during_tool2_side_transition(previous_position)
            or bottom_exit_prepared
        )
    ):
        _log(
            config,
            "[TableB DXF Tool2] side transition %s -> %s starts J7 %.3f -> %.3f during safe-height J6/RZ rotation",
            previous_position.side,
            next_side,
            float(previous_position.axis7),
            float(next_axis7),
        )
        _move(
            cps,
            config,
            None,
            seventh=float(next_axis7),
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True,
        )
        started_axis7 = True
    record_tool2_recovery_state(
        previous_position.side,
        _pose(previous_position.x, previous_position.y, previous_position.z, previous_position.rz),
        y_total=_active_y_total(),
    )
    _log(
        config,
        "[TableB DXF Tool2] side transition %s -> %s using coordinate RZ %.1f and relative J6 delta %.1f",
        previous_position.side,
        next_side,
        TOOL2_SIDE_RZ_DEG[next_side],
        j6_delta,
    )
    if not rz_already_commanded:
        guarded_move_only_j6r(cps, j6_delta, config, wait=True, context=f"Tool 2 side transition {previous_position.side} -> {next_side}")
    return started_axis7

