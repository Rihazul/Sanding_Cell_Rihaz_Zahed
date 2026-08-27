from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from Server_Better_V2 import clear_stop, communicate, request_stop, setUCS_TCP

from .joint_safety import guarded_move_j6_to_absolute, guarded_move_only_j6r
from .motion_config import robot_speed
from .tool2_side_geometry import (
    TOOL2_APPROACH_OUTWARD_MM,
    TOOL2_CONTACT_Z_MM,
    TOOL2_LEFT_PREFORCE_LOCAL_X_MM,
    tool2_lift_z_for_side,
)

_STATE_PATH = Path(__file__).resolve().parents[1] / "tool2_recovery_state.json"

# Tool 2 commanded sanding poses are on the side offset line: 15 mm for Side,
# 5 mm for Edge. After stop, reset RY, lift from that line, then move into a
# side-specific clearance corridor before the normal homing sequence.
TOOL2_BOTTOM_HOMING_CLEARANCE_Y_MM = 80.0
# Left/right home through Y=200 at the safe height: below that the arm is too
# folded to reposition to RZ=90 comfortably, so it needs the extra room whether
# the operator is homing or restarting the task.
TOOL2_UPPER_HOMING_CLEARANCE_Y_MM = 200.0
TOOL2_HOMING_CLEARANCE_Z_MM = -80.0
TOOL2_HOMING_RZ_DEG = 90.0
TOOL2_HOMING_J6_DEG = 90.012

# On STOP during Tool 2 side/edge sanding, back the tool straight off the door corner/side by
# this much along the side's force axis (opposite the force direction), with NO force, so it
# releases the contact immediately and leaves no mark. Homing then runs the normal Tool 2
# recovery. Per side: (axis_index_in_pose, signed_direction_away_from_door).
#   right presses +X -> back off -X;  left presses -X -> back off +X
#   bottom presses +Y -> back off -Y; top  presses -Y -> back off +Y
TOOL2_STOP_BACKOFF_MM = TOOL2_APPROACH_OUTWARD_MM
_TOOL2_STOP_BACKOFF_AXIS = {
    "right": (0, -1.0),
    "left": (0, +1.0),
    "bottom": (1, -1.0),
    "top": (1, +1.0),
}

# Bottom is the special side on stop: instead of backing off 15 mm at RY=0 like
# the others, it retracts to Y=-5 holding RY=-22 (side AND edge). That pose is
# already the one homing and task-restart run from, so no Z lift is needed.
TOOL2_BOTTOM_STOP_Y_MM = -5.0
TOOL2_BOTTOM_STOP_RY_DEG = -22.0

_TOOL2_SIDE_RZ_DEG = {
    "right": 0.0,
    "bottom": 90.0,
    "left": 180.0,
    "top": -90.0,
}

_TOOL2_TO_HOMING_J6_DEG = {
    "right": 90.0,
    "bottom": 0.0,
    "left": -90.0,
    "top": -180.0,
}


def _log(config: dict[str, Any], message: str, *args: Any) -> None:
    logger = config.get("logger") if isinstance(config, dict) else None
    if logger:
        logger.info(message, *args)


def _tool2_tcp(config: dict[str, Any]) -> str:
    coords = config.get("coords") or {}
    tcp = coords.get("tcpSideTool") or coords.get("tcptool2plane2")
    if not tcp:
        raise RuntimeError("Missing Tool 2 TCP config: coords.tcpSideTool or coords.tcptool2plane2")
    return str(tcp)


def _tool2_ucs(config: dict[str, Any]) -> str:
    try:
        return str(config["coords"]["ucsTable2"])
    except KeyError as error:
        raise RuntimeError("Missing Tool 2 UCS config: coords.ucsTable2") from error


def record_tool2_recovery_state(
    side: str,
    pose: list[float] | None = None,
    *,
    active: bool = True,
    y_total: float | None = None,
    tcp: str | None = None,
    ucs: str | None = None,
) -> None:
    if side not in _TOOL2_SIDE_RZ_DEG:
        return
    payload: dict[str, Any] = {
        "active": bool(active),
        "tool": 2,
        "side": side,
        "safe_z_mm": tool2_lift_z_for_side(side),
        "timestamp": time.time(),
    }
    # The stored pose is meaningless without the frame it was taken in. After a
    # server restart the config is re-read and the active TCP/UCS on the
    # controller are whatever the last motion left, so recovery must replay the
    # frame the pose belongs to rather than assume the Tool 2 defaults.
    if tcp:
        payload["tcp"] = str(tcp)
    if ucs:
        payload["ucs"] = str(ucs)
    if y_total is not None:
        # Kept so top-side stop recovery can land on y_total + 15 without the
        # stop path having to thread door size through every caller.
        payload["y_total_mm"] = float(y_total)
    if pose is not None:
        payload["last_commanded_pose"] = [float(v) for v in pose[:6]]
    try:
        _STATE_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except Exception:
        pass


def clear_tool2_recovery_state() -> None:
    try:
        _STATE_PATH.unlink(missing_ok=True)
    except Exception:
        pass


def tool2_recovery_pending() -> bool:
    """Return True when a stopped Tool 2 pose still needs safe recovery."""
    return _load_tool2_recovery_state() is not None


def tool2_backoff_from_contact_on_stop(cps: Any, config: dict[str, Any]) -> bool:
    """On STOP, back Tool 2 straight off the door side/corner by TOOL2_STOP_BACKOFF_MM.

    Moves outward along the side's force axis (opposite the force direction) with NO force,
    so the pressed contact is released immediately and no mark is left. Force control is
    already OFF (stop_active_motion) at this point, so this is a pure position move.

    communicate() and its wait loop both bail while the stop flag is latched, so we clear it
    only for this single retract move and re-latch it afterward, preserving the caller's
    "stopped" semantics. Best-effort: never raises; homing still runs the full Tool 2
    recovery afterward. Returns True if a back-off move was issued.
    """
    logger = config.get("logger") if isinstance(config, dict) else None
    state = _load_tool2_recovery_state()
    if not state:
        return False
    side = str(state.get("side") or "")
    axis_dir = _TOOL2_STOP_BACKOFF_AXIS.get(side)
    if axis_dir is None:
        return False
    axis_index, direction = axis_dir
    try:
        tcp, ucs = _frame_from_state(state, config, cps)
        fallback_pose = state.get("last_commanded_pose") if isinstance(state.get("last_commanded_pose"), list) else None
        current_pose = _current_tool2_coord_pose(cps, tcp, ucs, fallback_pose)
        if current_pose is None:
            if logger:
                logger.info("[TableB DXF Tool2 Recovery] stop back-off skipped: no current pose for side=%s", side)
            return False
        backoff_pose = [float(v) for v in current_pose[:6]]
        if side == "bottom":
            # Bottom retracts to its own travel line and keeps RY=-22 (side and
            # edge alike) so homing / restart can continue straight from here.
            backoff_pose[1] = TOOL2_BOTTOM_STOP_Y_MM
            backoff_pose[4] = TOOL2_BOTTOM_STOP_RY_DEG
        else:
            # Top/left/right sit 15 mm clear of the face, flat at RY=0. Use the
            # side's absolute offset line rather than adding to the current pose,
            # otherwise a stop during side sanding (already on that line) would
            # push out to 30 mm and be pulled back by the pre-homing retract.
            state_y_total = state.get("y_total_mm")
            try:
                state_y_total = float(state_y_total) if state_y_total is not None else None
            except (TypeError, ValueError):
                state_y_total = None
            safe_pose = _side_safe_contact_pose(side, backoff_pose, state_y_total)
            backoff_pose[axis_index] = safe_pose[axis_index]
            backoff_pose[4] = 0.0
        backoff_pose[5] = _TOOL2_SIDE_RZ_DEG[side]
        if logger:
            logger.info(
                "[TableB DXF Tool2 Recovery] stop back-off side=%s %s: %s -> %s (RY=%.1f RZ=%.1f)",
                side,
                "Y=-5 RY=-22 hold" if side == "bottom"
                else f"axis={'X' if axis_index == 0 else 'Y'} {TOOL2_STOP_BACKOFF_MM:.1f}mm away",
                current_pose[:3],
                backoff_pose[:3],
                backoff_pose[4],
                backoff_pose[5],
            )
        clear_stop()
        try:
            communicate(
                cps=cps,
                config=config,
                point=backoff_pose,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed(config),
                velocity_profile="robotspeed",
                wait=True,
            )
        finally:
            request_stop()
        return True
    except Exception as exc:
        if logger:
            logger.warning("[TableB DXF Tool2 Recovery] stop back-off failed (continuing): %s", exc)
        return False


# Which table is tilted to 45 degrees identifies the frame a stopped Tool 2 pose
# belongs to. After a server restart the controller resets to Base/default TCP,
# so the saved coordinates are meaningless until the frame is re-established --
# and homing would otherwise run its safety moves in the wrong plane.
#
#   Table A at 45 deg -> UCS Plane_1, TCP tcptool2plane1
#   Table B at 45 deg -> UCS Plane_2, TCP tcptool2plane2 / tcpSideTool
TABLE_A_DI_CLOSED_45 = ("1", "0")
TABLE_B_CO_INDEX = 1
TABLE_B_CO_CLOSED_45 = "1"


def _read_table_a_tilt(cps: Any) -> tuple[str, str] | None:
    di0: list[Any] = []
    di1: list[Any] = []
    try:
        ret0 = cps.HRIF_ReadBoxDI(0, 0, di0)
        ret1 = cps.HRIF_ReadBoxDI(0, 1, di1)
    except Exception:
        return None
    if ret0 != 0 or ret1 != 0 or not di0 or not di1:
        return None
    return (str(di0[0]), str(di1[0]))


def _read_table_b_tilt(cps: Any) -> str | None:
    state: list[Any] = []
    try:
        ret = cps.HRIF_ReadBoxCO(0, TABLE_B_CO_INDEX, state)
    except Exception:
        return None
    if ret != 0 or not state:
        return None
    return str(state[0])


def _read_active_ucs_name(cps: Any, config: dict[str, Any]) -> str | None:
    """Name of the UCS the controller currently has active, or None.

    HRIF_ReadCurUCS returns a POSE, not a name, so it is matched back against the
    named UCSs via HRIF_ReadUCSByName. A restart leaves the controller on Base,
    whose pose is all zeros -- that is the case this exists to detect.
    """
    current: list[Any] = []
    try:
        if cps.HRIF_ReadCurUCS(0, 0, current) != 0 or len(current) < 6:
            return None
        active = tuple(round(float(v), 3) for v in list(current)[:6])
    except (TypeError, ValueError, Exception):
        return None
    if all(abs(v) <= 1e-3 for v in active):
        return "Base"
    coords = config.get("coords") or {}
    for key in ("ucsTable1", "ucsTable2"):
        name = coords.get(key)
        if not name:
            continue
        probe: list[Any] = []
        try:
            if cps.HRIF_ReadUCSByName(0, 0, str(name), probe) != 0 or len(probe) < 6:
                continue
            known = tuple(round(float(v), 3) for v in list(probe)[:6])
        except (TypeError, ValueError, Exception):
            continue
        if all(abs(a - b) <= 0.5 for a, b in zip(active, known)):
            return str(name)
    return None


def tool2_active_frame_is_reset(cps: Any, config: dict[str, Any]) -> bool:
    """True when the controller is on Base rather than a table plane.

    After a server restart the active UCS resets to Base while Tool 2 is still
    physically parked against the door in a table plane. Any pose commanded in
    Base would be interpreted in the wrong frame entirely.
    """
    return _read_active_ucs_name(cps, config) == "Base"


def tool2_frame_from_table_tilt(
    cps: Any, config: dict[str, Any]
) -> tuple[str, str, str] | None:
    """Resolve (table, tcp, ucs) from which table is at 45 degrees.

    This is the only reliable way to recover the frame after a server restart:
    the tilted table physically holds the door Tool 2 was working on, so it says
    which plane the stopped pose was recorded in. Returns None when neither table
    is tilted or both are, since then the frame cannot be established.
    """
    coords = config.get("coords") or {}
    a_tilted = _read_table_a_tilt(cps) == TABLE_A_DI_CLOSED_45
    b_tilted = _read_table_b_tilt(cps) == TABLE_B_CO_CLOSED_45
    if a_tilted and not b_tilted:
        tcp = coords.get("tcptool2plane1")
        ucs = coords.get("ucsTable1")
        return ("A", str(tcp), str(ucs)) if tcp and ucs else None
    if b_tilted and not a_tilted:
        tcp = coords.get("tcpSideTool") or coords.get("tcptool2plane2")
        ucs = coords.get("ucsTable2")
        return ("B", str(tcp), str(ucs)) if tcp and ucs else None
    return None


def tool2_recovery_frame_unresolved(cps: Any, config: dict[str, Any]) -> str | None:
    """Reason homing must not run, or None when the frame is known.

    Only blocks when Tool 2 recovery is pending AND the frame cannot be
    established -- neither table tilted, both tilted, or the IO unreadable.
    Running the recovery moves in the wrong plane would drive the tool through
    the door, so refusing is the safe outcome.
    """
    state = _load_tool2_recovery_state()
    if not state:
        return None
    if state.get("tcp") and state.get("ucs"):
        return None
    if tool2_frame_from_table_tilt(cps, config) is not None:
        return None
    return (
        "Tool 2 is stopped holding the door, but its coordinate frame cannot be "
        "identified: no single table is at the 45 degree position. Tilt the table "
        "that holds the door (Table A -> Plane_1, Table B -> Plane_2) before homing."
    )


def _frame_from_state(
    state: dict[str, Any],
    config: dict[str, Any],
    cps: Any = None,
) -> tuple[str, str]:
    """TCP/UCS the stopped pose belongs to.

    Order of trust:
      1. the tilted table, read live -- authoritative after a restart, because the
         table physically holds the door Tool 2 is parked against. Checked FIRST
         when the controller has reset to Base, since a recorded frame from
         before the restart cannot be assumed to still be applied;
      2. the frame recorded alongside the pose;
      3. the Tool 2 config defaults, as a last resort.
    """
    recorded_tcp = str(state.get("tcp") or "")
    recorded_ucs = str(state.get("ucs") or "")
    if cps is not None:
        resolved = tool2_frame_from_table_tilt(cps, config)
        if resolved is not None:
            # A reset controller means nothing is applied; trust the table.
            if tool2_active_frame_is_reset(cps, config) or not (recorded_tcp and recorded_ucs):
                return resolved[1], resolved[2]
    if recorded_tcp and recorded_ucs:
        return recorded_tcp, recorded_ucs
    return recorded_tcp or _tool2_tcp(config), recorded_ucs or _tool2_ucs(config)


def _load_tool2_recovery_state() -> dict[str, Any] | None:
    try:
        payload = json.loads(_STATE_PATH.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not isinstance(payload, dict) or not payload.get("active"):
        return None
    if payload.get("tool") != 2:
        return None
    side = str(payload.get("side") or "")
    if side not in _TOOL2_SIDE_RZ_DEG:
        return None
    return payload


def _current_tool2_coord_pose(
    cps: Any,
    tcp: str,
    ucs: str,
    fallback: list[float] | None = None,
) -> list[float] | None:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result)
    except Exception:
        ret = -1
    if ret == 0:
        try:
            if len(result) >= 12:
                return [float(result[i]) for i in range(6, 12)]
        except (TypeError, ValueError):
            pass
    if fallback and len(fallback) >= 6:
        try:
            return [float(v) for v in fallback[:6]]
        except (TypeError, ValueError):
            return None
    return None


def _current_j6(cps: Any) -> float | None:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActJointPos(0, 0, result)
    except Exception:
        ret = -1
    if ret == 0 and len(result) >= 6:
        try:
            return float(result[5])
        except (TypeError, ValueError):
            return None
    return None


def _is_upper_homing_clearance_zone(pose: list[float]) -> bool:
    try:
        return (
            float(pose[1]) >= TOOL2_UPPER_HOMING_CLEARANCE_Y_MM - 1e-6
            and float(pose[2]) <= TOOL2_HOMING_CLEARANCE_Z_MM + 1e-6
        )
    except (TypeError, ValueError, IndexError):
        return False


def _j6_delta_to_homing(cps: Any, config: dict[str, Any], side: str, clearance_pose: list[float]) -> float:
    if _is_upper_homing_clearance_zone(clearance_pose):
        current_j6 = _current_j6(cps)
        if current_j6 is not None:
            delta = TOOL2_HOMING_J6_DEG - current_j6
            _log(
                config,
                "[TableB DXF Tool2 Recovery] upper clearance J6 normalize current=%.3f target=%.3f delta=%.3f",
                current_j6,
                TOOL2_HOMING_J6_DEG,
                delta,
            )
            return delta
        _log(config, "[TableB DXF Tool2 Recovery] upper clearance J6 read failed; using side fallback for side=%s", side)
    return float(_TOOL2_TO_HOMING_J6_DEG.get(side, 0.0))


def _side_safe_contact_pose(side: str, pose: list[float], y_total: float | None = None) -> list[float]:
    """Pose 15 mm clear of the sanded face, flat at RY=0, at the side's own RZ.

    Top/left/right all retract along their own outward normal and hold contact
    height (Z=0) — they do not lift. Bottom does not use this helper; it has its
    own Y=-5 / RY=-22 retract.
    """
    x, y, z, rx, ry, _rz = [float(v) for v in pose[:6]]
    rz = _TOOL2_SIDE_RZ_DEG[side]
    recovery_z = TOOL2_CONTACT_Z_MM if z > -5.0 else z

    if side == "right":
        x = -TOOL2_APPROACH_OUTWARD_MM
    elif side == "bottom":
        y = -TOOL2_APPROACH_OUTWARD_MM
    elif side == "left":
        x = TOOL2_LEFT_PREFORCE_LOCAL_X_MM
    elif side == "top":
        # Land on the 15 mm top offset line regardless of whether the pass was
        # side (already 15 mm out) or edge (5 mm out at RY=-22).
        if y_total is not None:
            y = float(y_total) + TOOL2_APPROACH_OUTWARD_MM
        elif abs(ry) > 1e-6:
            # Edge uses a 5 mm line; side-safe top placement is 10 mm farther out.
            y += 10.0

    return [x, y, recovery_z, rx, 0.0, rz]


def _homing_clearance_pose(side: str, side_safe_pose: list[float]) -> list[float]:
    clearance_pose = list(side_safe_pose)
    y = float(clearance_pose[1])
    if side in {"left", "right"} and y < TOOL2_UPPER_HOMING_CLEARANCE_Y_MM:
        clearance_pose[1] = TOOL2_UPPER_HOMING_CLEARANCE_Y_MM
    elif y <= TOOL2_UPPER_HOMING_CLEARANCE_Y_MM:
        clearance_pose[1] = TOOL2_BOTTOM_HOMING_CLEARANCE_Y_MM
    current_z = float(clearance_pose[2])
    if current_z > TOOL2_HOMING_CLEARANCE_Z_MM:
        clearance_pose[2] = TOOL2_HOMING_CLEARANCE_Z_MM
    clearance_pose[4] = 0.0
    return clearance_pose


def recover_tool2_before_homing_if_needed(
    cps: Any,
    config: dict[str, Any],
    *,
    clear_state: bool = True,
) -> bool:
    """Retract Tool 2 safely after a stop before running the normal homing path.

    Tool 2 uses side-specific lifted clearance in Table B plane 2. If the
    operator stops during side sanding, the commanded XY is already on the
    15 mm safe-offset line. Homing must lift there first, move inward into a
    safer clearance corridor, then allow the existing homing sequence to continue.
    """
    state = _load_tool2_recovery_state()
    if not state:
        return False

    side = str(state["side"])
    fallback_pose = state.get("last_commanded_pose") if isinstance(state.get("last_commanded_pose"), list) else None
    tcp, ucs = _frame_from_state(state, config, cps)
    # Re-apply the frame before anything else. After a restart the controller is
    # on Base with the default TCP while the tool is still parked in a table
    # plane, so every pose below -- including the live read -- would otherwise be
    # interpreted in the wrong frame.
    if tool2_active_frame_is_reset(cps, config):
        _log(
            config,
            "[TableB DXF Tool2 Recovery] controller is on Base; applying stopped frame tcp=%s ucs=%s before recovery",
            tcp,
            ucs,
        )
        try:
            setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
        except Exception as error:
            raise RuntimeError(
                f"Tool 2 recovery could not restore the stopped frame (tcp={tcp}, ucs={ucs}): {error}"
            ) from error
    current_pose = _current_tool2_coord_pose(cps, tcp, ucs, fallback_pose)
    if current_pose is None:
        _log(config, "[TableB DXF Tool2 Recovery] skipped: no Tool 2 TCP/UCS current/fallback pose for side=%s tcp=%s ucs=%s", side, tcp, ucs)
        return False

    speed = robot_speed(config)

    # BOTTOM side (side OR edge): two combined moves off the bottom edge to a safe standoff,
    # then let normal homing continue. This deliberately bypasses the shared straighten
    # (RY=0) / 3-step retract used by the other sides, to remove redundant bottom motion.
    #   Move 1: RY=-22, Y=-5, Z=0  (at the current X, RZ=90 held)
    #   Move 2: Y=20,  Z=-50       (X, RY=-22, RZ=90 all held)
    if side == "bottom":
        cur_x, _cur_y, cur_z, rx, _ry, _rz = [float(v) for v in current_pose[:6]]
        rz = _TOOL2_SIDE_RZ_DEG["bottom"]
        # Bottom needs no Z lift and no clearance climb: retracting to Y=-5 while
        # holding RY=-22 already leaves it in the pose homing and restart run
        # from. Z is held wherever the pass stopped (contact height in practice).
        retract = [cur_x, TOOL2_BOTTOM_STOP_Y_MM, cur_z, rx, TOOL2_BOTTOM_STOP_RY_DEG, rz]
        _log(
            config,
            "[TableB DXF Tool2 Recovery] bottom recovery current=%s retract=%s (no Z lift; homing continues from here)",
            current_pose, retract,
        )
        communicate(
            cps=cps,
            config=config,
            point=retract,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=speed,
            velocity_profile="robotspeed",
            wait=True,
        )
        if clear_state:
            clear_tool2_recovery_state()
        return True

    state_y_total = state.get("y_total_mm")
    try:
        state_y_total = float(state_y_total) if state_y_total is not None else None
    except (TypeError, ValueError):
        state_y_total = None
    side_safe_pose = _side_safe_contact_pose(side, current_pose, state_y_total)
    clearance_pose = _homing_clearance_pose(side, side_safe_pose)

    _log(
        config,
        "[TableB DXF Tool2 Recovery] side=%s current=%s combined_side_safe=%s clearance=%s",
        side,
        current_pose,
        side_safe_pose,
        clearance_pose,
    )
    communicate(
        cps=cps,
        config=config,
        point=side_safe_pose,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    communicate(
        cps=cps,
        config=config,
        point=clearance_pose,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    j6_delta = _j6_delta_to_homing(cps, config, side, clearance_pose)
    homing_orientation_pose = list(clearance_pose)
    homing_orientation_pose[4] = 0.0
    homing_orientation_pose[5] = TOOL2_HOMING_RZ_DEG
    if abs(j6_delta) > 1e-6:
        _log(
            config,
            "[TableB DXF Tool2 Recovery] side=%s applying homing J6 delta=%.3f before RZ=%s",
            side,
            j6_delta,
            TOOL2_HOMING_RZ_DEG,
        )
        if _is_upper_homing_clearance_zone(clearance_pose):
            guarded_move_j6_to_absolute(
                cps,
                TOOL2_HOMING_J6_DEG,
                config,
                wait=True,
                context=f"Tool 2 recovery absolute J6 homing from {side}",
            )
        else:
            guarded_move_only_j6r(cps, j6_delta, config, wait=True, context=f"Tool 2 recovery to homing from {side}")
    communicate(
        cps=cps,
        config=config,
        point=homing_orientation_pose,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=speed,
        velocity_profile="robotspeed",
        wait=True,
    )

    if clear_state:
        clear_tool2_recovery_state()
    return True
