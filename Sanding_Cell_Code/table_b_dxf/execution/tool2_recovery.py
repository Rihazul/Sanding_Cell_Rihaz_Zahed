from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from Server_Better_V2 import clear_stop, communicate, request_stop

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
TOOL2_UPPER_HOMING_CLEARANCE_Y_MM = 100.0
TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM = -80.0
TOOL2_HOMING_RZ_DEG = 90.0
TOOL2_HOMING_J6_DEG = 90.012

# Bottom-side stop -> homing retract (side OR edge). Two combined moves that pull the tool off
# the bottom edge and up to a safe standoff before the normal homing runs, replacing the older
# straighten/lift/standoff sequence to avoid redundant motion. RY=-22 and RZ=90 are held across
# both moves; only Y and Z change. X is left at wherever the pass stopped.
TOOL2_BOTTOM_RECOVERY_RY_DEG = -22.0
TOOL2_BOTTOM_RECOVERY_STEP1_Y_MM = -5.0
TOOL2_BOTTOM_RECOVERY_STEP1_Z_MM = 0.0
TOOL2_BOTTOM_RECOVERY_STEP2_Y_MM = 20.0
TOOL2_BOTTOM_RECOVERY_STEP2_Z_MM = -50.0

# On STOP during Tool 2 side/edge sanding, back the tool straight off the door corner/side by
# this much along the side's force axis (opposite the force direction), with NO force, so it
# releases the contact immediately and leaves no mark. Homing then runs the normal Tool 2
# recovery. Per side: (axis_index_in_pose, signed_direction_away_from_door).
#   right presses +X -> back off -X;  left presses -X -> back off +X
#   bottom presses +Y -> back off -Y; top  presses -Y -> back off +Y
TOOL2_STOP_BACKOFF_MM = 5.0
_TOOL2_STOP_BACKOFF_AXIS = {
    "right": (0, -1.0),
    "left": (0, +1.0),
    "bottom": (1, -1.0),
    "top": (1, +1.0),
}

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


def record_tool2_recovery_state(side: str, pose: list[float] | None = None, *, active: bool = True) -> None:
    if side not in _TOOL2_SIDE_RZ_DEG:
        return
    payload: dict[str, Any] = {
        "active": bool(active),
        "tool": 2,
        "side": side,
        "safe_z_mm": tool2_lift_z_for_side(side),
        "timestamp": time.time(),
    }
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
        tcp = _tool2_tcp(config)
        ucs = _tool2_ucs(config)
        fallback_pose = state.get("last_commanded_pose") if isinstance(state.get("last_commanded_pose"), list) else None
        current_pose = _current_tool2_coord_pose(cps, tcp, ucs, fallback_pose)
        if current_pose is None:
            if logger:
                logger.info("[TableB DXF Tool2 Recovery] stop back-off skipped: no current pose for side=%s", side)
            return False
        backoff_pose = [float(v) for v in current_pose[:6]]
        backoff_pose[axis_index] += direction * TOOL2_STOP_BACKOFF_MM
        if logger:
            logger.info(
                "[TableB DXF Tool2 Recovery] stop back-off side=%s axis=%s %.1fmm away: %s -> %s",
                side,
                "X" if axis_index == 0 else "Y",
                TOOL2_STOP_BACKOFF_MM,
                current_pose[:3],
                backoff_pose[:3],
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
            and float(pose[2]) <= TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM + 1e-6
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


def _side_safe_contact_pose(side: str, pose: list[float]) -> list[float]:
    x, y, z, rx, ry, _rz = [float(v) for v in pose[:6]]
    rz = _TOOL2_SIDE_RZ_DEG[side]
    recovery_z = TOOL2_CONTACT_Z_MM if z > -5.0 else z

    if side == "right":
        x = -TOOL2_APPROACH_OUTWARD_MM
    elif side == "bottom":
        y = -TOOL2_APPROACH_OUTWARD_MM
    elif side == "left":
        x = TOOL2_LEFT_PREFORCE_LOCAL_X_MM
    elif side == "top" and abs(ry) > 1e-6:
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
    if current_z > TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM:
        clearance_pose[2] = TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM
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
    tcp = _tool2_tcp(config)
    ucs = _tool2_ucs(config)
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
        cur_x, _cur_y, _cur_z, rx, _ry, _rz = [float(v) for v in current_pose[:6]]
        rz = _TOOL2_SIDE_RZ_DEG["bottom"]
        move1 = [cur_x, TOOL2_BOTTOM_RECOVERY_STEP1_Y_MM, TOOL2_BOTTOM_RECOVERY_STEP1_Z_MM,
                 rx, TOOL2_BOTTOM_RECOVERY_RY_DEG, rz]
        move2 = [cur_x, TOOL2_BOTTOM_RECOVERY_STEP2_Y_MM, TOOL2_BOTTOM_RECOVERY_STEP2_Z_MM,
                 rx, TOOL2_BOTTOM_RECOVERY_RY_DEG, rz]
        _log(
            config,
            "[TableB DXF Tool2 Recovery] bottom recovery current=%s move1=%s move2=%s",
            current_pose, move1, move2,
        )
        for point in (move1, move2):
            communicate(
                cps=cps,
                config=config,
                point=point,
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

    side_safe_pose = _side_safe_contact_pose(side, current_pose)
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
    if abs(float(clearance_pose[1]) - float(side_safe_pose[1])) > 1e-6:
        clearance_xy_pose = list(side_safe_pose)
        clearance_xy_pose[1] = clearance_pose[1]
        communicate(
            cps=cps,
            config=config,
            point=clearance_xy_pose,
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
