from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from Server_Better_V2 import communicate, moveOnlyJ6r

from .motion_config import robot_speed
from .tool2_side_geometry import TOOL2_LIFT_Z_MM, tool2_lift_z_for_side

_STATE_PATH = Path(__file__).resolve().parents[1] / "tool2_recovery_state.json"

# Tool 2 commanded sanding poses are on the side offset line: 15 mm for Side,
# 5 mm for Edge. After stop, reset RY, lift from that line, then move into a
# side-specific clearance corridor before the normal homing sequence.
TOOL2_HOMING_INWARD_CLEARANCE_MM = 95.0
TOOL2_BOTTOM_HOMING_CLEARANCE_Y_MM = 80.0
TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM = -80.0
TOOL2_HOMING_RZ_DEG = 90.0
TOOL2_HOMING_J6_DEG = 90.012

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


def _current_cartesian_pose(cps: Any, fallback: list[float] | None = None) -> list[float] | None:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActPos(0, 0, result)
    except Exception:
        ret = -1
    if ret == 0 and len(result) >= 12:
        try:
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
        return float(pose[1]) > TOOL2_BOTTOM_HOMING_CLEARANCE_Y_MM and float(pose[2]) < TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM
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


def _inward_clearance_pose(side: str, pose: list[float], inward_mm: float) -> list[float]:
    x, y, z, rx, ry, rz = [float(v) for v in pose[:6]]
    if side == "right":
        x += inward_mm
    elif side == "bottom":
        y = TOOL2_BOTTOM_HOMING_CLEARANCE_Y_MM
        z = TOOL2_BOTTOM_HOMING_CLEARANCE_Z_MM
    elif side == "left":
        x -= inward_mm
    elif side == "top":
        y -= inward_mm
    return [x, y, z, rx, ry, rz]


def recover_tool2_before_homing_if_needed(cps: Any, config: dict[str, Any]) -> bool:
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
    inward_mm = float(state.get("inward_clearance_mm") or TOOL2_HOMING_INWARD_CLEARANCE_MM)
    safe_z = float(state.get("safe_z_mm") or TOOL2_LIFT_Z_MM)
    fallback_pose = state.get("last_commanded_pose") if isinstance(state.get("last_commanded_pose"), list) else None
    current_pose = _current_cartesian_pose(cps, fallback_pose)
    if current_pose is None:
        _log(config, "[TableB DXF Tool2 Recovery] skipped: no current/fallback pose for side=%s", side)
        return False

    tcp = _tool2_tcp(config)
    ucs = _tool2_ucs(config)
    speed = robot_speed(config)
    neutral_current_pose = list(current_pose)
    neutral_current_pose[4] = 0.0
    lift_pose = list(neutral_current_pose)
    lift_pose[2] = safe_z
    clearance_pose = _inward_clearance_pose(side, lift_pose, inward_mm)

    _log(
        config,
        "[TableB DXF Tool2 Recovery] side=%s current=%s neutral_current=%s lift=%s clearance=%s",
        side,
        current_pose,
        neutral_current_pose,
        lift_pose,
        clearance_pose,
    )
    if abs(float(current_pose[4])) > 1e-6:
        communicate(
            cps=cps,
            config=config,
            point=neutral_current_pose,
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
        point=lift_pose,
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
        moveOnlyJ6r(cps, j6_delta, config, wait=True)
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

    clear_tool2_recovery_state()
    return True

