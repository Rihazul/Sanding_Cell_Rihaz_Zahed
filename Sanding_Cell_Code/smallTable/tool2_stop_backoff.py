"""Shared Table A Tool 2 (side/edge) stop back-off.

Tool 2 sands the door thickness by pressing the tool into the door edge with an X/Y force
(putForceXplus / putForceXminus / putForceYplus1 / putForceYminus1). When the operator
presses Stop, this helper backs the tool 5 mm straight OFF the door along that force axis
(opposite the force direction) with NO force, so the pressed contact releases immediately
and leaves no mark.

The pose's RY is preserved, which gives exactly what each operation needs:
  - Side poses carry RY=0  -> back off 5 mm with RY=0
  - Edge poses carry RY=22 -> back off 5 mm with RY=22

Best-effort: never raises, so stop cleanup always completes. Force control is turned OFF
before the move so it is a pure position move.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from Server_Better_V2 import (
    clear_stop,
    communicate,
    moveOnlyJ6r,
    putForceXminus,
    putForceXplus,
    putForceYminus1,
    putForceYplus1,
    request_stop,
    turn_vibration_off,
    waitForBlending,
)

TOOL2_STOP_BACKOFF_MM = 5.0

# Recovery (homing / restart) after a Tool 2 stop back-off.
# From the backed-off +-5 mm position, lift Z to a safe standoff, then (for non-top sides)
# read the current J6 and turn to an absolute homing J6 so the wrist reaches homing smoothly.
TOOL2_RECOVERY_LIFT_Z_MM = 50.0
TOOL2_RECOVERY_HOMING_J6_DEG = 90.0
_STATE_PATH = Path(__file__).resolve().parent / "tool2_stop_state.json"

# Which side each force function corresponds to, and whether it needs the J6->90 turn on
# recovery. The TOP side already sits at an orientation that reaches homing without a wrist
# turn, so it only needs the Z lift. The other three sides need the lift AND the J6 turn.
#   putForceYminus1 -> top    (RZ -90) : lift only, no rotation
#   putForceXplus   -> right  (RZ  0)  : lift + J6->90
#   putForceXminus  -> left   (RZ -180): lift + J6->90
#   putForceYplus1  -> bottom (RZ  90) : lift + J6->90
_SIDE_BY_FORCE_FUNC_NAME = {
    "putForceYminus1": ("top", False),
    "putForceXplus": ("right", True),
    "putForceXminus": ("left", True),
    "putForceYplus1": ("bottom", True),
}

# Per force function: (pose axis index, signed direction AWAY from the door).
#   putForceXplus presses +X -> back off -X ;  putForceXminus presses -X -> back off +X
#   putForceYplus1 presses +Y -> back off -Y ; putForceYminus1 presses -Y -> back off +Y
_BACKOFF_BY_FORCE_FUNC = {
    putForceXplus: (0, -1.0),
    putForceXminus: (0, +1.0),
    putForceYplus1: (1, -1.0),
    putForceYminus1: (1, +1.0),
}


def _read_actual_pose(cps: Any, tcp: str, ucs: str) -> list[float] | None:
    result: list[Any] = []
    try:
        if cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result) == 0 and len(result) >= 12:
            return [float(result[i]) for i in range(6, 12)]
    except Exception:
        return None
    return None


def tool2_backoff_on_stop(
    cps: Any,
    config: dict[str, Any],
    *,
    tcp: str,
    ucs: str,
    force_func: Any,
    robot_speed: float,
    last_pose: list[float] | None = None,
) -> bool:
    """On STOP, back Tool 2 5 mm off the door along the force axis, preserving RY.

    `force_func` is the putForce* used for this pass; it selects the back-off axis and the
    away-from-door direction. `last_pose` is the last commanded 6D pose (carries the correct
    RY) and is used if the live pose read fails. Returns True if a back-off move was issued.
    """
    logger = config.get("logger") if isinstance(config, dict) else None

    axis_dir = _BACKOFF_BY_FORCE_FUNC.get(force_func)
    if axis_dir is None:
        return False
    axis_index, direction = axis_dir

    # Force control OFF + vibration OFF before the retract so it is a pure position move.
    try:
        cps.HRIF_SetForceControlState(0, 0, 0)
    except Exception:
        pass
    try:
        turn_vibration_off(cps)
    except Exception:
        pass

    try:
        pose = _read_actual_pose(cps, tcp, ucs)
        if pose is None and last_pose is not None and len(last_pose) >= 6:
            pose = [float(v) for v in last_pose[:6]]
        if pose is None:
            if logger:
                logger.info("[tool2 stop back-off] skipped: no live or fallback pose.")
            return False

        backoff_pose = list(pose)
        backoff_pose[axis_index] += direction * TOOL2_STOP_BACKOFF_MM
        if logger:
            logger.info(
                "[tool2 stop back-off] %s %.1fmm away (axis=%s, RY=%.1f preserved): %s -> %s",
                getattr(force_func, "__name__", "force"),
                TOOL2_STOP_BACKOFF_MM,
                "X" if axis_index == 0 else "Y",
                backoff_pose[4],
                pose[:3],
                backoff_pose[:3],
            )

        # communicate() and its wait loop both bail while the stop flag is latched, so a
        # MoveL issued now would be a no-op. The back-off IS the intended stop action, so we
        # clear the flag only for this single retract move and re-latch it afterward.
        clear_stop()
        try:
            communicate(
                cps=cps,
                config=config,
                point=backoff_pose,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed,
                velocity_profile="robotspeed",
                wait=True,
            )
        finally:
            request_stop()
        # Remember which side/tcp/ucs we stopped on so a later Homing or task restart runs the
        # correct lift (+ J6 turn for non-top sides) from this backed-off position.
        _record_tool2_stop_state(getattr(force_func, "__name__", ""), tcp, ucs)
        return True
    except Exception as exc:
        if logger:
            logger.warning("[tool2 stop back-off] failed (continuing): %s", exc)
        return False


def _record_tool2_stop_state(force_func_name: str, tcp: str, ucs: str) -> None:
    if force_func_name not in _SIDE_BY_FORCE_FUNC_NAME:
        return
    side, needs_rotation = _SIDE_BY_FORCE_FUNC_NAME[force_func_name]
    payload = {
        "active": True,
        "tool": 2,
        "side": side,
        "needs_rotation": bool(needs_rotation),
        "tcp": tcp,
        "ucs": ucs,
    }
    try:
        _STATE_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except Exception:
        pass


def clear_tool2_stop_state() -> None:
    try:
        _STATE_PATH.unlink(missing_ok=True)
    except Exception:
        pass


def _load_tool2_stop_state() -> dict[str, Any] | None:
    try:
        payload = json.loads(_STATE_PATH.read_text(encoding="utf-8"))
    except Exception:
        return None
    if not isinstance(payload, dict) or not payload.get("active") or payload.get("tool") != 2:
        return None
    if str(payload.get("side") or "") not in {"top", "right", "left", "bottom"}:
        return None
    return payload


def _read_actual_j6(cps: Any) -> float | None:
    result: list[Any] = []
    try:
        if cps.HRIF_ReadActJointPos(0, 0, result) == 0 and len(result) >= 6:
            return float(result[5])
    except Exception:
        return None
    return None


def recover_tool2_after_stop_if_needed(cps: Any, config: dict[str, Any]) -> bool:
    """Run the Tool 2 lift recovery from a backed-off stop position, before Homing or restart.

    From the +-5 mm backed-off pose: lift Z to a safe standoff (so the tool clears the surface
    when lifting), then for non-top sides read the current J6 and turn to the absolute homing
    J6 (90 deg) so the wrist reaches the homing position smoothly. Top side needs the lift only.

    Best-effort: never raises. Returns True if a recovery ran. Clears the state when done so it
    runs exactly once per stop.
    """
    logger = config.get("logger") if isinstance(config, dict) else None
    state = _load_tool2_stop_state()
    if not state:
        return False

    side = str(state["side"])
    needs_rotation = bool(state.get("needs_rotation"))
    tcp = str(state.get("tcp") or "")
    ucs = str(state.get("ucs") or "")
    if not tcp or not ucs:
        # Fall back to the standard Tool 2 plane-1 coords.
        coords = config.get("coords", {}) if isinstance(config, dict) else {}
        tcp = tcp or str(coords.get("tcptool2plane1") or "")
        ucs = ucs or str(coords.get("ucsTable1") or "")

    robot_speed = 0.9
    try:
        cycle_cfg_path = Path("./configs/cycleData.json")
        robot_speed = float(json.loads(cycle_cfg_path.read_text(encoding="utf-8")).get("robotSpeed", 0.9))
    except Exception:
        robot_speed = 0.9

    try:
        pose = _read_actual_pose(cps, tcp, ucs)
        if pose is None:
            if logger:
                logger.warning("[tool2 recovery] skipped: could not read current pose (side=%s).", side)
            clear_tool2_stop_state()
            return False

        # 1. Lift straight up to the safe standoff, preserving XY and orientation (RY/RZ), so
        #    the tool clears the surface before any wrist rotation.
        lift_pose = list(pose)
        lift_pose[2] = TOOL2_RECOVERY_LIFT_Z_MM
        if logger:
            logger.info(
                "[tool2 recovery] side=%s lift to Z=%.1f (needs_rotation=%s) from %s",
                side, TOOL2_RECOVERY_LIFT_Z_MM, needs_rotation, pose[:3],
            )
        communicate(
            cps=cps,
            config=config,
            point=lift_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=True,
        )

        # 2. Non-top sides: read the current J6, then turn to absolute homing J6 (90 deg) so the
        #    wrist reaches homing smoothly. Top needs no rotation.
        if needs_rotation:
            current_j6 = _read_actual_j6(cps)
            if current_j6 is not None:
                delta = TOOL2_RECOVERY_HOMING_J6_DEG - current_j6
                if abs(delta) > 1e-3:
                    if logger:
                        logger.info(
                            "[tool2 recovery] side=%s J6 %.3f -> %.1f (delta=%.3f)",
                            side, current_j6, TOOL2_RECOVERY_HOMING_J6_DEG, delta,
                        )
                    waitForBlending(cps=cps, config=config)
                    moveOnlyJ6r(cps, delta, config, wait=True)
            elif logger:
                logger.warning("[tool2 recovery] side=%s J6 read failed; skipping wrist turn.", side)

        clear_tool2_stop_state()
        return True
    except Exception as exc:
        if logger:
            logger.warning("[tool2 recovery] failed (continuing): %s", exc)
        clear_tool2_stop_state()
        return False
