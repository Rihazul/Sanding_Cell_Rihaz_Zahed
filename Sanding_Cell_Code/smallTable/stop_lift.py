"""Shared Table A stop-recovery lift for Tool 1/3/4 force-locked sanding.

When the operator presses Stop mid-sanding, Table A operations press the tool DOWN into
the surface under Z- force control (putForceZminus). This helper is called from each
force-locked sanding helper's `finally` on stop to do the CNC-style recovery:

  1. force control OFF + vibration OFF  (so the tool stops pressing)
  2. lift the tool STRAIGHT UP to the operation's pre-height (+Z), with NO force, so it
     releases the surface immediately and leaves no drag mark.

From the pre-height the operator can re-send the task or run homing. Table A presses Z-,
so "lift" means going to a MORE-positive Z (the pre-height, e.g. +15, or +13 for modelD).

This mirrors the Table B stop-lift. It is best-effort: it never raises, so stop cleanup
always completes even if the controller rejects the move.
"""

from __future__ import annotations

from typing import Any

from Server_Better_V2 import (
    clear_stop,
    communicate,
    releaseForce,
    request_stop,
    stop_requested,
    turn_vibration_off,
)


def _logger(config: dict[str, Any]):
    return config.get("logger") if isinstance(config, dict) else None


def _read_actual_xyz(cps: Any) -> tuple[float, float, float] | None:
    result: list[Any] = []
    try:
        if cps.HRIF_ReadActPos(0, 0, result) == 0 and len(result) >= 9:
            return (float(result[6]), float(result[7]), float(result[8]))
    except Exception:
        return None
    return None


def stop_lift_to_preheight_tableA(
    cps: Any,
    config: dict[str, Any],
    *,
    tcp: str,
    ucs: str,
    preheight_z_mm: float,
    robot_speed: float,
    last_xy: tuple[float, float] | list[float] | None = None,
) -> bool:
    """On STOP, release force and lift the tool straight up to the pre-height with no force.

    Reads the tool's actual XY so a mid-move stop retracts straight up (no lateral scrub);
    falls back to `last_xy` if the read fails. Table A presses Z-, so the pre-height is a
    MORE-positive Z; if the tool is somehow already at/above it, the lift is skipped so we
    never push back down into the surface.

    Returns True if a lift move was issued. Best-effort: never raises.
    """
    logger = _logger(config)

    # Ensure force control is off before the retract so it is a pure position move.
    try:
        cps.HRIF_SetForceControlState(0, 0, 0)
    except Exception:
        pass
    try:
        turn_vibration_off(cps)
    except Exception:
        pass
    try:
        releaseForce(cps=cps, config=config, wait_for_blending=False)
    except Exception:
        pass

    try:
        actual = _read_actual_xyz(cps)
        if actual is not None:
            lift_x, lift_y, actual_z = actual
            # Table A presses Z-, so pre-height is above the surface (more positive Z). If
            # already at/above it, nothing to lift.
            if actual_z >= preheight_z_mm - 0.5:
                if logger:
                    logger.info(
                        "[TableA StopLift] skipped: already at/above pre-height "
                        "(actual Z=%.2f, pre_z=%.2f).",
                        actual_z, preheight_z_mm,
                    )
                return False
        elif last_xy is not None and len(last_xy) >= 2:
            lift_x, lift_y = float(last_xy[0]), float(last_xy[1])
        else:
            if logger:
                logger.info("[TableA StopLift] skipped: no actual XY and no fallback XY.")
            return False

        lift_pose = [lift_x, lift_y, float(preheight_z_mm), 0.0, 0.0, 0.0]
        if logger:
            logger.info(
                "[TableA StopLift] retracting tool straight up to pre-height %.2f mm with no force.",
                preheight_z_mm,
            )

        # communicate() and its wait loop both bail while the stop flag is latched, so a
        # MoveL issued now would be a no-op. The lift IS the intended stop action, so we
        # clear the flag only for this single retract move and re-latch it afterward, so the
        # caller's "stopped" semantics are preserved.
        clear_stop()
        try:
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
        finally:
            request_stop()
        return True
    except Exception as exc:
        if logger:
            logger.warning("[TableA StopLift] lift failed (continuing): %s", exc)
        return False
