"""Shared Table A XY-plane safety helpers for Tool 1/3/4 force-locked sanding.

When the operator presses Stop mid-sanding, Table A operations press the tool DOWN into
the surface under Z- force control (putForceZminus). This helper is called from each
force-locked sanding helper's `finally` on stop to do the CNC-style recovery:

  1. force control OFF + vibration OFF  (so the tool stops pressing)
  2. lift the tool STRAIGHT UP to the Table A XY safe height (+Z), with NO force, so it
     releases the surface immediately and leaves no drag mark.

From the safe height the operator can re-send the task or run homing. Table A presses Z-,
so "lift" means going to a MORE-positive Z.

Tool 2 is intentionally separate because it sands the door thickness with X/Y force.
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

TABLE_A_XY_SAFE_Z_MM = 50.0
TABLE_A_XY_SAFE_Z_TOL_MM = 0.5


def _logger(config: dict[str, Any]):
    return config.get("logger") if isinstance(config, dict) else None


def _read_actual_pose(cps: Any, tcp: str, ucs: str) -> list[float] | None:
    result: list[Any] = []
    try:
        if cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result) == 0 and len(result) >= 12:
            return [float(result[i]) for i in range(6, 12)]
    except Exception:
        return None
    return None


def _fallback_pose_from_xy(
    last_xy: tuple[float, float] | list[float] | None,
    target_z_mm: float,
) -> list[float] | None:
    if last_xy is None or len(last_xy) < 2:
        return None
    return [float(last_xy[0]), float(last_xy[1]), float(target_z_mm), 0.0, 0.0, 0.0]


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
    """On STOP, release force and lift Tool 1/3/4 straight up to Table A safe Z.

    `preheight_z_mm` is kept for backwards-compatible call sites, but the effective target
    is never below TABLE_A_XY_SAFE_Z_MM. Reads the actual pose in the same TCP/UCS as the
    operation so the retract is straight up in the active sanding coordinate frame.

    Returns True if a lift move was issued. Best-effort: never raises.
    """
    logger = _logger(config)
    target_z_mm = max(float(preheight_z_mm), TABLE_A_XY_SAFE_Z_MM)

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
        pose = _read_actual_pose(cps, tcp, ucs)
        if pose is not None:
            actual_z = pose[2]
            # Table A presses Z-, so pre-height is above the surface (more positive Z). If
            # already at/above it, nothing to lift.
            if actual_z >= target_z_mm - TABLE_A_XY_SAFE_Z_TOL_MM:
                if logger:
                    logger.info(
                        "[TableA StopLift] skipped: already at/above safe Z "
                        "(actual Z=%.2f, safe_z=%.2f).",
                        actual_z, target_z_mm,
                    )
                return False
        else:
            pose = _fallback_pose_from_xy(last_xy, target_z_mm)
            if pose is None:
                if logger:
                    logger.info("[TableA StopLift] skipped: no actual pose and no fallback XY.")
                return False

        lift_pose = list(pose)
        lift_pose[2] = target_z_mm
        if logger:
            logger.info(
                "[TableA StopLift] retracting Tool 1/3/4 straight up to safe Z %.2f mm with no force.",
                target_z_mm,
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


def stop_lift_to_safe_z_tableA(
    cps: Any,
    config: dict[str, Any],
    *,
    tcp: str,
    ucs: str,
    robot_speed: float,
    last_xy: tuple[float, float] | list[float] | None = None,
) -> bool:
    """Explicit Table A Tool 1/3/4 stop lift API."""
    return stop_lift_to_preheight_tableA(
        cps,
        config,
        tcp=tcp,
        ucs=ucs,
        preheight_z_mm=TABLE_A_XY_SAFE_Z_MM,
        robot_speed=robot_speed,
        last_xy=last_xy,
    )


def move_tablea_xy_safe_then_prepoint(
    cps: Any,
    config: dict[str, Any],
    *,
    point: list[float] | tuple[float, ...],
    tcp: str,
    ucs: str,
    robot_speed: float,
    safe_z_mm: float = TABLE_A_XY_SAFE_Z_MM,
    velocity_profile: str = "robotspeed",
) -> None:
    """Travel at safe Z before descending to a Tool 1/3/4 Table A XY pre-force point."""
    if len(point) < 6:
        raise ValueError("Table A XY prepoint must contain [X,Y,Z,RX,RY,RZ].")

    target_pose = [float(v) for v in point[:6]]
    safe_z = float(safe_z_mm)
    logger = _logger(config)

    actual = _read_actual_pose(cps, tcp, ucs)
    if actual is not None and actual[2] < safe_z - TABLE_A_XY_SAFE_Z_TOL_MM:
        lift_pose = list(actual)
        lift_pose[2] = safe_z
        if logger:
            logger.info("[TableA SafeTravel] lifting current pose to Z=%.2f before XY travel.", safe_z)
        communicate(
            cps=cps,
            config=config,
            point=lift_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile=velocity_profile,
            wait=True,
        )

    safe_target_pose = list(target_pose)
    safe_target_pose[2] = safe_z
    if logger:
        logger.info("[TableA SafeTravel] travelling at Z=%.2f to target XY before prepoint.", safe_z)
    communicate(
        cps=cps,
        config=config,
        point=safe_target_pose,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=robot_speed,
        velocity_profile=velocity_profile,
        wait=True,
    )

    if abs(target_pose[2] - safe_z) > TABLE_A_XY_SAFE_Z_TOL_MM:
        if logger:
            logger.info("[TableA SafeTravel] descending from safe Z to prepoint Z=%.2f.", target_pose[2])
        communicate(
            cps=cps,
            config=config,
            point=target_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile=velocity_profile,
            wait=True,
        )
