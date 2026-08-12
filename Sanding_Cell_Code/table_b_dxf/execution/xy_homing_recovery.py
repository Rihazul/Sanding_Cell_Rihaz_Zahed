from __future__ import annotations

from typing import Any

from Server_Better_V2 import _get_tool_in_hand_for_drop, communicate

from .motion_config import robot_speed, safe_travel_z_mm, table_b_ucs, tcp_for_physical_tool


NEGATIVE_X_PULLBACK_TOLERANCE_MM = 1.0


def _log(config: dict[str, Any], message: str, *args: Any) -> None:
    logger = config.get("logger") if isinstance(config, dict) else None
    if logger:
        logger.info(message, *args)


def _warn(config: dict[str, Any], message: str, *args: Any) -> None:
    logger = config.get("logger") if isinstance(config, dict) else None
    if logger:
        logger.warning(message, *args)


def _read_tool_coord_pose(cps: Any, tcp: str, ucs: str) -> list[float] | None:
    result: list[Any] = []
    try:
        ret = cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result)
    except Exception:
        ret = -1
    if ret != 0 or len(result) < 12:
        return None
    try:
        return [float(result[i]) for i in range(6, 12)]
    except (TypeError, ValueError):
        return None


def recover_xy_tool_before_homing_if_needed(cps: Any, config: dict[str, Any]) -> bool:
    """Pull Table B Tool 1/3/4 out of negative-X before normal homing.

    After a stop, the XY force runner retracts Tool 1/3/4 to the safe travel Z.
    If the TCP is still on the top/negative-X side of Plane 2, the normal default
    homing move can choose a twisted joint solution. Moving only local X back to
    zero in the same Table B TCP/UCS frame gives homing a safer starting pose.
    """
    try:
        tool = _get_tool_in_hand_for_drop(cps)
    except Exception as exc:
        _warn(config, "[TableB DXF XY Recovery] skipped: mounted tool read failed: %s", exc)
        return False

    if tool not in (1, 3, 4):
        return False

    try:
        tcp = tcp_for_physical_tool(config, int(tool))
        ucs = table_b_ucs(config)
    except Exception as exc:
        _warn(config, "[TableB DXF XY Recovery] skipped: config error for tool %s: %s", tool, exc)
        return False

    pose = _read_tool_coord_pose(cps, tcp, ucs)
    if pose is None:
        _warn(
            config,
            "[TableB DXF XY Recovery] skipped: unable to read current pose for tool=%s tcp=%s ucs=%s",
            tool,
            tcp,
            ucs,
        )
        return False

    x, y, z, rx, ry, rz = pose
    safe_z = safe_travel_z_mm(config)
    speed = robot_speed(config)
    moved = False

    # If the stop cleanup did not reach the safe height for any reason, lift
    # first at the current XY before pulling X toward zero.
    if z > safe_z + 0.5:
        lift_pose = [x, y, safe_z, rx, ry, rz]
        _log(
            config,
            "[TableB DXF XY Recovery] lifting tool=%s before homing: current=(%.1f, %.1f, %.1f) target=%s",
            tool,
            x,
            y,
            z,
            lift_pose,
        )
        result = communicate(
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
        if result is None:
            _warn(config, "[TableB DXF XY Recovery] safe-Z lift failed for tool=%s; continuing to homing.", tool)
            return moved
        z = safe_z
        moved = True

    if x >= -NEGATIVE_X_PULLBACK_TOLERANCE_MM:
        return moved

    pullback_pose = [0.0, y, safe_z, rx, ry, rz]
    _log(
        config,
        "[TableB DXF XY Recovery] pulling negative-X pose to X=0 before homing: tool=%s current=(%.1f, %.1f, %.1f) target=%s",
        tool,
        x,
        y,
        z,
        pullback_pose,
    )
    result = communicate(
        cps=cps,
        config=config,
        point=pullback_pose,
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    if result is None:
        _warn(config, "[TableB DXF XY Recovery] negative-X pullback failed for tool=%s; continuing to homing.", tool)
        return moved
    return True
