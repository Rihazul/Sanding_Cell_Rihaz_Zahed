from __future__ import annotations

from typing import Any

from Server_Better_V2 import (
    stop_requested,
    turn_vibration_off,
)


class TableBDxfRobotExecutionError(RuntimeError):
    """Raised when a Table B DXF robot path cannot be executed safely."""


class TableBDxfRobotStopRequested(TableBDxfRobotExecutionError):
    """Raised to abort the whole Table B DXF run after operator Stop."""


def log(config: dict[str, Any], message: str, *args: Any) -> None:
    logger = config.get("logger") if isinstance(config, dict) else None
    if logger:
        logger.info(message, *args)


def stop_active_motion(cps: Any, config: dict[str, Any], reason: str) -> None:
    log(config, "[TableB DXF Robot] stop requested: %s", reason)
    try:
        cps.HRIF_GrpStop(0, 0)
    except Exception:
        pass
    try:
        result = []
        cps.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
    except Exception:
        pass
    try:
        cps.HRIF_SetForceControlState(0, 0, 0)
    except Exception:
        pass
    try:
        turn_vibration_off(cps)
    except Exception:
        pass
    try:
        cps.HRIF_SetBoxDO(0, 7, 0)  # tool spin off
    except Exception:
        pass
    try:
        cps.HRIF_SetBoxDO(0, 3, 0)  # laser off
    except Exception:
        pass


def raise_if_stop_requested(cps: Any, config: dict[str, Any], reason: str) -> None:
    if stop_requested():
        stop_active_motion(cps, config, reason)
        raise TableBDxfRobotStopRequested("Table B DXF operation stopped by user.")