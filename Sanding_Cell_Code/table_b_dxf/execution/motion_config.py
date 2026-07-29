from __future__ import annotations

from typing import Any


TABLE_B_PREHEIGHT_Z_MM = -17.0
TABLE_B_ORIENTATION = [180.0, 0.0, 0.0]
TABLE_B_UCS_KEY = "ucsTable2"

TOOL_TCP_KEYS: dict[int, str] = {
    1: "tcptool1plane2",
    3: "tcptool3plane2",
    4: "tcptool4plane2",
}


class TableBDxfMotionConfigError(RuntimeError):
    """Raised when Table B DXF robot motion config is incomplete."""


def _motion_overrides(config: dict[str, Any]) -> dict[str, Any]:
    value = config.get("tableBDxfMotion", {}) if isinstance(config, dict) else {}
    return value if isinstance(value, dict) else {}


def _float_override(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(_motion_overrides(config).get(key, default))
    except (TypeError, ValueError):
        return default


def preheight_z_mm(config: dict[str, Any]) -> float:
    return _float_override(config, "preheightZMm", TABLE_B_PREHEIGHT_Z_MM)




def orientation(config: dict[str, Any]) -> list[float]:
    value = _motion_overrides(config).get("orientation")
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            pass
    return list(TABLE_B_ORIENTATION)


def tcp_for_physical_tool(config: dict[str, Any], physical_tool: int) -> str:
    key = TOOL_TCP_KEYS.get(int(physical_tool))
    if not key:
        raise TableBDxfMotionConfigError(
            f"Table B DXF robot execution is not configured for physical tool {physical_tool}."
        )
    try:
        return str(config["coords"][key])
    except KeyError as error:
        raise TableBDxfMotionConfigError(
            f"Missing Table B TCP config key: coords.{key}"
        ) from error


def table_b_ucs(config: dict[str, Any]) -> str:
    try:
        return str(config["coords"][TABLE_B_UCS_KEY])
    except KeyError as error:
        raise TableBDxfMotionConfigError(
            f"Missing Table B UCS config key: coords.{TABLE_B_UCS_KEY}"
        ) from error


def _required_ui_speed(config: dict[str, Any], key: str, label: str) -> float:
    try:
        value = config["UI"][key]
    except KeyError as error:
        raise TableBDxfMotionConfigError(
            f"Missing UI-selected {label} speed: config.UI.{key}"
        ) from error
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise TableBDxfMotionConfigError(
            f"Invalid UI-selected {label} speed config.UI.{key}={value!r}"
        ) from error


def robot_speed(config: dict[str, Any]) -> float:
    return _required_ui_speed(config, "robotSpeed", "robot")


def sanding_speed(config: dict[str, Any]) -> float:
    return _required_ui_speed(config, "sandSpeed", "sanding")

