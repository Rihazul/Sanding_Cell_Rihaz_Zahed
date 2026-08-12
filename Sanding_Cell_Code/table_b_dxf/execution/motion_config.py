from __future__ import annotations

from typing import Any


TABLE_B_SAFE_TRAVEL_Z_MM = -50.0
TABLE_B_BOTTOM_SAFE_TRAVEL_Y_MM = 100.0
TABLE_B_PREHEIGHT_Z_MM = -17.0
# How far (mm) to descend FAST (regular robot speed, no force control) from the
# pre-height TOWARD the surface before starting the slow force search. On Table B
# "toward the surface" is the +Z direction of the pose frame, so the fast move
# targets (preheight_z + this). This closes the large, variable air gap quickly so
# the slow 7 mm/s force search only ever crawls the last few mm to contact — making
# contact time consistent instead of randomly long. Must stay safely ABOVE the real
# surface (never descend into it). Set to 0 to disable (old all-slow behavior).
TABLE_B_FORCE_FAST_APPROACH_MM = 6.0
# How far (mm) BELOW the surface to command the sanding-point Z during force control.
# The sanding MoveL points otherwise command Z = preheight (-17), which sits ABOVE the
# real surface — so the position setpoint pulls the tool UP while force control fights
# to press it DOWN. At slow XY speed the force loop wins and pressure is constant; at
# fast XY speed the surface height changes faster than the (bandwidth-limited) Z force
# loop can correct, the up-pulling setpoint wins, and the tool hovers off the surface.
# By commanding Z past the surface (preheight + this overshoot, since +Z is toward the
# surface on Table B), the trajectory pushes INTO the surface and force control simply
# CAPS the pressure at the force goal — the tool physically cannot hover at any speed.
# Must exceed the deepest surface dip so the setpoint always lands below the real
# surface. Force control still limits how hard it presses. Set to 0 for old behavior.
TABLE_B_FORCE_SANDING_OVERSHOOT_MM = 8.0
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
    if not isinstance(config, dict):
        return {}
    value = config.get("tableBDxfMotion")
    if isinstance(value, dict):
        return value
    # Most Table B DXF motion settings currently live under the existing
    # "door" section in config.yaml. Keep supporting that layout so motion
    # constants are not silently ignored.
    door_value = config.get("door")
    return door_value if isinstance(door_value, dict) else {}


def _float_override(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(_motion_overrides(config).get(key, default))
    except (TypeError, ValueError):
        return default


def preheight_z_mm(config: dict[str, Any]) -> float:
    return _float_override(config, "preheightZMm", TABLE_B_PREHEIGHT_Z_MM)


def safe_travel_z_mm(config: dict[str, Any]) -> float:
    return _float_override(config, "safeTravelZMm", TABLE_B_SAFE_TRAVEL_Z_MM)


def bottom_safe_travel_y_mm(config: dict[str, Any]) -> float:
    return _float_override(config, "bottomSafeTravelYMm", TABLE_B_BOTTOM_SAFE_TRAVEL_Y_MM)


def force_fast_approach_mm(config: dict[str, Any]) -> float:
    # Clamp to >= 0; a negative value would move the tool away from the surface.
    return max(0.0, _float_override(config, "forceFastApproachMm", TABLE_B_FORCE_FAST_APPROACH_MM))


def force_sanding_overshoot_mm(config: dict[str, Any]) -> float:
    # Clamp to >= 0; the sanding Z setpoint is pushed toward/into the surface by this much.
    return max(0.0, _float_override(config, "forceSandingOvershootMm", TABLE_B_FORCE_SANDING_OVERSHOOT_MM))




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

