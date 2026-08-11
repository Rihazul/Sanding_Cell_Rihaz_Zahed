from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Tuple


def _to_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return default


@dataclass(frozen=True)
class SpiralSettings:
    enabled: bool
    speed_percent: int
    radius_mm: float
    linear_speed_mm_s: float
    turns: int
    orientation: str
    movement: str


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _map_linear_speed_to_turns(linear_speed_mm_s: float) -> int:
    """
    Map linear speed (100–300 mm/s) to turns (20 down to 10).
    Higher speed => fewer turns.
    """
    speed = _clamp(linear_speed_mm_s, 100.0, 300.0)
    # 100 -> 20, 300 -> 10 (linear interpolation)
    turns = 20 - (speed - 100.0) * (10.0 / 200.0)
    return int(round(_clamp(turns, 10.0, 20.0)))


def get_spiral_settings(cycle_data: Mapping[str, Any]) -> SpiralSettings:
    cfg = cycle_data.get("spiralSettings") or {}
    radius_mm = _clamp(_to_float(cfg.get("radiusMm"), 12.0), 10.0, 15.0)
    linear_speed_mm_s = _clamp(_to_float(cfg.get("linearSpeedMmS"), 150.0), 100.0, 300.0)
    turns = _to_int(cfg.get("turns"), _map_linear_speed_to_turns(linear_speed_mm_s))
    turns = _clamp(float(turns), 10.0, 20.0)
    orientation = str(cfg.get("orientation") or "vertical").strip().lower()
    movement = str(cfg.get("movement") or "zigzag").lower()
    if movement not in {"zigzag", "rect", "rectangular_spiral"}:
        movement = "zigzag"
    if orientation not in {"vertical", "horizontal", "both"}:
        orientation = "vertical"
    return SpiralSettings(
        enabled=_to_bool(cfg.get("enabled"), False),
        speed_percent=max(0, min(100, _to_int(cfg.get("speedPercent"), 100))),
        radius_mm=radius_mm,
        linear_speed_mm_s=linear_speed_mm_s,
        turns=int(turns),
        orientation=orientation,
        movement=movement,
    )


def _extract_task_fields(task_cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """Extracts known fields for a TableA/TableB task section."""
    vertical_spiral = _to_bool(task_cfg.get("verticalSpiral"), False)
    horizontal_spiral = _to_bool(task_cfg.get("horizontalSpiral"), False)
    orientation = str(task_cfg.get("orientation") or "").strip().lower()
    if orientation not in {"vertical", "horizontal", "both"}:
        if vertical_spiral and horizontal_spiral:
            orientation = "both"
        elif horizontal_spiral:
            orientation = "horizontal"
        else:
            orientation = "vertical"
    edge_flag = _to_bool(task_cfg.get("edge"), _to_bool(task_cfg.get("edgeCoverage"), False))
    rectangular_spiral = _to_bool(task_cfg.get("rectangularSpiral"), False)
    movement = str(task_cfg.get("movement") or "").strip().lower()
    if rectangular_spiral:
        movement = "rectangular_spiral"
    elif movement not in {"zigzag", "rect", "rectangular_spiral"}:
        movement = "zigzag"
    return {
        "cycle": _to_int(task_cfg.get("cycle"), 0),
        "force": _to_int(task_cfg.get("force"), 0),
        # Pocket ZigZag options (safe defaults)
        "verticalSpiral": vertical_spiral,
        "horizontalSpiral": horizontal_spiral,
        "rectangularSpiral": rectangular_spiral,
        "edgeCoverage": edge_flag,
        "edge": edge_flag,
        "orientation": orientation,
        "movement": movement,
    }


def _normalize_tablea_task_fields(task_key: str, fields: Dict[str, Any]) -> Dict[str, Any]:
    """Enforce Table A Pocket ZigZag multi-cycle orientation behavior."""
    if (
        task_key == "pocketzigzag"
        and _to_int(fields.get("cycle"), 0) > 1
        and str(fields.get("movement") or "zigzag") != "rectangular_spiral"
    ):
        fields["verticalSpiral"] = True
        fields["horizontalSpiral"] = True
        fields["orientation"] = "both"
    return fields


def get_tableA_task_by_door(tableA_cfg: Mapping[str, Any], task_key: str) -> Dict[int, Dict[str, Any]]:
    """Return per-door config for a TableA task.

    Supports both formats:
    - New door-based: TableA.doors = [{doorNumber, tasks:{...}}]
    - Legacy task-based: TableA[task_key].doors = [1,2,...] with shared cycle/force

    Returns: { doorNumber: {cycle, force, verticalSpiral, horizontalSpiral, edgeCoverage} }
    """

    by_door: Dict[int, Dict[str, Any]] = {}

    # New format
    doors_list = tableA_cfg.get("doors")
    if isinstance(doors_list, list):
        shared_task_cfg = tableA_cfg.get(task_key) if isinstance(tableA_cfg.get(task_key), Mapping) else {}
        shared_fields = _extract_task_fields(shared_task_cfg) if shared_task_cfg else {}
        for door_entry in doors_list:
            if not isinstance(door_entry, Mapping):
                continue
            door_num = _to_int(door_entry.get("doorNumber"), 0)
            if door_num <= 0:
                continue
            tasks = door_entry.get("tasks")
            # Some payloads embed tasks directly on the door entry (no "tasks" wrapper)
            if not isinstance(tasks, Mapping):
                tasks = door_entry if isinstance(door_entry, Mapping) else {}
            task_cfg = tasks.get(task_key)
            if not isinstance(task_cfg, Mapping):
                continue
            fields = _extract_task_fields(task_cfg)

            # Pocket ZigZag resilience:
            # if a per-door payload omits orientation/spiral flags, inherit from shared task config
            # instead of defaulting back to vertical.
            if task_key == "pocketzigzag" and shared_fields:
                has_orientation = bool(str(task_cfg.get("orientation") or "").strip())
                has_vertical = "verticalSpiral" in task_cfg
                has_horizontal = "horizontalSpiral" in task_cfg
                if not has_orientation and not has_vertical and not has_horizontal:
                    fields["orientation"] = str(shared_fields.get("orientation") or fields.get("orientation") or "vertical")
                    fields["verticalSpiral"] = _to_bool(shared_fields.get("verticalSpiral"), fields.get("verticalSpiral", False))
                    fields["horizontalSpiral"] = _to_bool(shared_fields.get("horizontalSpiral"), fields.get("horizontalSpiral", False))
                    fields["rectangularSpiral"] = _to_bool(shared_fields.get("rectangularSpiral"), fields.get("rectangularSpiral", False))
                    fields["edgeCoverage"] = _to_bool(shared_fields.get("edgeCoverage"), fields.get("edgeCoverage", False))
                    fields["edge"] = fields["edgeCoverage"]
                    fields["movement"] = str(shared_fields.get("movement") or fields.get("movement") or "zigzag")

            by_door[door_num] = _normalize_tablea_task_fields(task_key, fields)
        return by_door

    # Legacy format
    task_cfg = tableA_cfg.get(task_key)
    if not isinstance(task_cfg, Mapping):
        return by_door

    doors = task_cfg.get("doors")
    if not isinstance(doors, list):
        doors = []

    shared = _extract_task_fields(task_cfg)
    for door in doors:
        door_num = _to_int(door, 0)
        if door_num <= 0:
            continue
        by_door[door_num] = _normalize_tablea_task_fields(task_key, dict(shared))

    return by_door


def doors_with_cycles(by_door: Mapping[int, Mapping[str, Any]]) -> List[int]:
    """Filter door numbers to only those with cycle > 0."""
    out: List[int] = []
    for door_num, cfg in by_door.items():
        if _to_int(cfg.get("cycle"), 0) > 0:
            out.append(int(door_num))
    return sorted(set(out))


def any_cycles(by_door: Mapping[int, Mapping[str, Any]]) -> bool:
    return any(_to_int(cfg.get("cycle"), 0) > 0 for cfg in by_door.values())


def overlap_mm_to_step(
    overlap_mm: Any,
    tool_diameter_mm: float = 140.0,
    max_overlap_mm: float = 100.0,
    min_step_mm: float = 1.0,
) -> int:
    overlap = _clamp(_to_float(overlap_mm, 0.0), 0.0, max_overlap_mm)
    step = tool_diameter_mm - overlap
    return int(round(max(min_step_mm, step)))


def get_inverse_overlap_step(
    cycle_data: Mapping[str, Any],
    key: str = "inverseOverlapping",
    tool_diameter_mm: float = 140.0,
    max_overlap_mm: float = 100.0,
    min_step_mm: float = 1.0,
) -> int:
    """
    Convert UI raw overlap (mm) into backend step (mm).
    Backward-compatible: if stored value looks like an old step value
    (> max_overlap and <= tool_diameter), keep it as-is.
    """
    raw = _to_float(cycle_data.get(key), 0.0)
    if raw > max_overlap_mm and raw <= tool_diameter_mm:
        return int(round(max(min_step_mm, raw)))
    return overlap_mm_to_step(
        raw,
        tool_diameter_mm=tool_diameter_mm,
        max_overlap_mm=max_overlap_mm,
        min_step_mm=min_step_mm,
    )
