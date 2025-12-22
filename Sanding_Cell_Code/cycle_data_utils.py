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


def get_spiral_settings(cycle_data: Mapping[str, Any]) -> SpiralSettings:
    cfg = cycle_data.get("spiralSettings") or {}
    return SpiralSettings(
        enabled=_to_bool(cfg.get("enabled"), False),
        speed_percent=max(0, min(100, _to_int(cfg.get("speedPercent"), 100))),
        radius_mm=_to_float(cfg.get("radiusMm"), 0.0),
        linear_speed_mm_s=_to_float(cfg.get("linearSpeedMmS"), 0.0),
    )


def _extract_task_fields(task_cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """Extracts known fields for a TableA/TableB task section."""
    return {
        "cycle": _to_int(task_cfg.get("cycle"), 0),
        "force": _to_int(task_cfg.get("force"), 0),
        # Pocket ZigZag options (safe defaults)
        "verticalSpiral": _to_bool(task_cfg.get("verticalSpiral"), False),
        "horizontalSpiral": _to_bool(task_cfg.get("horizontalSpiral"), False),
        "edgeCoverage": _to_bool(task_cfg.get("edgeCoverage"), False),
    }


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
        for door_entry in doors_list:
            if not isinstance(door_entry, Mapping):
                continue
            door_num = _to_int(door_entry.get("doorNumber"), 0)
            if door_num <= 0:
                continue
            tasks = door_entry.get("tasks")
            if not isinstance(tasks, Mapping):
                continue
            task_cfg = tasks.get(task_key)
            if not isinstance(task_cfg, Mapping):
                continue
            by_door[door_num] = _extract_task_fields(task_cfg)
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
        by_door[door_num] = dict(shared)

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
