from __future__ import annotations

"""Pocket zigzag cycle expansion for Table B DXF execution.

This module owns the runtime-only expansion for Tool 4 pocket zigzag when the
operator selects cycle > 1. It rebuilds each reach window as a continuous
vertical/horizontal alternating path while preserving the approved DXF geometry
and 7th-axis reach constraints.
"""

import json
import logging
import math
import re
from pathlib import Path
from typing import Any

from ..tool_reach import attach_station_plan, station_window_for_path

logger = logging.getLogger(__name__)

TOOL4_POCKET_PASS_WIDTH_MM = 144.0
CYCLE_WINDOW_BOUNDARY_EPS_MM = 0.001
_DEBUG_PATH = Path(__file__).resolve().parents[1] / "debug" / "last_pocket_zigzag_cycles.json"


def _float_or_none(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _xy_points(points: Any) -> list[list[float]]:
    out: list[list[float]] = []
    for point in points or []:
        try:
            out.append([float(point[0]), float(point[1])])
        except (TypeError, ValueError, IndexError):
            continue
    return out


def _normalize_recipe_entry(entry: dict[str, Any] | None) -> dict[str, int]:
    if not isinstance(entry, dict):
        return {"force": 0, "cycle": 0}
    try:
        force = int(entry.get("force") or 0)
    except (TypeError, ValueError):
        force = 0
    try:
        cycle = int(entry.get("cycle") or 0)
    except (TypeError, ValueError):
        cycle = 0
    return {"force": force, "cycle": cycle}


def _point_distance(a: list[float] | None, b: list[float] | None) -> float:
    if not a or not b or len(a) < 2 or len(b) < 2:
        return float("inf")
    return ((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2) ** 0.5


def _ordered_pattern_paths(paths: list[dict[str, Any]], current_end: list[float] | None) -> list[dict[str, Any]]:
    """Order/reverse one pattern so the next path starts near the previous end.

    This does not invent connector moves. It only chooses the safest orientation of
    already reach-planned paths so cycle N+1 can continue from where cycle N ended
    whenever the endpoints make that possible.
    """
    remaining = [dict(path) for path in paths]
    ordered: list[dict[str, Any]] = []
    cursor = current_end

    while remaining:
        if cursor is None:
            path = remaining.pop(0)
            pts = _xy_points(path.get("points") or [])
            path["points"] = pts
            ordered.append(path)
            cursor = pts[-1] if pts else cursor
            continue

        best_index = 0
        best_reverse = False
        best_distance = float("inf")
        for index, candidate in enumerate(remaining):
            pts = _xy_points(candidate.get("points") or [])
            if len(pts) < 2:
                continue
            forward_distance = _point_distance(cursor, pts[0])
            reverse_distance = _point_distance(cursor, pts[-1])
            if forward_distance < best_distance:
                best_index = index
                best_reverse = False
                best_distance = forward_distance
            if reverse_distance < best_distance:
                best_index = index
                best_reverse = True
                best_distance = reverse_distance

        path = remaining.pop(best_index)
        pts = _xy_points(path.get("points") or [])
        if best_reverse:
            pts = list(reversed(pts))
        path["points"] = pts
        ordered.append(path)
        cursor = pts[-1] if pts else cursor

    return ordered


def _rebuild_serpentine_from_corner(
    window_points: list[list[float]],
    orientation: str,
    start_corner: list[float] | None,
) -> list[list[float]] | None:
    """Rebuild a window's serpentine so it STARTS at the corner nearest start_corner and
    sweeps into the window from there — giving true continuous alternation between cycles.

    A serpentine has two independent directions: the sweep axis (long passes) and the step
    axis (offset between passes). Reversing the flattened point list only flips both together,
    so it cannot express "continue from the exact end corner and sweep the right way". Here we
    recover the lane grid from the points and re-emit it from the chosen start corner:

      - vertical   : passes run along Y, step across X.
      - horizontal : passes run along X, step across Y.

    Continuity rule (matches operator spec):
      * first lane = the step-axis extreme NEAREST the previous end's step coordinate;
      * first sweep goes from the previous end's sweep coordinate toward the opposite extreme;
      * subsequent lanes step AWAY from the start corner (serpentine).

    So e.g. a vertical pass ending on the LEFT continues as a horizontal pass sweeping RIGHT
    from that left corner; ending on the RIGHT continues sweeping LEFT; a horizontal pass
    ending TOP-left continues as a vertical pass going DOWN from top-left; ending BOTTOM-left
    continues going UP. Returns None if the grid can't be recovered (caller falls back).
    """
    pts = _xy_points(window_points)
    if len(pts) < 2:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(ys), max(ys)

    if orientation == "vertical":
        sweep_lo, sweep_hi = y_lo, y_hi          # passes run along Y
        lane_values = sorted({round(p[0], 3) for p in pts})  # lanes at distinct X
        step_lo, step_hi = x_lo, x_hi
    else:  # horizontal: passes run along X
        sweep_lo, sweep_hi = x_lo, x_hi
        lane_values = sorted({round(p[1], 3) for p in pts})  # lanes at distinct Y
        step_lo, step_hi = y_lo, y_hi
    if len(lane_values) < 1:
        return None

    # Decide the start corner from the previous end. With no previous end we keep the default
    # bottom-right origin (step at min, sweep at min) so the first cycle is unchanged.
    have_start = start_corner is not None and len(start_corner) >= 2
    ex = float(start_corner[0]) if have_start else 0.0
    ey = float(start_corner[1]) if have_start else 0.0

    if orientation == "vertical":
        step_coord, sweep_coord = ex, ey  # step across X, sweep along Y
    else:
        step_coord, sweep_coord = ey, ex  # step across Y, sweep along X

    step_start_near_hi = have_start and abs(step_coord - step_hi) < abs(step_coord - step_lo)
    sweep_start_at_hi = have_start and abs(sweep_coord - sweep_hi) < abs(sweep_coord - sweep_lo)

    # Lanes ordered so the FIRST lane is nearest the start corner's step coordinate.
    ordered_lanes = list(reversed(lane_values)) if step_start_near_hi else lane_values

    out: list[list[float]] = []
    sweep_at_hi = sweep_start_at_hi  # first pass starts at this sweep end
    for lane in ordered_lanes:
        a = sweep_hi if sweep_at_hi else sweep_lo
        b = sweep_lo if sweep_at_hi else sweep_hi
        if orientation == "vertical":
            p0, p1 = [lane, a], [lane, b]
        else:
            p0, p1 = [a, lane], [b, lane]
        # Avoid duplicating the shared endpoint between consecutive lanes.
        if out and _point_distance(out[-1], p0) <= 1e-6:
            out.append(p1)
        else:
            out.extend([p0, p1])
        sweep_at_hi = not sweep_at_hi  # serpentine
    return out if len(out) >= 2 else None


def _pocket_zigzag_window_key(path: dict[str, Any]) -> str:
    explicit = path.get("cycle_window_id")
    if explicit:
        base_key = str(explicit)
        try:
            split_count = int(path.get("reach_split_count") or 1)
        except (TypeError, ValueError):
            split_count = 1
        axis = _float_or_none(path.get("axis7_position_mm"))
        if split_count > 1 and axis is not None:
            return f"{base_key}@j7:{axis:.1f}"
        if split_count > 1:
            return f"{base_key}@split:{path.get('reach_split_index')}"
        return base_key
    base = str(path.get("split_from_path_id") or path.get("path_id") or "")
    # Triangular pockets use Tool 4 spiral fill, not vertical/horizontal rows.
    match = re.search(r"(.+?_tool4_tri)(?:_|$)", base)
    if match:
        return match.group(1)
    # Rectangular-spiral pockets: match BEFORE the generic _tool4 rule so the "_rect" marker
    # survives in the window key. A wide pocket is divided into per-reach-window sections
    # (`_tool4_rect_sec{n}`), each an independent spiral window; capture the section suffix so
    # sections group separately. Narrow pockets are `_tool4_rect` (no section). The INWARD
    # variant is `_tool4_rect...` and the OUTWARD variant `_tool4_rectout...`; normalize `rectout`
    # -> `rect` so the in/out spirals of the SAME section map to ONE window key and stitch into a
    # single continuous force contact.
    match = re.search(r"(.+?_tool4_rect)(?:out)?((?:_sec\d+)?)(?:_|$)", base)
    if match:
        return match.group(1) + match.group(2)
    match = re.search(r"(.+?_tool4_sec\d+)", base)
    if match:
        return match.group(1)
    match = re.search(r"(.+?_tool4)(?:_|$)", base)
    if match:
        return match.group(1)
    return base


def _is_triangle_pocket_zigzag_window(window_key: str) -> bool:
    return "_tool4_tri" in str(window_key)


def _is_rect_spiral_pocket_zigzag_window(window_key: str) -> bool:
    return "_tool4_rect" in str(window_key)


def _group_pocket_zigzag_windows(paths: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for path in paths:
        key = _pocket_zigzag_window_key(path)
        groups.setdefault(key, []).append(dict(path))
    return groups


def _append_continuous_points(target: list[list[float]], points: list[list[float]]) -> None:
    if not points:
        return
    if not target:
        target.extend(points)
        return
    if _point_distance(target[-1], points[0]) <= 0.5:
        target.extend(points[1:])
    else:
        target.extend(points)


def _float_bounds(bounds: Any) -> dict[str, float] | None:
    if not isinstance(bounds, dict):
        return None
    try:
        x_min = float(bounds["x_min"])
        x_max = float(bounds["x_max"])
        y_min = float(bounds["y_min"])
        y_max = float(bounds["y_max"])
    except (KeyError, TypeError, ValueError):
        return None
    if x_max <= x_min or y_max <= y_min:
        return None
    return {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}


def _window_bounds_for_paths(paths: list[dict[str, Any]]) -> dict[str, float] | None:
    point_bounds = _point_bounds_for_paths(paths)
    for path in paths:
        bounds = _float_bounds(path.get("cycle_window_bounds"))
        if bounds is not None:
            if point_bounds is None:
                return bounds
            # Reach planning may split a large frontend window into smaller executable
            # pieces. Keep the real post-reach X/Y limits where they are informative,
            # but fall back to the original cycle window when a split is only a single
            # row/column. That produces a true rectangular execution window per J7 stop.
            x_span = point_bounds["x_max"] - point_bounds["x_min"]
            y_span = point_bounds["y_max"] - point_bounds["y_min"]
            return {
                "x_min": point_bounds["x_min"] if x_span > 1.0 else bounds["x_min"],
                "x_max": point_bounds["x_max"] if x_span > 1.0 else bounds["x_max"],
                "y_min": point_bounds["y_min"] if y_span > 1.0 else bounds["y_min"],
                "y_max": point_bounds["y_max"] if y_span > 1.0 else bounds["y_max"],
            }
    if point_bounds is not None:
        return point_bounds
    return None


def _point_bounds_for_paths(paths: list[dict[str, Any]]) -> dict[str, float] | None:
    points: list[list[float]] = []
    for path in paths:
        points.extend(_xy_points(path.get("points") or []))
    if not points:
        return None
    x_min = min(p[0] for p in points)
    x_max = max(p[0] for p in points)
    y_min = min(p[1] for p in points)
    y_max = max(p[1] for p in points)
    if x_max <= x_min or y_max <= y_min:
        return None
    return {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}


def _cycle_window_base_key(window_key: str) -> str:
    return str(window_key).split("@", 1)[0]


def _cycle_window_family_key(window_key: str) -> str:
    """Group reach-split sections that belong to the same physical pocket."""
    return re.sub(r"_sec\d+$", "", _cycle_window_base_key(window_key))


def _is_rectangular_fill_window(window_key: str) -> bool:
    key = str(window_key)
    return "_tool4" in key and "_tool4_tri" not in key and "_tool4_rect" not in key


def _paths_for_cycle_window_key(
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
    window_key: str,
) -> list[dict[str, Any]]:
    paths: list[dict[str, Any]] = []
    seen: set[str] = set()
    for orientation_groups in grouped.values():
        for path in orientation_groups.get(window_key) or []:
            path_id = str(path.get("path_id") or id(path))
            if path_id in seen:
                continue
            seen.add(path_id)
            paths.append(path)
    return paths


def _original_cycle_window_bounds(paths: list[dict[str, Any]]) -> dict[str, float] | None:
    for path in paths:
        bounds = _float_bounds(path.get("cycle_window_bounds"))
        if bounds is not None:
            return bounds
    return None


def _partition_cycle_window_bounds(
    window_order: list[str],
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
) -> dict[str, dict[str, float]]:
    """Return non-overlapping execution bounds for pocket-zigzag cycle windows.

    Reach planning can split a frontend cycle rectangle into uneven fragments. If those
    fragment extents are reused as sanding rectangles, the robot can sand a big area,
    then a tiny leftover, then repass part of that area after the next J7 move. The
    frontend cycle_window_bounds is the intended physical rectangle, so split that
    rectangle into clean X slices whenever several planner keys belong to the same
    frontend window.
    """
    by_base: dict[str, list[tuple[str, list[dict[str, Any]], dict[str, float]]]] = {}
    for window_key in window_order:
        paths = _paths_for_cycle_window_key(grouped, window_key)
        original_bounds = _original_cycle_window_bounds(paths) or _window_bounds_for_paths(paths)
        if original_bounds is None:
            continue
        by_base.setdefault(_cycle_window_base_key(window_key), []).append((window_key, paths, original_bounds))

    overrides: dict[str, dict[str, float]] = {}
    for entries in by_base.values():
        base_bounds = entries[0][2]
        if len(entries) == 1:
            overrides[entries[0][0]] = dict(base_bounds)
            continue

        def entry_center(entry: tuple[str, list[dict[str, Any]], dict[str, float]]) -> float:
            point_bounds = _point_bounds_for_paths(entry[1])
            bounds = point_bounds or entry[2]
            return (bounds["x_min"] + bounds["x_max"]) * 0.5

        ordered = sorted(entries, key=entry_center)
        width = (base_bounds["x_max"] - base_bounds["x_min"]) / len(ordered)
        for index, (window_key, _paths, _bounds) in enumerate(ordered):
            x_min = base_bounds["x_min"] + index * width
            x_max = base_bounds["x_max"] if index == len(ordered) - 1 else x_min + width
            overrides[window_key] = {
                "x_min": x_min,
                "x_max": x_max,
                "y_min": base_bounds["y_min"],
                "y_max": base_bounds["y_max"],
            }
    return _trim_adjacent_cycle_window_bounds(overrides, window_order)


def _rectangle_station_window(bounds: dict[str, float]) -> tuple[float, float] | None:
    """Return a legal J7 station interval for a full rectangular pocket window."""
    corners = [
        [bounds["x_min"], bounds["y_min"]],
        [bounds["x_min"], bounds["y_max"]],
        [bounds["x_max"], bounds["y_min"]],
        [bounds["x_max"], bounds["y_max"]],
    ]
    return station_window_for_path("tool_4", corners)


def _split_rect_bounds_by_tool4_reach(bounds: dict[str, float]) -> list[dict[str, float]]:
    """Split a pocket rectangle into even, non-overlapping Tool 4 reachable X windows."""
    x_min = bounds["x_min"]
    x_max = bounds["x_max"]
    y_min = bounds["y_min"]
    y_max = bounds["y_max"]
    if x_max <= x_min or y_max <= y_min:
        return []

    if _rectangle_station_window(bounds) is not None:
        return [dict(bounds)]

    low = x_min
    high = x_max
    best = x_min
    for _ in range(48):
        mid = (low + high) * 0.5
        candidate = {"x_min": x_min, "x_max": mid, "y_min": y_min, "y_max": y_max}
        if _rectangle_station_window(candidate) is not None:
            best = mid
            low = mid
        else:
            high = mid

    max_width = best - x_min
    if max_width <= 1.0:
        return [dict(bounds)]

    total_width = x_max - x_min
    section_count = max(1, math.ceil(total_width / max_width))
    section_width = total_width / section_count
    windows: list[dict[str, float]] = []
    for index in range(section_count):
        section_x_min = x_min + index * section_width
        section_x_max = x_max if index == section_count - 1 else x_min + (index + 1) * section_width
        if index:
            section_x_min += CYCLE_WINDOW_BOUNDARY_EPS_MM
        windows.append({"x_min": section_x_min, "x_max": section_x_max, "y_min": y_min, "y_max": y_max})
    return windows


def _trim_internal_rect_window_boundaries(
    windows: list[dict[str, float]],
    step_mm: float,
) -> list[dict[str, float]]:
    """Keep adjacent rectangular execution windows from reusing the same X edge."""
    if len(windows) <= 1:
        return windows

    trimmed: list[dict[str, float]] = []
    for index, bounds in enumerate(windows):
        out = dict(bounds)
        if index > 0:
            width = out["x_max"] - out["x_min"]
            inset = min(max(float(step_mm) * 0.5, 1.0), width * 0.33)
            if out["x_min"] + inset < out["x_max"] - 1.0:
                out["x_min"] += inset
        trimmed.append(out)
    return trimmed


def _collect_rectangular_pocket_bounds(
    window_order: list[str],
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
) -> list[tuple[str, dict[str, float], dict[str, Any]]]:
    """Union stored UI cycle bounds into one full rectangle per pocket."""
    ordered_families: list[str] = []
    family_bounds: dict[str, list[dict[str, float]]] = {}
    family_template: dict[str, dict[str, Any]] = {}

    for window_key in window_order:
        if not _is_rectangular_fill_window(window_key):
            continue
        family_key = _cycle_window_family_key(window_key)
        paths = _paths_for_cycle_window_key(grouped, window_key)
        bounds = _original_cycle_window_bounds(paths)
        if bounds is None:
            bounds = _window_bounds_for_paths(paths)
        if bounds is None:
            continue
        if family_key not in ordered_families:
            ordered_families.append(family_key)
        family_bounds.setdefault(family_key, []).append(bounds)
        if family_key not in family_template and paths:
            family_template[family_key] = dict(paths[0])

    rectangles: list[tuple[str, dict[str, float], dict[str, Any]]] = []
    for family_key in ordered_families:
        bounds_list = family_bounds.get(family_key) or []
        template = family_template.get(family_key)
        if not bounds_list or template is None:
            continue
        rectangle = {
            "x_min": min(bounds["x_min"] for bounds in bounds_list),
            "x_max": max(bounds["x_max"] for bounds in bounds_list),
            "y_min": min(bounds["y_min"] for bounds in bounds_list),
            "y_max": max(bounds["y_max"] for bounds in bounds_list),
        }
        rectangles.append((family_key, rectangle, template))
    return rectangles


def _build_rectangular_cycle_windows(
    window_order: list[str],
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
    step_mm: float,
) -> list[tuple[str, dict[str, float], dict[str, Any]]]:
    """Build clean backend execution windows for rectangular pocket multi-cycles."""
    out: list[tuple[str, dict[str, float], dict[str, Any]]] = []
    for family_key, rectangle, template in _collect_rectangular_pocket_bounds(window_order, grouped):
        splits = _trim_internal_rect_window_boundaries(_split_rect_bounds_by_tool4_reach(rectangle), step_mm)
        for index, bounds in enumerate(splits, start=1):
            out.append((f"{family_key}_exec{index}", bounds, template))
    return out


def _trim_adjacent_cycle_window_bounds(
    overrides: dict[str, dict[str, float]],
    window_order: list[str],
) -> dict[str, dict[str, float]]:
    """Remove repeated X coverage between neighboring pocket cycle windows.

    Adjacent reach windows can share the same X boundary. If both generated
    serpentine paths include that boundary lane, section 2 resands the end lane
    of section 1. This trims only the following window by an epsilon, keeping the
    viewer coordinates effectively unchanged while preventing duplicate runtime
    lanes.
    """
    groups: dict[tuple[str, float, float], list[tuple[str, dict[str, float]]]] = {}
    for window_key in window_order:
        bounds = overrides.get(window_key)
        if not bounds:
            continue
        group_key = (
            _cycle_window_family_key(window_key),
            round(bounds["y_min"], 2),
            round(bounds["y_max"], 2),
        )
        groups.setdefault(group_key, []).append((window_key, bounds))

    for entries in groups.values():
        previous_x_max: float | None = None
        for window_key, bounds in sorted(entries, key=lambda item: (item[1]["x_min"], item[1]["x_max"])):
            if previous_x_max is not None and bounds["x_min"] <= previous_x_max:
                trimmed_x_min = previous_x_max + CYCLE_WINDOW_BOUNDARY_EPS_MM
                # Keep a valid rectangle. If the planner produced a tiny leftover
                # sliver, keep it unchanged rather than deleting coverage silently.
                if trimmed_x_min < bounds["x_max"] - 1.0:
                    bounds = dict(bounds)
                    bounds["x_min"] = trimmed_x_min
                    overrides[window_key] = bounds
            previous_x_max = bounds["x_max"] if previous_x_max is None else max(previous_x_max, bounds["x_max"])

    return overrides


def _round_bounds(bounds: dict[str, float] | None) -> dict[str, float] | None:
    if bounds is None:
        return None
    return {key: round(float(value), 3) for key, value in bounds.items()}


def _write_cycle_debug(
    *,
    selected: str,
    alternate: str,
    cycles: int,
    step_mm: float,
    window_order: list[str],
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
    bounds_overrides: dict[str, dict[str, float]],
    expanded: list[dict[str, Any]],
) -> None:
    """Write the final runtime pocket-cycle split for robot-side debugging."""
    try:
        expanded_by_window: dict[str, list[dict[str, Any]]] = {}
        for path in expanded:
            expanded_by_window.setdefault(str(path.get("cycle_window_id") or ""), []).append(path)

        windows: list[dict[str, Any]] = []
        for window_key in window_order:
            source_paths = _paths_for_cycle_window_key(grouped, window_key)
            runtime_paths = expanded_by_window.get(window_key) or []
            windows.append(
                {
                    "window_key": window_key,
                    "family_key": _cycle_window_family_key(window_key),
                    "source_path_ids": [path.get("path_id") for path in source_paths],
                    "source_cycle_bounds": _round_bounds(_original_cycle_window_bounds(source_paths)),
                    "source_point_bounds": _round_bounds(_point_bounds_for_paths(source_paths)),
                    "runtime_bounds_override": _round_bounds(bounds_overrides.get(window_key)),
                    "runtime_point_bounds": _round_bounds(_point_bounds_for_paths(runtime_paths)),
                    "runtime_axis7_positions": sorted(
                        {
                            round(float(axis), 3)
                            for axis in (path.get("axis7_position_mm") for path in runtime_paths)
                            if _float_or_none(axis) is not None
                        }
                    ),
                    "runtime_path_ids": [path.get("path_id") for path in runtime_paths],
                }
            )

        _DEBUG_PATH.parent.mkdir(parents=True, exist_ok=True)
        _DEBUG_PATH.write_text(
            json.dumps(
                {
                    "selected": selected,
                    "alternate": alternate,
                    "cycles": cycles,
                    "step_mm": round(float(step_mm), 3),
                    "windows": windows,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
    except Exception as debug_error:  # noqa: BLE001 - debug must not block robot task
        logger.debug("[TableB DXF Run] pocket zigzag cycle debug skipped: %s", debug_error)


def _zigzag_adjusted_step(span: float, requested_step: float) -> float:
    step = requested_step if requested_step > 1e-9 else span
    steps = max(1, round(span / step))
    return span / steps


def _build_window_serpentine_from_bounds(
    bounds: dict[str, float],
    orientation: str,
    start_corner: list[float] | None,
    step_mm: float,
) -> list[list[float]]:
    x_lo = bounds["x_min"]
    x_hi = bounds["x_max"]
    y_lo = bounds["y_min"]
    y_hi = bounds["y_max"]
    have_start = start_corner is not None and len(start_corner) >= 2
    sx = float(start_corner[0]) if have_start else x_hi
    sy = float(start_corner[1]) if have_start else y_lo

    if orientation == "vertical":
        adjusted = _zigzag_adjusted_step(x_hi - x_lo, step_mm)
        lane_count = max(1, round((x_hi - x_lo) / adjusted))
        lanes = [x_lo + index * adjusted for index in range(lane_count + 1)]
        if abs(sx - x_hi) <= abs(sx - x_lo):
            lanes = list(reversed(lanes))
        sweep_from_hi = abs(sy - y_hi) < abs(sy - y_lo)
        out: list[list[float]] = []
        for lane in lanes:
            p0 = [lane, y_hi if sweep_from_hi else y_lo]
            p1 = [lane, y_lo if sweep_from_hi else y_hi]
            _append_continuous_points(out, [p0, p1])
            sweep_from_hi = not sweep_from_hi
        return out

    adjusted = _zigzag_adjusted_step(y_hi - y_lo, step_mm)
    lane_count = max(1, round((y_hi - y_lo) / adjusted))
    lanes = [y_lo + index * adjusted for index in range(lane_count + 1)]
    if abs(sy - y_hi) <= abs(sy - y_lo):
        lanes = list(reversed(lanes))
    sweep_from_hi = abs(sx - x_hi) < abs(sx - x_lo)
    out = []
    for lane in lanes:
        p0 = [x_hi if sweep_from_hi else x_lo, lane]
        p1 = [x_lo if sweep_from_hi else x_hi, lane]
        _append_continuous_points(out, [p0, p1])
        sweep_from_hi = not sweep_from_hi
    return out


def _remove_initial_duplicate_vertical_lane(
    points: list[list[float]],
    previous_end: list[float] | None,
    step_mm: float,
) -> list[list[float]]:
    """Avoid sanding the previous window's shared X-boundary lane again.

    Adjacent rectangular reach windows share a physical boundary. The previous
    window owns that boundary lane; the next window should start at its first
    interior lane so the robot does not repeat the same pass after the J7 move.
    """
    if previous_end is None or len(previous_end) < 2 or len(points) < 4:
        return points
    try:
        previous_x = float(previous_end[0])
        first_x = float(points[0][0])
        second_x = float(points[1][0])
    except (TypeError, ValueError, IndexError):
        return points
    same_first_lane = abs(first_x - second_x) <= 0.5
    near_previous_boundary = abs(first_x - previous_x) <= max(2.0, float(step_mm) * 0.1)
    if same_first_lane and near_previous_boundary:
        return points[2:]
    return points


def _combined_window_cycle_path(
    *,
    window_key: str,
    grouped: dict[str, dict[str, list[dict[str, Any]]]],
    selected: str,
    alternate: str,
    cycles: int,
    force: int,
    start_cursor: list[float] | None,
    step_mm: float,
    bounds_override: dict[str, float] | None = None,
) -> tuple[dict[str, Any] | None, list[float] | None]:
    """Build one force-contact path for all cycles inside one shared reach window.

    The visible preview can show one orientation. Multi-cycle execution needs a
    deterministic shared window, otherwise independently planned vertical and
    horizontal paths cause lift/reposition between cycles.
    """
    combined: list[list[float]] = []
    cursor = start_cursor
    first_path: dict[str, Any] | None = None
    orientations: list[str] = []
    all_window_paths: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for orientation_name in ("vertical", "horizontal", selected, alternate):
        for candidate in grouped.get(orientation_name, {}).get(window_key) or []:
            candidate_id = str(candidate.get("path_id") or id(candidate))
            if candidate_id in seen_ids:
                continue
            seen_ids.add(candidate_id)
            all_window_paths.append(candidate)

    shared_bounds = bounds_override or _window_bounds_for_paths(all_window_paths)
    if all_window_paths and first_path is None:
        first_path = dict(all_window_paths[0])

    for cycle_index in range(1, cycles + 1):
        orientation = selected if cycle_index % 2 == 1 else alternate
        window_paths = grouped.get(orientation, {}).get(window_key) or []
        if not window_paths and shared_bounds is None:
            return None, start_cursor

        if first_path is None and window_paths:
            first_path = dict(window_paths[0])
        bounds = bounds_override or _window_bounds_for_paths(window_paths) or shared_bounds
        if bounds is not None:
            pts = _build_window_serpentine_from_bounds(bounds, orientation, cursor, step_mm)
            if len(pts) < 2:
                return None, start_cursor
            _append_continuous_points(combined, pts)
            cursor = combined[-1]
        else:
            ordered_paths = _ordered_pattern_paths(window_paths, cursor)
            for path in ordered_paths:
                pts = _xy_points(path.get("points") or [])
                if len(pts) < 2:
                    continue
                rebuilt = _rebuild_serpentine_from_corner(pts, orientation, cursor)
                if rebuilt and len(rebuilt) >= 2:
                    pts = rebuilt
                _append_continuous_points(combined, pts)
                cursor = combined[-1]
        orientations.append(orientation)

    if first_path is None or len(combined) < 2:
        return None, start_cursor

    out = dict(first_path)
    out["points"] = combined
    out["path_id"] = f"{first_path.get('path_id') or 'pocketzigzag'}__{window_key}__cycles1-{cycles}_{selected}_{alternate}"
    out["operation"] = f"Pocket zigzag continuous cycles {cycles} ({'/'.join(orientations)})"
    out["recipe_override"] = {"force": force, "cycle": 1}
    out["cycle_index"] = None
    out["cycle_orientation"] = "+".join(orientations)
    out["cycle_window_id"] = window_key
    out["continuous_cycle_chain"] = True
    return out, combined[-1]


def _combined_rect_window_cycle_path(
    *,
    window_key: str,
    template_path: dict[str, Any],
    bounds: dict[str, float],
    selected: str,
    alternate: str,
    cycles: int,
    force: int,
    start_cursor: list[float] | None,
    step_mm: float,
) -> tuple[dict[str, Any] | None, list[float] | None]:
    """Generate all alternating cycles inside one clean rectangular reach window."""
    combined: list[list[float]] = []
    cursor = start_cursor
    orientations: list[str] = []

    for cycle_index in range(1, cycles + 1):
        orientation = selected if cycle_index % 2 == 1 else alternate
        pts = _build_window_serpentine_from_bounds(bounds, orientation, cursor, step_mm)
        if cycle_index == 1 and orientation == "vertical":
            pts = _remove_initial_duplicate_vertical_lane(pts, cursor, step_mm)
        if len(pts) < 2:
            return None, start_cursor
        _append_continuous_points(combined, pts)
        cursor = combined[-1]
        orientations.append(orientation)

    if len(combined) < 2:
        return None, start_cursor

    out = dict(template_path)
    out["points"] = combined
    out["path_id"] = f"{template_path.get('path_id') or 'pocketzigzag'}__{window_key}__cycles1-{cycles}_{selected}_{alternate}"
    out["operation"] = f"Pocket zigzag continuous cycles {cycles} ({'/'.join(orientations)})"
    out["recipe_override"] = {"force": force, "cycle": 1}
    out["cycle_index"] = None
    out["cycle_orientation"] = "+".join(orientations)
    out["cycle_window_id"] = window_key
    out["cycle_window_bounds"] = dict(bounds)
    out["continuous_cycle_chain"] = True
    return out, combined[-1]


def build_tool4_rectangular_alternating_cycle_paths(
    *,
    source_paths: list[dict[str, Any]],
    cycles: int,
    force: int,
    step_mm: float,
    selected: str = "vertical",
    alternate: str | None = None,
    path_prefix: str = "tool4_rect",
    operation_label: str = "Tool 4 rectangular alternating",
) -> list[dict[str, Any]] | None:
    """Build clean Tool 4 rectangular cycle windows for pocket or flat-frame fills.

    This is the shared runtime planner for large rectangular sanding surfaces:
    one full rectangle is recovered from the approved preview paths, split into
    non-overlapping Tool 4 reach windows, then each window receives a continuous
    selected/alternate cycle chain. Preview geometry stays unchanged; this only
    changes the robot execution paths for cycle > 1.
    """
    if cycles <= 1 or not source_paths:
        return None
    selected = "horizontal" if selected == "horizontal" else "vertical"
    alternate = alternate or ("vertical" if selected == "horizontal" else "horizontal")
    alternate = "horizontal" if alternate == "horizontal" else "vertical"

    bounds_list: list[dict[str, float]] = []
    for path in source_paths:
        bounds = _float_bounds(path.get("cycle_window_bounds"))
        if bounds is None:
            bounds = _point_bounds_for_paths([path])
        if bounds is not None:
            bounds_list.append(bounds)
    if not bounds_list:
        return None

    full_bounds = {
        "x_min": min(bounds["x_min"] for bounds in bounds_list),
        "x_max": max(bounds["x_max"] for bounds in bounds_list),
        "y_min": min(bounds["y_min"] for bounds in bounds_list),
        "y_max": max(bounds["y_max"] for bounds in bounds_list),
    }

    template = dict(source_paths[0])
    expanded: list[dict[str, Any]] = []
    cursor: list[float] | None = None
    windows = _trim_internal_rect_window_boundaries(_split_rect_bounds_by_tool4_reach(full_bounds), step_mm)
    for index, bounds in enumerate(windows, start=1):
        window_key = f"{path_prefix}_exec{index}"
        combined_path, cursor_after = _combined_rect_window_cycle_path(
            window_key=window_key,
            template_path=template,
            bounds=bounds,
            selected=selected,
            alternate=alternate,
            cycles=cycles,
            force=force,
            start_cursor=cursor,
            step_mm=step_mm,
        )
        if combined_path is None:
            continue
        combined_path["path_id"] = f"{path_prefix}_{index:03d}__cycles1-{cycles}_{selected}_{alternate}"
        combined_path["operation"] = f"{operation_label} cycles {cycles}"
        combined_path["cycle_window_id"] = window_key
        expanded.append(combined_path)
        cursor = cursor_after

    if expanded:
        attach_station_plan(expanded, tool="tool_4", chain=False)
    return expanded or None


def _same_cycle_window(a: dict[str, Any], b: dict[str, Any]) -> bool:
    return a.get("cycle_window_id") == b.get("cycle_window_id")


def _pocket_zigzag_chain_reachable(points: list[list[float]]) -> bool:
    return station_window_for_path("tool_4", points) is not None


def _stitch_compatible_pocket_zigzag_paths(paths: list[dict[str, Any]], tolerance_mm: float = 5.0) -> list[dict[str, Any]]:
    """Merge pocket-zigzag cycle paths that can run under one force contact.

    Cycle alternation is generated as separate vertical/horizontal preview paths,
    but the robot should not lift and re-force between compatible cycles. If the
    next path is the same pocket window and same 7th-axis station, append it to
    the current MoveL chain. A small endpoint gap skips the duplicate point; a
    larger gap becomes a normal in-contact connector inside that same window.
    """
    stitched: list[dict[str, Any]] = []
    for path in paths:
        pts = _xy_points(path.get("points") or [])
        if len(pts) < 2:
            continue
        path = dict(path)
        path["points"] = pts
        if not stitched:
            stitched.append(path)
            continue

        previous = stitched[-1]
        previous_pts = _xy_points(previous.get("points") or [])
        if previous_pts and _same_cycle_window(previous, path):
            gap = _point_distance(previous_pts[-1], pts[0])
            merged_pts = list(previous_pts)
            if gap <= 0.5:
                merged_pts.extend(pts[1:])
            else:
                merged_pts.extend(pts)
            if not _pocket_zigzag_chain_reachable(merged_pts):
                stitched.append(path)
                continue
            previous["points"] = merged_pts
            previous["path_id"] = f"{previous.get('path_id', 'pocketzigzag')}+{path.get('path_id', 'cycle')}"
            previous["operation"] = f"{previous.get('operation', 'Pocket zigzag')} + {path.get('operation', 'Pocket zigzag')}"
            previous["cycle_orientation"] = f"{previous.get('cycle_orientation', '')}+{path.get('cycle_orientation', '')}"
            previous["continuous_cycle_chain"] = True
            continue

        stitched.append(path)
    return stitched

def _pocket_zigzag_cycle_paths(approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]] | None:
    zigzag_recipe = _normalize_recipe_entry(recipe.get("pocketzigzag") if isinstance(recipe, dict) else None)
    cycles = zigzag_recipe["cycle"]
    if cycles <= 1:
        return None

    settings = approved.get("settings") if isinstance(approved, dict) else None
    patterns = settings.get("pocket_zigzag_cycle_patterns") if isinstance(settings, dict) else None
    if not isinstance(patterns, dict):
        return None

    raw_selected = patterns.get("selected_orientation") or settings.get("pocket_zigzag_orientation") or "vertical"
    rect_spiral_selected = raw_selected == "rectspiral"
    selected = "horizontal" if raw_selected == "horizontal" else "vertical"
    alternate = "vertical" if selected == "horizontal" else "horizontal"
    try:
        overlap_mm = max(0.0, min(100.0, float(settings.get("pocket_overlap_mm") or 0.0)))
    except (TypeError, ValueError):
        overlap_mm = 0.0
    step_mm = max(TOOL4_POCKET_PASS_WIDTH_MM - overlap_mm, 1.0)
    vertical_paths = [dict(path) for path in (patterns.get("vertical_paths") or [])]
    horizontal_paths = [dict(path) for path in (patterns.get("horizontal_paths") or [])]
    spiral_paths = [dict(path) for path in (patterns.get("spiral_paths") or [])]
    # Offset-outward spiral variant (rings shifted by s/2, threaded between the inward rings) so
    # even cycles spiral OUT on a different path instead of reversing the inward line. Both in and
    # out normalize to the SAME window key per section (see _pocket_zigzag_window_key) so they
    # stitch continuously. Missing/empty (older approved payloads) -> falls back to inward.
    spiral_out_paths = [dict(path) for path in (patterns.get("spiral_out_paths") or [])]
    by_orientation = {"vertical": vertical_paths, "horizontal": horizontal_paths}

    grouped = {
        "vertical": _group_pocket_zigzag_windows(vertical_paths),
        "horizontal": _group_pocket_zigzag_windows(horizontal_paths),
        "rectspiral": _group_pocket_zigzag_windows(spiral_paths),
        "rectspiral_out": _group_pocket_zigzag_windows(spiral_out_paths),
    }

    window_order: list[str] = []
    if rect_spiral_selected:
        # Rectangular spiral has no vertical/horizontal alternation; every cycle reruns the
        # same spiral window (reversed from the cursor for the return pass). Drive the window
        # order purely from the spiral paths.
        if not spiral_paths:
            return None
        for path in spiral_paths:
            key = _pocket_zigzag_window_key(path)
            if key not in window_order:
                window_order.append(key)
    else:
        if not by_orientation[selected] or not by_orientation[alternate]:
            logger.info(
                "[TableB DXF Run] pocket zigzag cycle alternation has incomplete hidden paths "
                "selected=%s count=%s alternate=%s count=%s; using available window bounds",
                selected,
                len(by_orientation[selected]),
                alternate,
                len(by_orientation[alternate]),
            )
        # Window order must come from one source orientation only. The alternate
        # orientation is consumed inside the same window by _combined_window_cycle_path.
        # Adding alternate-only keys here creates duplicate line/U fragments after the
        # correct continuous cycles finish.
        source_paths = by_orientation[selected] or by_orientation[alternate]
        for path in source_paths:
            key = _pocket_zigzag_window_key(path)
            if key not in window_order:
                window_order.append(key)
        if not window_order:
            for orientation_paths in (vertical_paths, horizontal_paths):
                for path in orientation_paths:
                    key = _pocket_zigzag_window_key(path)
                    if key not in window_order:
                        window_order.append(key)
        logger.info(
            "[TableB DXF Run] pocket zigzag continuous cycles selected=%s alternate=%s cycles=%s windows=%s",
            selected,
            alternate,
            cycles,
            len(window_order),
        )

    expanded: list[dict[str, Any]] = []
    cursor: list[float] | None = None
    cycle_bounds_overrides = _partition_cycle_window_bounds(window_order, grouped)
    clean_rect_windows = [] if rect_spiral_selected else _build_rectangular_cycle_windows(window_order, grouped, step_mm)
    clean_rect_by_key = {window_key: (bounds, template) for window_key, bounds, template in clean_rect_windows}
    if clean_rect_windows:
        for window_key, bounds, _template in clean_rect_windows:
            cycle_bounds_overrides[window_key] = dict(bounds)
        effective_window_order = [
            *(window_key for window_key, _bounds, _template in clean_rect_windows),
            *(
                window_key
                for window_key in window_order
                if _is_triangle_pocket_zigzag_window(window_key) or _is_rect_spiral_pocket_zigzag_window(window_key)
            ),
        ]
    else:
        effective_window_order = window_order

    for window_key in effective_window_order:
        clean_rect = clean_rect_by_key.get(window_key)
        if clean_rect is not None:
            bounds, template = clean_rect
            combined_path, cursor_after = _combined_rect_window_cycle_path(
                window_key=window_key,
                template_path=template,
                bounds=bounds,
                selected=selected,
                alternate=alternate,
                cycles=cycles,
                force=zigzag_recipe["force"],
                start_cursor=cursor,
                step_mm=step_mm,
            )
            if combined_path is not None:
                expanded.append(combined_path)
                cursor = cursor_after
            continue

        triangle_window = _is_triangle_pocket_zigzag_window(window_key)
        rect_spiral_window = _is_rect_spiral_pocket_zigzag_window(window_key)
        # Spiral-type windows (triangle or rectangular) have no vertical/horizontal fill:
        # reuse the same spiral each cycle and let _ordered_pattern_paths reverse it from the
        # cursor for the return pass. They must NOT go through the serpentine rebuild.
        spiral_window = triangle_window or rect_spiral_window
        if not spiral_window:
            combined_path, cursor_after = _combined_window_cycle_path(
                window_key=window_key,
                grouped=grouped,
                selected=selected,
                alternate=alternate,
                cycles=cycles,
                force=zigzag_recipe["force"],
                start_cursor=cursor,
                step_mm=step_mm,
                bounds_override=cycle_bounds_overrides.get(window_key),
            )
            if combined_path is not None:
                expanded.append(combined_path)
                cursor = cursor_after
                continue

        for cycle_index in range(1, cycles + 1):
            orientation = selected if cycle_index % 2 == 1 else alternate
            if triangle_window:
                orientation = "triangle_spiral"
                window_paths = grouped[selected].get(window_key) or grouped[alternate].get(window_key) or []
            elif rect_spiral_window:
                # Alternate IN / OUT so the spiral never retraces (no reversed ping-pong):
                # odd cycles = inward spiral (corner->center); even cycles = the offset-outward
                # spiral (center->corner on rings threaded between the inward rings). Fall back to
                # the inward spiral when no offset-out variant was shipped (older payloads).
                if cycle_index % 2 == 1:
                    orientation = "rectangular_spiral_in"
                    window_paths = grouped["rectspiral"].get(window_key) or []
                else:
                    orientation = "rectangular_spiral_out"
                    window_paths = (
                        grouped["rectspiral_out"].get(window_key)
                        or grouped["rectspiral"].get(window_key)
                        or []
                    )
            else:
                window_paths = grouped[orientation].get(window_key) or []
            if not window_paths:
                continue
            ordered_paths = _ordered_pattern_paths(window_paths, cursor)
            for path_index, path in enumerate(ordered_paths, start=1):
                pts = _xy_points(path.get("points") or [])
                if len(pts) < 2:
                    continue
                # For vertical/horizontal fill, rebuild this window's serpentine so it STARTS
                # at the corner where the previous cycle ended and sweeps the correct way
                # (continuous alternation, no lift). Reversal alone can't do this. Spiral
                # windows keep the reordered/reversed path. Falls back to pts on failure. Only
                # the first path of a window seeds from the cross-cycle cursor; further paths in
                # the same window continue from the running end so lanes stay contiguous.
                if not spiral_window:
                    rebuilt = _rebuild_serpentine_from_corner(pts, orientation, cursor)
                    if rebuilt and len(rebuilt) >= 2:
                        pts = rebuilt
                expanded_path = dict(path)
                expanded_path["points"] = pts
                expanded_path["path_id"] = (
                    f"{path.get('path_id') or f'pocketzigzag_{path_index}'}"
                    f"__{window_key}__cycle{cycle_index}_{orientation}"
                )
                if triangle_window:
                    operation_label = "Pocket zigzag triangle spiral"
                elif rect_spiral_window:
                    operation_label = "Pocket zigzag rectangular spiral"
                else:
                    operation_label = f"Pocket zigzag {orientation}"
                expanded_path["operation"] = f"{operation_label} cycle {cycle_index}"
                expanded_path["recipe_override"] = {"force": zigzag_recipe["force"], "cycle": 1}
                expanded_path["cycle_index"] = cycle_index
                expanded_path["cycle_orientation"] = orientation
                expanded_path["cycle_window_id"] = window_key
                expanded.append(expanded_path)
                cursor = pts[-1]

    expanded = _stitch_compatible_pocket_zigzag_paths(expanded)
    if expanded:
        # Cycle expansion can reorder/rebuild Tool 4 pocket paths after the preview reach
        # planner ran. Re-plan the final points so each execution path carries a legal J7
        # station for the exact geometry the robot will run.
        attach_station_plan(expanded, tool="tool_4", chain=False)
    _write_cycle_debug(
        selected=selected,
        alternate=alternate,
        cycles=cycles,
        step_mm=step_mm,
        window_order=effective_window_order,
        grouped=grouped,
        bounds_overrides=cycle_bounds_overrides,
        expanded=expanded,
    )
    return expanded or None


def _expand_pocket_zigzag_cycles(paths: list[dict[str, Any]], approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]]:
    expanded_zigzag = _pocket_zigzag_cycle_paths(approved, recipe)
    if not expanded_zigzag:
        return paths

    out: list[dict[str, Any]] = []
    inserted = False
    for path in paths:
        if str(path.get("tool", "")) == "tool_4":
            if not inserted:
                out.extend(expanded_zigzag)
                inserted = True
            continue
        out.append(path)
    if not inserted:
        out.extend(expanded_zigzag)
    return out



expand_pocket_zigzag_cycles = _expand_pocket_zigzag_cycles
