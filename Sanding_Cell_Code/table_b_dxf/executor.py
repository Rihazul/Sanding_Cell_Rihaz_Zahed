from __future__ import annotations

"""Run an operator-approved Table B DXF toolpath on the robot.

Table B no longer uses the legacy model presets (Table<N>Model / model<N>cycle).
A run is fully defined by the approved toolpath JSON the 2D DXF workspace saved for
a job, plus the force/cycle recipe the operator set in the UI.

SAFETY / STATUS: this module resolves the approved plan and hands it to the
Table B DXF robot runner for real execution. Motion is executed only from the
child process that owns a fresh CPS session.
"""

import logging
import re
from typing import Any

from .jobs import load_approved_toolpath
from .tool_reach import TOOL_REACH, attach_station_plan, reach_span_at_y, station_window_for_path

logger = logging.getLogger(__name__)

TOOL4_POCKET_PASS_WIDTH_MM = 144.0

# Real execution is enabled. The symbol stays available so Flask can report the
# current mode to the UI, but the dry-run bypass is no longer used.
DRY_RUN = False

# ---------------------------------------------------------------------------
# Motion mapping. The approved toolpath carries [x, y] millimetre points in the
# machine frame plus a logical tool tag. Runtime TCP/UCS/Z/orientation/speeds are
# resolved in table_b_dxf.execution.path_runner from configs/config.yaml. Force
# and cycle come from the operator's recipe (see _recipe_for_tool).
# ---------------------------------------------------------------------------
TOOL_MOTION_PARAMS: dict[str, dict[str, Any]] = {
    # Tools 1, 3, and 4 use the shared Table B XY-force runner:
    # [X, Y, Z, 180, 0, 0], Z+ force approach, preheight Z=-17mm.
    # TCP/UCS names and speeds are resolved from configs/config.yaml at runtime.
    "tool_3": {"runner": "table_b_force_xy"},
    "tool_4": {"runner": "table_b_force_xy"},
    "tool_3d": {"runner": "table_b_force_xy"},
    "tool_4_frame": {"runner": "table_b_force_xy"},
    "frame_section": {"runner": "table_b_force_xy"},
    "tool_2_side": {"runner": "table_b_tool2_side"},
    "tool_2_edgeOutside": {"runner": "table_b_tool2_side"},
}

# --- Physical tool mapping -------------------------------------------------
# The 2D viewer tags each path with a logical tool name. Map those to the physical
# tool the changer mounts, and to the UI recipe row that supplies force/cycle.
#
#   Pocket Edge          -> Tool 3
#   Frame, Pocket ZigZag -> Tool 4
#   3D                   -> Tool 1
#   Edge Outside, Side   -> Tool 2   (no 2D DXF paths generate these yet)
#
# `tool_3d*` is matched by prefix: the viewer emits ids like "tool_3d_pocket".
_TOOL_SPECS: dict[str, dict[str, Any]] = {
    "tool_3": {"physical_tool": 3, "recipe_key": "pocketedge", "label": "Pocket edge"},
    "tool_4": {"physical_tool": 4, "recipe_key": "pocketzigzag", "label": "Pocket zigzag"},
    "tool_4_frame": {"physical_tool": 4, "recipe_key": "frame", "label": "Frame zigzag"},
    "frame_section": {"physical_tool": 4, "recipe_key": "frame", "label": "Frame section pass"},
    "tool_3d": {"physical_tool": 1, "recipe_key": "3D", "label": "3D contour"},
    # Tool 2 paths are derived from the part's outer corners (see _build_tool2_paths),
    # not drawn in the viewer.
    "tool_2_side": {"physical_tool": 2, "recipe_key": "side", "label": "Side"},
    "tool_2_edgeOutside": {"physical_tool": 2, "recipe_key": "edgeOutside", "label": "Edge outside"},
}

# Tool batch order. All XY-plane work is completed first (pocket edge -> frame /
# pocket zigzag -> 3D), then the side/edge tool runs last. Batching by tool keeps
# tool changes to a minimum: each tool is mounted at most once per run.
TOOL_BATCH_ORDER = (3, 4, 1, 2)

# --- Tool detection (CI sensor map) ---------------------------------------
# The gripper reports which tool is mounted through three CI bits. The mapping is
# taken from the Table A reference (smallTable/smallmodelfinal.py check_tool /
# decode_tool_in_hand) and MUST match the physical wiring.
#   (CI0, CI1, CI2) -> tool number, or None for an empty hand.
CI_TOOL_MAP: dict[tuple[int, int, int], int | None] = {
    (1, 0, 0): 3,
    (0, 1, 0): 2,
    (0, 1, 1): 1,
    (0, 0, 1): 4,
    (0, 0, 0): None,  # empty hand
}

# Joint-only correction for the tool 1 <-> tool 3 transition. moveOnlyJ6r applies a
# RELATIVE offset to the current joint positions, so the sign depends on which way the
# arm is travelling: going 3 -> 1 must undo the rotation that 1 -> 3 applies.
#
# 1 -> 3 comes from the Table A reference (smallTable/smallmodelfinal*.py):
#     moveOnlyJ6r(cps, -90, config, J1=30, wait=True)
# 3 -> 1 is its inverse. Table A never performs it (its order is 3->4->2->1, so tool 1
# is last); Table B's 3->4->1->2 order does, in-run.
#
# It must be a joint move, not a Cartesian one: interpolating in Cartesian space to the
# tool-1/tool-3 station pose twists the wrist and can crash.
TOOL_1_3_JOINT_CORRECTIONS: dict[tuple[int, int], dict[str, float]] = {
    (1, 3): {"J1": 30.0, "J6": -90.0},
    (3, 1): {"J1": -30.0, "J6": 90.0},
}


def decode_tool_in_hand(ci0: int | None, ci1: int | None, ci2: int | None) -> int | None:
    """Decode the mounted tool from the CI bits.

    Raises TableBDxfExecutionError when the bits cannot be read or describe a
    combination that is not in the map: an unknown gripper state is never assumed
    to be safe.
    """
    if ci0 is None or ci1 is None or ci2 is None:
        raise TableBDxfExecutionError(
            "Failed to read tool-detection CI bits — cannot verify which tool is mounted."
        )
    key = (int(ci0), int(ci1), int(ci2))
    if key not in CI_TOOL_MAP:
        raise TableBDxfExecutionError(
            f"Unrecognized tool-detection CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}. "
            "Refusing to move with an unknown tool state."
        )
    return CI_TOOL_MAP[key]


def plan_tool_change(current_tool: int | None, requested_tool: int) -> list[dict[str, Any]]:
    """The safety sequence required to end up holding `requested_tool`.

    Mirrors the Table A reference: verify what is mounted, drop the wrong tool from
    the tool station with the rail idle, then pick the requested one. Returns the
    ordered steps so the sequence can be logged and reviewed before any motion.
    """
    if current_tool == requested_tool:
        return [{"action": "keep", "detail": f"Tool {requested_tool} already in hand; no change needed."}]

    steps: list[dict[str, Any]] = []
    if current_tool is not None:
        # Wrong tool mounted: it must be returned before picking another.
        steps.append({"action": "move_to_safe_point", "detail": "Arm to safe point before rail move."})
        steps.append({"action": "move_seventh_to_tool_station", "detail": "Rail (J7) to tool station."})
        steps.append({"action": "wait_for_j7_idle", "detail": "Rail must be idle before the changer engages."})
        steps.append({"action": "drop_tool", "tool": current_tool, "detail": f"Return tool {current_tool}."})
        # Tool 1 <-> Tool 3 needs a joint-only J6 move before the pick, otherwise the
        # wrist twists on the way to the station pose and can crash. The offsets are
        # relative, so each direction gets its own signs (3 -> 1 undoes 1 -> 3).
        correction = TOOL_1_3_JOINT_CORRECTIONS.get((current_tool, requested_tool))
        if correction:
            steps.append({
                "action": "joint_correction",
                "j1": correction["J1"],
                "j6": correction["J6"],
                "detail": (
                    f"Tool {current_tool} -> Tool {requested_tool} transition: joint-only "
                    f"J1={correction['J1']:+g}, J6={correction['J6']:+g} before picking "
                    "(moveOnlyJ6r — avoids a wrist twist/crash)."
                ),
            })
    else:
        steps.append({"action": "move_to_safe_point", "detail": "Arm to safe point before picking."})

    steps.append({"action": "pick_tool", "tool": requested_tool, "detail": f"Pick tool {requested_tool}."})
    steps.append({"action": "verify_tool", "tool": requested_tool,
                  "detail": f"Re-read CI bits and confirm tool {requested_tool} is mounted."})
    return steps


def _spec_for_tool(tool: str) -> dict[str, Any] | None:
    """Resolve a viewer tool tag to its physical tool + recipe row."""
    spec = _TOOL_SPECS.get(tool)
    if spec:
        return spec
    # Viewer emits variants such as "tool_3d_pocket" / "tool_3d_frame".
    if tool.startswith("tool_3d"):
        return _TOOL_SPECS["tool_3d"]
    return None


def _motion_for_tool(tool: str) -> dict[str, Any] | None:
    motion = TOOL_MOTION_PARAMS.get(tool)
    if motion:
        return motion
    if tool.startswith("tool_3d"):
        return TOOL_MOTION_PARAMS.get("tool_3d")
    return None


# Tool 2 (side / edge outside) sands the door thickness. It does not use the
# closed XY surface loop used by tools 1/3/4; it runs four independent outer side
# segments derived from the approved structural outer corner points.
_OUTER_CORNER_LABEL = "door outer corners"

# Operator-defined Tool 2 side sequence and RZ orientation. The X=min side is
# named "right" and X=max is named "left" to match the station convention.
_TOOL2_SIDE_DEFS: tuple[tuple[str, int, float], ...] = (
    ("right", 1, 0.0),
    ("bottom", 2, 90.0),
    ("left", 3, 180.0),
    ("top", 4, -90.0),
)


def _outer_corner_points(approved: dict[str, Any]) -> list[list[float]] | None:
    """The part's 4 outer corner points (mm) from the approved toolpath."""
    for region in approved.get("regions") or []:
        for shape in region.get("corner_shapes") or []:
            if str(shape.get("label", "")).strip().lower() == _OUTER_CORNER_LABEL:
                points = shape.get("points") or []
                if len(points) >= 4:
                    return [[float(p[0]), float(p[1])] for p in points]
    return None


def _outer_bounds(approved: dict[str, Any]) -> tuple[float, float, float, float] | None:
    """Return min_x, min_y, max_x, max_y for the rectangular-bounded door."""
    corners = _outer_corner_points(approved)
    if not corners:
        return None
    xs = [float(point[0]) for point in corners]
    ys = [float(point[1]) for point in corners]
    min_x = min(xs)
    min_y = min(ys)
    max_x = max(xs)
    max_y = max(ys)
    if max_x <= min_x or max_y <= min_y:
        return None
    return min_x, min_y, max_x, max_y


def _tool2_side_points(side_label: str, bounds: tuple[float, float, float, float]) -> list[list[float]]:
    min_x, min_y, max_x, max_y = bounds
    if side_label == "right":
        return [[min_x, min_y], [min_x, max_y]]
    if side_label == "bottom":
        return [[min_x, min_y], [max_x, min_y]]
    if side_label == "left":
        return [[max_x, min_y], [max_x, max_y]]
    if side_label == "top":
        return [[min_x, max_y], [max_x, max_y]]
    raise ValueError(f"Unknown Tool 2 side label: {side_label}")


def _build_tool2_paths(approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]]:
    """Synthesize Tool 2 side / edge-outside paths from the outer corners.

    Tool 2 is side-face sanding, so each side is an open 2-point line with a fixed
    RZ. Sequence is right -> bottom -> left -> top to avoid unnecessary J7 travel.
    """
    bounds = _outer_bounds(approved)
    if not bounds:
        return []

    paths: list[dict[str, Any]] = []
    for recipe_key, label in (("side", "Side"), ("edgeOutside", "Edge outside")):
        entry = recipe.get(recipe_key)
        if not isinstance(entry, dict):
            continue
        try:
            cycle = int(entry.get("cycle") or 0)
        except (TypeError, ValueError):
            cycle = 0
        if cycle <= 0:
            continue  # operation not selected
        for side_label, side_sequence, rz_deg in _TOOL2_SIDE_DEFS:
            paths.append({
                "path_id": f"outer_{recipe_key}_{side_label}",
                "tool": f"tool_2_{recipe_key}",
                "operation": f"{label} {side_label}",
                "closed": False,
                "side_label": side_label,
                "side_sequence": side_sequence,
                "rz_deg": rz_deg,
                "points": _tool2_side_points(side_label, bounds),
            })
    return paths

class TableBDxfExecutionError(RuntimeError):
    """Raised when an approved toolpath cannot be run."""


def _recipe_for_tool(tool: str, recipe: dict[str, Any]) -> dict[str, Any]:
    """Force/cycle the operator set for the row that drives this tool."""
    spec = _spec_for_tool(tool)
    entry = recipe.get(spec["recipe_key"]) if spec else None
    return _normalize_recipe_entry(entry)


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

    shared_bounds = _window_bounds_for_paths(all_window_paths)
    if all_window_paths and first_path is None:
        first_path = dict(all_window_paths[0])

    for cycle_index in range(1, cycles + 1):
        orientation = selected if cycle_index % 2 == 1 else alternate
        window_paths = grouped.get(orientation, {}).get(window_key) or []
        if not window_paths and shared_bounds is None:
            return None, start_cursor

        if first_path is None and window_paths:
            first_path = dict(window_paths[0])
        bounds = _window_bounds_for_paths(window_paths) or shared_bounds
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
        for path in by_orientation[selected]:
            key = _pocket_zigzag_window_key(path)
            if key not in window_order:
                window_order.append(key)
        for path in by_orientation[alternate]:
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
    for window_key in window_order:
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


def _is_flat_frame_zigzag_path(path: dict[str, Any]) -> bool:
    if str(path.get("tool") or "") != "tool_4_frame":
        return False
    operation_type = str(path.get("operation_type") or "").lower()
    operation = str(path.get("operation") or "").lower()
    path_id = str(path.get("path_id") or "").lower()
    # Computed-frame sections also use Tool 4, but those are already section-planned.
    # Only whole-door Frame Level zigzag paths should get alternate-cycle expansion.
    return (
        operation_type == "frame_zigzag_pass"
        or "frame zigzag" in operation
        or path_id.startswith("frame_zigzag")
    )


def _frame_zigzag_station_key(path: dict[str, Any], index: int) -> str:
    station = path.get("station_index")
    axis = path.get("axis7_position_mm")
    if station is not None:
        return f"station:{station}"
    if axis is not None:
        try:
            return f"axis:{round(float(axis), 1)}"
        except (TypeError, ValueError):
            pass
    return f"path:{index}"


def _frame_zigzag_cycle_paths(paths: list[dict[str, Any]], approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]] | None:
    frame_recipe = _normalize_recipe_entry(recipe.get("frame") if isinstance(recipe, dict) else None)
    cycles = frame_recipe["cycle"]
    if cycles <= 1:
        return None

    frame_paths = [dict(path) for path in paths if _is_flat_frame_zigzag_path(path)]
    if not frame_paths:
        return None

    settings = approved.get("settings") if isinstance(approved, dict) else None
    try:
        overlap_mm = max(0.0, min(100.0, float((settings or {}).get("frame_overlap_mm") or 0.0)))
    except (TypeError, ValueError):
        overlap_mm = 0.0
    step_mm = max(TOOL4_POCKET_PASS_WIDTH_MM - overlap_mm, 1.0)

    grouped: dict[str, list[dict[str, Any]]] = {}
    for index, path in enumerate(frame_paths):
        grouped.setdefault(_frame_zigzag_station_key(path, index), []).append(path)

    expanded: list[dict[str, Any]] = []
    cursor: list[float] | None = None
    for _, station_paths in sorted(
        grouped.items(),
        key=lambda item: min(float(path.get("axis7_position_mm") or 0.0) for path in item[1]),
    ):
        bounds = _window_bounds_for_paths(station_paths)
        if bounds is None:
            points = []
            for path in station_paths:
                points.extend(_xy_points(path.get("points") or []))
            bounds = _float_bounds({
                "x_min": min((p[0] for p in points), default=0.0),
                "x_max": max((p[0] for p in points), default=0.0),
                "y_min": min((p[1] for p in points), default=0.0),
                "y_max": max((p[1] for p in points), default=0.0),
            })
        if bounds is None:
            continue

        combined: list[list[float]] = []
        orientations: list[str] = []
        for cycle_index in range(1, cycles + 1):
            orientation = "vertical" if cycle_index % 2 == 1 else "horizontal"
            pts = _build_window_serpentine_from_bounds(bounds, orientation, cursor, step_mm)
            if len(pts) < 2:
                continue
            _append_continuous_points(combined, pts)
            cursor = combined[-1]
            orientations.append(orientation)

        if len(combined) < 2:
            continue

        first_path = dict(station_paths[0])
        first_path["points"] = combined
        first_path["path_id"] = f"{first_path.get('path_id') or 'frame_zigzag'}__cycles1-{cycles}_{'_'.join(orientations)}"
        first_path["operation"] = f"Frame zigzag continuous cycles {cycles} ({'/'.join(orientations)})"
        first_path["recipe_override"] = {"force": frame_recipe["force"], "cycle": 1}
        first_path["cycle_index"] = None
        first_path["cycle_orientation"] = "+".join(orientations)
        first_path["continuous_cycle_chain"] = True
        expanded.append(first_path)

    if expanded:
        attach_station_plan(expanded, tool="tool_4", chain=False)
    return expanded or None


def _expand_frame_zigzag_cycles(paths: list[dict[str, Any]], approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]]:
    expanded_frame = _frame_zigzag_cycle_paths(paths, approved, recipe)
    if not expanded_frame:
        return paths

    out: list[dict[str, Any]] = []
    inserted = False
    for path in paths:
        if _is_flat_frame_zigzag_path(path):
            if not inserted:
                out.extend(expanded_frame)
                inserted = True
            continue
        out.append(path)
    if not inserted:
        out.extend(expanded_frame)
    return out


def _robot_base_points(viewer_points: list[list[float]], axis7_position_mm: float | None) -> list[list[float]]:
    """Convert viewer/global UCS points into the robot-base frame at one J7 stop.

    The 2D viewer is in the fixed Table B UCS. Moving the 7th axis shifts the robot
    base along UCS X, so the robot must receive local X = viewer X - J7 position.
    Y is unchanged because the rail only translates the base in X.
    """
    if axis7_position_mm is None:
        return [list(point) for point in viewer_points]
    return [[point[0] - axis7_position_mm, point[1]] for point in viewer_points]


def _reach_tool_for_physical_tool(physical_tool: int) -> str | None:
    if physical_tool in (1, 3, 4):
        return f"tool_{physical_tool}"
    return None


def _low_y_limit_for_reach_tool(reach_tool: str) -> float | None:
    spec = TOOL_REACH.get(reach_tool) or {}
    try:
        return float(spec.get("rect_below_y"))
    except (TypeError, ValueError):
        return None


def _is_negative_local_x_in_low_y_zone(reach_tool: str, local_x: float, y: float) -> bool:
    low_y_limit = _low_y_limit_for_reach_tool(reach_tool)
    return low_y_limit is not None and float(y) < low_y_limit and float(local_x) < 0.0


def _sample_segment_points(points: list[list[float]], max_step_mm: float = 25.0) -> list[list[float]]:
    if len(points) < 2:
        return [list(point) for point in points]

    sampled: list[list[float]] = [list(points[0])]
    for a, b in zip(points, points[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dist = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
        count = int(dist // max(max_step_mm, 1.0))
        for index in range(1, count + 1):
            t = (index * max_step_mm) / dist if dist > 1e-9 else 1.0
            if t < 1.0:
                sampled.append([ax + (bx - ax) * t, ay + (by - ay) * t])
        sampled.append([bx, by])
    return sampled


def _validate_reach_at_station(step: dict[str, Any]) -> None:
    """Reject an approved path if its assigned J7 stop cannot physically reach it.

    The viewer stores global Table-B UCS XY points. Robot execution converts them to
    local base coordinates with local_x = viewer_x - axis7_position_mm. At low Y,
    each tool's envelope forbids negative local X; sampling every segment catches
    unsafe connectors between otherwise-valid endpoints.
    """
    reach_tool = _reach_tool_for_physical_tool(int(step.get("physical_tool") or 0))
    if reach_tool is None:
        return

    axis = step.get("axis7_position_mm")
    if axis is None:
        raise TableBDxfExecutionError(
            f"Path {step.get('path_id')} has no 7th-axis position. Preview/approve the reach-planned toolpath again."
        )
    axis = float(axis)

    for point in _sample_segment_points(step.get("viewer_points") or []):
        x, y = float(point[0]), float(point[1])
        span = reach_span_at_y(reach_tool, y)
        if span is None:
            raise TableBDxfExecutionError(
                f"Path {step.get('path_id')} has unreachable Y={y:.1f} for {reach_tool}."
            )
        local_x = x - axis
        if _is_negative_local_x_in_low_y_zone(reach_tool, local_x, y):
            low_y_limit = _low_y_limit_for_reach_tool(reach_tool)
            raise TableBDxfExecutionError(
                f"Path {step.get('path_id')} violates {reach_tool} low-Y safety at J7={axis:.1f} mm: "
                f"viewer=({x:.1f}, {y:.1f}) -> local_x={local_x:.1f}. "
                f"When Y < {float(low_y_limit):.1f}, local X must stay >= 0. "
                "Regenerate/approve the toolpath with a legal 7th-axis stop."
            )
        lo, hi = span
        if local_x < lo - 1e-6 or local_x > hi + 1e-6:
            raise TableBDxfExecutionError(
                f"Path {step.get('path_id')} violates {reach_tool} reach at J7={axis:.1f} mm: "
                f"viewer=({x:.1f}, {y:.1f}) -> local_x={local_x:.1f}, allowed=[{lo:.1f}, {hi:.1f}]. "
                "Regenerate/approve the toolpath so the 7th-axis stop is inside the reach window."
            )



def _step_station_sort_key(step: dict[str, Any]) -> tuple[float, int, int, int]:
    if int(step.get("physical_tool") or 0) == 2:
        try:
            side_sequence = int(step.get("side_sequence") or 0)
        except (TypeError, ValueError):
            side_sequence = 0
        return (0.0, side_sequence, 0, int(step.get("_plan_order") or 0))

    axis = step.get("axis7_position_mm")
    try:
        axis_value = float(axis)
    except (TypeError, ValueError):
        axis_value = 0.0
    station = step.get("station_index")
    try:
        station_value = int(station)
    except (TypeError, ValueError):
        station_value = 0
    return (axis_value, station_value, int(step.get("_plan_order") or 0), 0)

def resolve_run_plan(job_id: str, recipe: dict[str, Any]) -> dict[str, Any]:
    """Load the approved toolpath and pair each path with its motion parameters.

    Raises TableBDxfExecutionError when the job has no approved toolpath or the
    toolpath contains no runnable paths.
    """
    try:
        approved = load_approved_toolpath(job_id)
    except FileNotFoundError as error:
        raise TableBDxfExecutionError(
            f"No approved toolpath for job {job_id}. Approve the preview in the 2D DXF viewer first."
        ) from error

    paths = list(approved.get("paths") or [])
    if not paths:
        raise TableBDxfExecutionError(
            f"Approved toolpath for job {job_id} contains no paths."
        )

    # For Pocket ZigZag cycle > 1, alternate the approved selected pattern with the
    # hidden alternate pattern saved by the preview: V/H/V... or H/V/H... .
    paths = _expand_pocket_zigzag_cycles(paths, approved, recipe)
    # Whole-door flat Frame Level uses the same idea for cycle > 1: split by Tool 4
    # reach station, then alternate vertical/horizontal inside that station rectangle.
    # Computed-frame section paths are intentionally excluded.
    paths = _expand_frame_zigzag_cycles(paths, approved, recipe)

    # Tool 2 (side / edge outside) follows the part's outer boundary, which the viewer
    # does not draw, so those paths are derived from the approved outer corner points.
    paths.extend(_build_tool2_paths(approved, recipe))

    steps: list[dict[str, Any]] = []
    missing_params: set[str] = set()
    unknown_tools: set[str] = set()
    skipped: list[dict[str, Any]] = []

    for plan_order, path in enumerate(paths):
        tool = str(path.get("tool", ""))
        spec = _spec_for_tool(tool)
        if not spec:
            unknown_tools.add(tool)
            continue

        step_recipe = _normalize_recipe_entry(path.get("recipe_override")) if isinstance(path.get("recipe_override"), dict) else _recipe_for_tool(tool, recipe)
        viewer_points = _xy_points(path.get("points") or [])
        axis7_position_mm = _float_or_none(path.get("axis7_position_mm"))
        robot_points = _robot_base_points(viewer_points, axis7_position_mm)
        step = {
            "path_id": path.get("path_id"),
            "tool": tool,
            "physical_tool": spec["physical_tool"],
            "operation": path.get("operation") or spec["label"],
            "closed": bool(path.get("closed")),
            "points": viewer_points,
            "viewer_points": viewer_points,
            "robot_base_points": robot_points,
            "station_index": path.get("station_index"),
            "axis7_position_mm": axis7_position_mm,
            "side_label": path.get("side_label"),
            "side_sequence": path.get("side_sequence"),
            "rz_deg": path.get("rz_deg"),
            "motion": _motion_for_tool(tool),
            "recipe": step_recipe,
            "_plan_order": plan_order,
        }

        # An operation the operator did not configure is skipped, exactly like the
        # per-door cycle checks in the Table A reference: no cycle -> no work.
        if step_recipe["cycle"] <= 0:
            skipped.append({
                "path_id": step["path_id"],
                "tool": tool,
                "reason": "cycle is 0 — operation not selected",
            })
            continue

        # A selected operation with no force is an error, not a silent skip: the
        # robot must never run a sanding pass with an unset force.
        if step_recipe["force"] <= 0:
            raise TableBDxfExecutionError(
                f"{step['operation']} is selected ({step_recipe['cycle']} cycle(s)) but its "
                "force is 0. Set a force for that operation before starting the task."
            )

        if not step["motion"]:
            missing_params.add(tool)

        _validate_reach_at_station(step)
        steps.append(step)

    if not steps:
        raise TableBDxfExecutionError(
            "No operation is configured to run. Set force and cycle on at least one "
            "operation that the approved toolpath covers."
        )

    # Group into tool batches so each physical tool is mounted at most once, and
    # order them: pocket edge (T3) -> frame / pocket zigzag (T4) -> 3D (T1) -> side
    # and edge outside (T2).
    batches: list[dict[str, Any]] = []
    for physical_tool in TOOL_BATCH_ORDER:
        batch_steps = sorted(
            (s for s in steps if s["physical_tool"] == physical_tool),
            key=_step_station_sort_key,
        )
        if not batch_steps:
            continue  # nothing selected for this tool — skip the whole batch
        batches.append({
            "physical_tool": physical_tool,
            "steps": batch_steps,
            "path_count": len(batch_steps),
            "point_count": sum(len(s["points"]) for s in batch_steps),
            "operations": sorted({s["operation"] for s in batch_steps}),
        })

    return {
        "job_id": job_id,
        "file_name": approved.get("file_name"),
        "units": approved.get("units", "mm"),
        "approved_at": approved.get("approved_at"),
        "counts": approved.get("counts"),
        "steps": steps,
        "batches": batches,
        "tool_sequence": [b["physical_tool"] for b in batches],
        "skipped_paths": skipped,
        "unknown_tools": sorted(unknown_tools),
        "missing_motion_params": sorted(missing_params),
    }


def run_approved_toolpath(
    job_id: str,
    recipe: dict[str, Any],
    cps: Any = None,
    config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Execute the operator-approved Table B DXF toolpath for a job."""
    plan = resolve_run_plan(job_id, recipe)
    total_points = sum(len(s["points"]) for s in plan["steps"])

    logger.info(
        "[TableB DXF Run] job=%s file=%s paths=%s points=%s tool_sequence=%s dry_run=%s",
        job_id, plan.get("file_name"), len(plan["steps"]), total_points,
        plan["tool_sequence"], DRY_RUN,
    )
    for skip in plan["skipped_paths"]:
        logger.info(
            "[TableB DXF Run] skipping path id=%s tool=%s — %s",
            skip["path_id"], skip["tool"], skip["reason"],
        )
    if plan["unknown_tools"]:
        logger.warning(
            "[TableB DXF Run] approved toolpath contains unmapped tool tag(s): %s",
            plan["unknown_tools"],
        )

    # Tracks what the gripper holds as the plan progresses. On a real run this is
    # re-read from the CI sensors at each batch rather than trusted from memory.
    tool_in_hand: int | None = None

    for batch in plan["batches"]:
        tool_num = batch["physical_tool"]
        logger.info(
            "[TableB DXF Run] === TOOL %s BATCH START === paths=%s points=%s ops=%s",
            tool_num, batch["path_count"], batch["point_count"], batch["operations"],
        )
        # One mount per batch: the tool changer runs only between batches.
        logger.info("[TableB DXF Run] ensure_tool_in_hand(%s)", tool_num)
        logger.info("[TableB DXF Run]   safety: read CI0/CI1/CI2 -> decode tool in hand")
        for change_step in plan_tool_change(tool_in_hand, tool_num):
            logger.info(
                "[TableB DXF Run]   safety: %-28s %s",
                change_step["action"], change_step["detail"],
            )
        tool_in_hand = tool_num

        for index, step in enumerate(batch["steps"], start=1):
            logger.info(
                "[TableB DXF Run] Tool %s path %s/%s id=%s op=%s closed=%s points=%s force=%s cycle=%s station=%s j7=%.3f",
                tool_num, index, batch["path_count"], step["path_id"], step["operation"],
                step["closed"], len(step["points"]),
                step["recipe"]["force"], step["recipe"]["cycle"],
                step.get("station_index"),
                float(step.get("axis7_position_mm") or 0.0),
            )
            # Each configured cycle repeats the whole path, as the Table A cycles do.
            for cycle_index in range(1, step["recipe"]["cycle"] + 1):
                logger.info(
                    "[TableB DXF Run]   cycle %s/%s",
                    cycle_index, step["recipe"]["cycle"],
                )
                for point_index, point in enumerate(step["robot_base_points"]):
                    viewer_point = step["viewer_points"][point_index]
                    logger.info(
                        "[TableB DXF Run]     MoveL %s.%s -> base_x=%.3f y=%.3f (viewer_x=%.3f j7=%.3f mm)",
                        index,
                        point_index,
                        float(point[0]),
                        float(point[1]),
                        float(viewer_point[0]),
                        float(step.get("axis7_position_mm") or 0.0),
                    )

        logger.info("[TableB DXF Run]   safety: move_seventh_to_zero         Rail (J7) returns to 0 mm before tool change / next operation.")
        logger.info("[TableB DXF Run] === TOOL %s BATCH COMPLETE ===", tool_num)

    # End of run: keep the last mounted tool in hand at J7=0. The operator can
    # manually drop it or the next run can switch tools automatically.
    if tool_in_hand is not None:
        logger.info("[TableB DXF Run] === FINISH ===")
        logger.info(
            "[TableB DXF Run]   safety: keep_tool_in_hand            Tool %s remains mounted after J7 returns to 0 mm.",
            tool_in_hand,
        )

    if plan["missing_motion_params"]:
        raise TableBDxfExecutionError(
            "Missing motion parameters for tool(s): "
            + ", ".join(plan["missing_motion_params"])
        )
    if config is None:
        raise TableBDxfExecutionError("Real DXF execution requires runtime config.")

    try:
        from .execution.path_runner import run_robot_plan

        execution = run_robot_plan(cps=cps, config=config, plan=plan)
    except Exception as error:  # noqa: BLE001
        raise TableBDxfExecutionError(str(error)) from error

    return {"dry_run": False, **plan, "execution": execution}




