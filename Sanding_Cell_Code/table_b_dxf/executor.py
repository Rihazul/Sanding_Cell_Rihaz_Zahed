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
from typing import Any

from .jobs import load_approved_toolpath
from .execution.pocket_zigzag_cycles import (
    build_tool4_rectangular_alternating_cycle_paths,
    expand_pocket_zigzag_cycles as _expand_pocket_zigzag_cycles,
)
from .tool_reach import TOOL_REACH, reach_span_at_y

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
    raw_selected = str((settings or {}).get("frame_zigzag_orientation") or "vertical").lower()
    selected = "rectspiral" if raw_selected == "rectspiral" else "horizontal" if raw_selected == "horizontal" else "vertical"
    alternate = None if selected == "rectspiral" else ("vertical" if selected == "horizontal" else "horizontal")

    return build_tool4_rectangular_alternating_cycle_paths(
        source_paths=frame_paths,
        cycles=cycles,
        force=frame_recipe["force"],
        step_mm=step_mm,
        selected=selected,
        alternate=alternate,
        path_prefix="frame_zigzag",
        operation_label="Frame zigzag continuous",
    )


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



def _tool4_operation_priority(step: dict[str, Any]) -> int:
    """Stable order for Tool 4 paths that share the same J7 stop."""
    tool = str(step.get("tool") or "")
    text = " ".join(
        str(step.get(key) or "")
        for key in ("operation", "path_id", "split_from_path_id")
    ).lower()
    if tool in ("tool_4_frame", "frame_section") or "frame" in text:
        return 0
    if tool == "tool_4" or "zigzag" in text or "pocket" in text:
        return 1
    return 2


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


def _tool4_increasing_j7_sort_key(step: dict[str, Any]) -> tuple[float, int, int, int]:
    """Force Tool 4 frame and pocket work to run by increasing rail position."""
    axis = step.get("axis7_position_mm")
    try:
        axis_value = float(axis)
    except (TypeError, ValueError):
        axis_value = float("inf")
    station = step.get("station_index")
    try:
        station_value = int(station)
    except (TypeError, ValueError):
        station_value = 0
    return (
        round(axis_value, 3),
        station_value,
        _tool4_operation_priority(step),
        int(step.get("_plan_order") or 0),
    )


def _sort_batch_steps_for_tool(physical_tool: int, steps: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if int(physical_tool) == 4:
        return sorted(steps, key=_tool4_increasing_j7_sort_key)
    return sorted(steps, key=_step_station_sort_key)


def _format_batch_j7_sequence(steps: list[dict[str, Any]]) -> str:
    parts: list[str] = []
    for step in steps:
        axis = step.get("axis7_position_mm")
        try:
            axis_text = f"{float(axis):.1f}"
        except (TypeError, ValueError):
            axis_text = "?"
        operation = str(step.get("operation") or step.get("tool") or "path")
        parts.append(f"{axis_text}:{operation}")
    return " -> ".join(parts)

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
        batch_steps = _sort_batch_steps_for_tool(
            physical_tool,
            [s for s in steps if s["physical_tool"] == physical_tool],
        )
        if not batch_steps:
            continue  # nothing selected for this tool — skip the whole batch
        logger.info(
            "[TableB DXF Run] Tool %s sorted J7 sequence: %s",
            physical_tool,
            _format_batch_j7_sequence(batch_steps),
        )
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





