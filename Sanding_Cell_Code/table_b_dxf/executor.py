from __future__ import annotations

"""Run an operator-approved Table B DXF toolpath on the robot.

Table B no longer uses the legacy model presets (Table<N>Model / model<N>cycle).
A run is fully defined by the approved toolpath JSON the 2D DXF workspace saved for
a job, plus the force/cycle recipe the operator set in the UI.

SAFETY / STATUS: this module is currently DRY-RUN ONLY. It resolves everything a
real run needs and logs each MoveL it *would* send, but issues no motion commands.
Real motion stays off until the per-tool motion parameters below are filled in and
DRY_RUN is turned off deliberately.
"""

import logging
from typing import Any

from .jobs import load_approved_toolpath

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Dry-run switch. While True, nothing moves: the run is fully resolved and logged
# so the coordinates and ordering can be verified against the viewer first.
# ---------------------------------------------------------------------------
DRY_RUN = True

# ---------------------------------------------------------------------------
# TODO(motion params): fill these in before enabling real motion.
#
# The approved toolpath carries only [x, y] millimetre points in the machine frame
# plus a tool name per path. Everything else — Z height, tool orientation, TCP/UCS
# and the force applied — is machine configuration and must be supplied here.
#
# `tool` values that appear in approved_toolpath.json paths:
#   tool_3 | tool_4 | tool_3d | tool_4_frame | frame_section
#
# For each, specify:
#   tcp    : TCP name from configs/config.yaml (e.g. "TCP_tool1")
#   ucs    : UCS/plane name for Table B (e.g. "Plane_2")
#   z_mm   : sanding Z height in the UCS
#   retract_mm : safe Z to travel between paths
#   rx, ry, rz : fixed tool orientation
# Force/cycle come from the operator's recipe (see _recipe_for_tool), not from here.
# ---------------------------------------------------------------------------
TOOL_MOTION_PARAMS: dict[str, dict[str, Any]] = {
    # "tool_3": {"tcp": "", "ucs": "", "z_mm": 0.0, "retract_mm": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0},
    # "tool_4": {...},
    # "tool_3d": {...},
    # "tool_4_frame": {...},
    # "frame_section": {...},
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


# Tool 2 (side / edge outside) follows the part's outer boundary rather than a
# toolpath drawn in the viewer, so its path is derived here from the approved outer
# corner points: the rectangle from the origin (0,0) to the part's max (X, Y).
_OUTER_CORNER_LABEL = "door outer corners"


def _outer_corner_points(approved: dict[str, Any]) -> list[list[float]] | None:
    """The part's 4 outer corner points (mm) from the approved toolpath."""
    for region in approved.get("regions") or []:
        for shape in region.get("corner_shapes") or []:
            if str(shape.get("label", "")).strip().lower() == _OUTER_CORNER_LABEL:
                points = shape.get("points") or []
                if len(points) >= 4:
                    return [[float(p[0]), float(p[1])] for p in points]
    return None


def _build_tool2_paths(approved: dict[str, Any], recipe: dict[str, Any]) -> list[dict[str, Any]]:
    """Synthesize the Tool 2 side / edge-outside paths from the outer corners.

    The 2D viewer does not draw Tool 2 toolpaths: the side and outside edge follow
    the part's outer boundary, so the path is the closed outer rectangle. Both
    operations run the same outline and are only emitted when the operator
    configured them.
    """
    corners = _outer_corner_points(approved)
    if not corners:
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
        paths.append({
            "path_id": f"outer_{recipe_key}",
            "tool": f"tool_2_{recipe_key}",
            "operation": label,
            "closed": True,
            "points": corners,
        })
    return paths


class TableBDxfExecutionError(RuntimeError):
    """Raised when an approved toolpath cannot be run."""


def _recipe_for_tool(tool: str, recipe: dict[str, Any]) -> dict[str, Any]:
    """Force/cycle the operator set for the row that drives this tool."""
    spec = _spec_for_tool(tool)
    entry = recipe.get(spec["recipe_key"]) if spec else None
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


def _robot_base_points(viewer_points: list[list[float]], axis7_position_mm: float | None) -> list[list[float]]:
    """Convert viewer/global UCS points into the robot-base frame at one J7 stop.

    The 2D viewer is in the fixed Table B UCS. Moving the 7th axis shifts the robot
    base along UCS X, so the robot must receive local X = viewer X - J7 position.
    Y is unchanged because the rail only translates the base in X.
    """
    if axis7_position_mm is None:
        return [list(point) for point in viewer_points]
    return [[point[0] - axis7_position_mm, point[1]] for point in viewer_points]


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

    # Tool 2 (side / edge outside) follows the part's outer boundary, which the viewer
    # does not draw, so those paths are derived from the approved outer corner points.
    paths.extend(_build_tool2_paths(approved, recipe))

    steps: list[dict[str, Any]] = []
    missing_params: set[str] = set()
    unknown_tools: set[str] = set()
    skipped: list[dict[str, Any]] = []

    for path in paths:
        tool = str(path.get("tool", ""))
        spec = _spec_for_tool(tool)
        if not spec:
            unknown_tools.add(tool)
            continue

        step_recipe = _recipe_for_tool(tool, recipe)
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
            "motion": TOOL_MOTION_PARAMS.get(tool),
            "recipe": step_recipe,
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
        batch_steps = [s for s in steps if s["physical_tool"] == physical_tool]
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


def run_approved_toolpath(job_id: str, recipe: dict[str, Any], cps: Any = None) -> dict[str, Any]:
    """Execute (or dry-run) the approved toolpath for a job.

    While DRY_RUN is True this logs every MoveL that would be sent and returns the
    resolved plan without touching the robot.
    """
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

        logger.info("[TableB DXF Run] === TOOL %s BATCH COMPLETE ===", tool_num)

    # End of run: return the mounted tool and go home, as the Table A reference does.
    if tool_in_hand is not None:
        logger.info("[TableB DXF Run] === FINISH ===")
        for finish_step in (
            {"action": "move_to_safe_point", "detail": "Arm to safe point before final tool return."},
            {"action": "move_seventh_to_tool_station", "detail": "Rail (J7) to tool station."},
            {"action": "wait_for_j7_idle", "detail": "Rail must be idle before the changer engages."},
            {"action": "drop_tool", "detail": f"Return tool {tool_in_hand}."},
        ):
            logger.info(
                "[TableB DXF Run]   safety: %-28s %s",
                finish_step["action"], finish_step["detail"],
            )

    if DRY_RUN:
        logger.info(
            "[TableB DXF Run] DRY RUN complete — no motion sent. "
            "tool_sequence=%s missing_motion_params=%s",
            plan["tool_sequence"], plan["missing_motion_params"],
        )
        return {"dry_run": True, **plan}

    # Real motion is intentionally not implemented yet: TOOL_MOTION_PARAMS must be
    # supplied and reviewed first, otherwise the robot would be commanded with
    # unknown Z heights and orientations.
    if plan["missing_motion_params"]:
        raise TableBDxfExecutionError(
            "Missing motion parameters for tool(s): "
            + ", ".join(plan["missing_motion_params"])
        )
    raise TableBDxfExecutionError(
        "Real DXF execution is not enabled yet. Fill in TOOL_MOTION_PARAMS and add the "
        "motion calls before turning DRY_RUN off."
    )
