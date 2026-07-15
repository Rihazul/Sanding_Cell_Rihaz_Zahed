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

# Which UI recipe row drives force/cycle for each toolpath tool. The recipe comes
# from the Table B rows the operator configured (Frame, Pocket ZigZag, 3D, ...).
_TOOL_TO_RECIPE_KEY = {
    "tool_3": "pocketzigzag",
    "tool_4": "pocketzigzag",
    "tool_4_frame": "frame",
    "frame_section": "frame",
    "tool_3d": "3D",
}


class TableBDxfExecutionError(RuntimeError):
    """Raised when an approved toolpath cannot be run."""


def _recipe_for_tool(tool: str, recipe: dict[str, Any]) -> dict[str, Any]:
    """Force/cycle the operator set for the row that drives this tool."""
    key = _TOOL_TO_RECIPE_KEY.get(tool)
    entry = recipe.get(key) if key else None
    if not isinstance(entry, dict):
        return {"force": 0, "cycle": 0}
    return {"force": entry.get("force", 0), "cycle": entry.get("cycle", 0)}


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

    paths = approved.get("paths") or []
    if not paths:
        raise TableBDxfExecutionError(
            f"Approved toolpath for job {job_id} contains no paths."
        )

    steps: list[dict[str, Any]] = []
    missing_params: set[str] = set()
    for path in paths:
        tool = str(path.get("tool", ""))
        params = TOOL_MOTION_PARAMS.get(tool)
        if not params:
            missing_params.add(tool)
        steps.append({
            "path_id": path.get("path_id"),
            "tool": tool,
            "operation": path.get("operation"),
            "closed": bool(path.get("closed")),
            "points": path.get("points") or [],
            "motion": params,
            "recipe": _recipe_for_tool(tool, recipe),
        })

    return {
        "job_id": job_id,
        "file_name": approved.get("file_name"),
        "units": approved.get("units", "mm"),
        "approved_at": approved.get("approved_at"),
        "counts": approved.get("counts"),
        "steps": steps,
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
        "[TableB DXF Run] job=%s file=%s paths=%s points=%s dry_run=%s",
        job_id, plan.get("file_name"), len(plan["steps"]), total_points, DRY_RUN,
    )

    for index, step in enumerate(plan["steps"], start=1):
        logger.info(
            "[TableB DXF Run] path %s/%s id=%s tool=%s op=%s closed=%s points=%s force=%s cycle=%s",
            index, len(plan["steps"]), step["path_id"], step["tool"], step["operation"],
            step["closed"], len(step["points"]),
            step["recipe"]["force"], step["recipe"]["cycle"],
        )
        for point_index, point in enumerate(step["points"]):
            logger.info(
                "[TableB DXF Run]   MoveL %s.%s -> x=%.3f y=%.3f (mm)",
                index, point_index, float(point[0]), float(point[1]),
            )

    if DRY_RUN:
        logger.info(
            "[TableB DXF Run] DRY RUN complete — no motion sent. missing_motion_params=%s",
            plan["missing_motion_params"],
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
