from __future__ import annotations

import logging
import json
from datetime import datetime, timezone
from pathlib import Path

from flask import Blueprint, jsonify, request
from werkzeug.utils import secure_filename

from .jobs import (
    create_table_b_dxf_job,
    get_table_b_dxf_job_paths,
    save_approved_toolpath,
    save_job_metadata,
)
from .parser import parse_dxf_loops
from .surface_checker import check_selected_lines_closed
from .surface_detector import detect_selected_loops
from .frame.sections import compute_frame_sections_and_toolpaths
from .frame.paths import chain_computed_frame_paths_by_station
from .frame.zigzag import compute_frame_zigzag_fill
from .tool_reach import attach_station_plan, plan_closed_loop_best_start, station_window_for_path

logger = logging.getLogger(__name__)

table_b_dxf_bp = Blueprint("table_b_dxf", __name__, url_prefix="/api/table-b-dxf")

ALLOWED_DXF_EXTENSIONS = {".dxf"}


def _path_debug_summary(toolpaths):
    summary = {}
    for path in toolpaths or []:
        key = str(path.get("source_section_strategy") or path.get("path_strategy") or "unknown")
        entry = summary.setdefault(key, {"count": 0, "split_count": 0, "point_counts": []})
        entry["count"] += 1
        if path.get("reach_split_count"):
            entry["split_count"] += 1
        entry["point_counts"].append(len(path.get("points") or []))
    return summary



def _reach_plan_from_tagged_toolpaths(toolpaths, tool="tool_4"):
    """Build a reach_plan that matches the final executable toolpath list."""
    stations_by_index = {}
    unreachable = []
    for path_index, path in enumerate(toolpaths or []):
        station_index = path.get("station_index")
        if station_index is None or path.get("reach_unreachable"):
            unreachable.append(path_index)
            continue
        axis = path.get("axis7_position_mm")
        entry = stations_by_index.setdefault(
            int(station_index),
            {"position": float(axis or 0.0), "path_indices": []},
        )
        if axis is not None:
            entry["position"] = float(axis)
        entry["path_indices"].append(path_index)

    stations = [stations_by_index[key] for key in sorted(stations_by_index)]
    total_travel = 0.0
    previous = 0.0
    for station in stations:
        position = float(station.get("position") or 0.0)
        total_travel += abs(position - previous)
        previous = position

    return {
        "tool": tool,
        "stations": stations,
        "station_count": len(stations),
        "total_travel_mm": total_travel,
        "unreachable_path_indices": unreachable,
    }

def _write_frame_toolpath_debug(job_id, payload, result, before_reach_toolpaths, after_reach_toolpaths):
    try:
        debug_dir = Path(__file__).resolve().parent / "debug"
        debug_dir.mkdir(parents=True, exist_ok=True)
        sections = result.get("sections") or []
        data = {
            "job_id": job_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "payload_counts": {
                "outline_points": len(payload.get("outline_polygon") or []),
                "pocket_polygons": len(payload.get("pocket_polygons") or []),
                "surface3d_polygons": len(payload.get("surface3d_polygons") or []),
            },
            "sections_by_strategy": _path_debug_summary([
                {"source_section_strategy": section.get("section_strategy"), "points": section.get("corners") or []}
                for section in sections
            ]),
            "before_reach_summary": _path_debug_summary(before_reach_toolpaths),
            "after_reach_summary": _path_debug_summary(after_reach_toolpaths),
            "uncovered_section_ids": result.get("uncovered_section_ids") or [],
            "rings": result.get("rings") or [],
            "sections": sections,
            "toolpaths_before_reach": before_reach_toolpaths,
            "toolpaths_after_reach": after_reach_toolpaths,
        }
        (debug_dir / "last_frame_toolpaths.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    except Exception as debug_error:  # noqa: BLE001 - debug must not break preview
        logger.warning("Table B DXF frame-toolpaths debug dump skipped: %s", debug_error)


def _get_uploaded_file():
    if "file" not in request.files:
        return None, ("Missing upload file. Send the DXF file using form field 'file'.", 400)

    uploaded_file = request.files["file"]
    if not uploaded_file or not uploaded_file.filename:
        return None, ("Missing upload filename.", 400)

    return uploaded_file, None


def _validate_dxf_filename(filename: str) -> tuple[str | None, str | None]:
    safe_name = secure_filename(filename)
    extension = Path(safe_name).suffix.lower()
    logger.info("Validating Table B DXF Assisted upload: filename=%s extension=%s", safe_name, extension)

    if extension not in ALLOWED_DXF_EXTENSIONS:
        return None, "Unsupported file type. Upload only .dxf files."

    logger.info("Table B DXF Assisted upload validation passed: %s", safe_name)
    return safe_name, None


@table_b_dxf_bp.post("/upload")
def upload_table_b_dxf_file():
    logger.info("Table B DXF Assisted upload received")

    uploaded_file, upload_error = _get_uploaded_file()
    if upload_error:
        message, status_code = upload_error
        logger.warning("Table B DXF Assisted upload rejected: %s", message)
        return jsonify({"success": False, "error": message}), status_code

    safe_name, validation_error = _validate_dxf_filename(uploaded_file.filename)
    if validation_error:
        logger.warning("Table B DXF Assisted upload validation failed: %s", validation_error)
        return jsonify({"success": False, "error": validation_error}), 400

    # Name the job after the uploaded file + a timestamp so operators can recognise
    # it in the UI, the logs and the job folders.
    job = create_table_b_dxf_job(safe_name)
    job_id = job["job_id"]
    logger.info("Table B DXF Assisted job_id created for upload: %s", job_id)

    paths = get_table_b_dxf_job_paths(job_id)
    source_path = Path(paths["source_dxf"])
    uploaded_file.save(source_path)
    logger.info("Table B DXF Assisted upload saved: %s", source_path)

    metadata = {
        **job["metadata"],
        "status": "uploaded",
        "uploaded_at": datetime.now(timezone.utc).isoformat(),
        "original_filename": uploaded_file.filename,
        "stored_filename": "source.dxf",
        "source_file": str(source_path),
    }
    save_job_metadata(job_id, metadata)

    return jsonify({
        "success": True,
        "job_id": job_id,
        "status": "uploaded",
    })


@table_b_dxf_bp.get("/parsed/<job_id>")
def get_table_b_dxf_parsed_loops(job_id: str):
    logger.info("Table B DXF Assisted parse requested: %s", job_id)
    try:
        result = parse_dxf_loops(job_id)
        return jsonify({"success": True, **result})
    except FileNotFoundError as error:
        logger.warning("Table B DXF Assisted parse rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "source_dxf_missing",
            "message": str(error),
        }), 404
    except ValueError as error:
        logger.warning("Table B DXF Assisted parse rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_dxf",
            "message": str(error),
        }), 400


@table_b_dxf_bp.post("/check-lines-closed/<job_id>")
def check_table_b_dxf_lines_closed(job_id: str):
    logger.info("Table B DXF Assisted check-lines-closed requested: %s", job_id)
    payload = request.get_json(silent=True) or {}
    selected = payload.get("selected_entity_ids")
    if not isinstance(selected, list):
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_payload",
            "message": "selected_entity_ids must be a list of entity ids.",
        }), 400

    try:
        result = check_selected_lines_closed(job_id, selected)
        return jsonify({"success": True, "job_id": job_id, **result})
    except FileNotFoundError as error:
        logger.warning("Table B DXF Assisted check-lines-closed rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "parsed_loops_missing",
            "message": str(error),
        }), 404
    except ValueError as error:
        logger.warning("Table B DXF Assisted check-lines-closed rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_check_request",
            "message": str(error),
        }), 400


@table_b_dxf_bp.post("/approved-toolpath/<job_id>")
def save_table_b_dxf_approved_toolpath(job_id: str):
    """Persist the operator-approved toolpath + region corners as JSON.

    Re-approving overwrites the same file so the operation always reflects the
    operator's latest choice.
    """
    logger.info("Table B DXF Assisted approved-toolpath save requested: %s", job_id)
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_payload",
            "message": "Approved toolpath payload must be a JSON object.",
        }), 400

    try:
        saved_path = save_approved_toolpath(job_id, payload)
        return jsonify({"success": True, "job_id": job_id, "saved_path": saved_path})
    except ValueError as error:
        logger.warning("Table B DXF Assisted approved-toolpath save rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_job_id",
            "message": str(error),
        }), 400


@table_b_dxf_bp.post("/detect-loops/<job_id>")
def detect_table_b_dxf_loops(job_id: str):
    logger.info("Table B DXF Assisted detect-loops requested: %s", job_id)
    payload = request.get_json(silent=True) or {}
    selected = payload.get("selected_entity_ids")
    if not isinstance(selected, list):
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_payload",
            "message": "selected_entity_ids must be a list of entity ids.",
        }), 400

    try:
        result = detect_selected_loops(job_id, selected)
        return jsonify({"success": True, "job_id": job_id, **result})
    except FileNotFoundError as error:
        logger.warning("Table B DXF Assisted detect-loops rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "parsed_loops_missing",
            "message": str(error),
        }), 404
    except ValueError as error:
        logger.warning("Table B DXF Assisted detect-loops rejected: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "invalid_detect_request",
            "message": str(error),
        }), 400


@table_b_dxf_bp.post("/frame-toolpaths/<job_id>")
def compute_table_b_dxf_frame_toolpaths(job_id: str):
    """Compute frame area, frame sections, reach chunks, and toolpaths in backend."""
    logger.info("Table B DXF Assisted frame-toolpaths requested: %s", job_id)
    payload = request.get_json(silent=True) or {}
    try:
        result = compute_frame_sections_and_toolpaths(
            payload.get("outline_polygon"),
            payload.get("pocket_polygons") or [],
            payload.get("surface3d_polygons") or [],
            pass_width_mm=float(payload.get("pass_width_mm") or 75.0),
            offset_mm=float(payload.get("offset_mm") or 50.0),
            overlap_mm=float(payload.get("overlap_mm") or 0.0),
            reach_x_mm=float(payload.get("reach_x_mm") or 515.0),
            reach_y_mm=float(payload.get("reach_y_mm") or 750.0),
        )
        chunks = result.get("chunks") or []
        toolpaths = result.get("toolpaths") or []
        toolpaths_before_reach = json.loads(json.dumps(toolpaths))

        # Plan 7th-axis stations for the frame tool and tag every toolpath with the station
        # that runs it. The viewer colours by station so the operator can see which passes
        # happen together without repositioning, and the executor gets the axis positions
        # alongside the points it must trace.
        #
        # ORDER MATTERS. First tag/split by 7th-axis reach without changing the clean
        # generated frame paths. Then connect safe same-station computed-frame rails. Wide
        # frame zigzags and diagonal centerline bands may join to neighboring single-pass
        # bands when a short connector stays inside frame_area; curved fills stay separate.
        try:
            result["reach_plan"] = attach_station_plan(toolpaths, tool="tool_4", chain=False)
            toolpaths = chain_computed_frame_paths_by_station(
                toolpaths,
                result.get("rings") or [],
                max_connector_mm=max(float(payload.get("pass_width_mm") or 75.0) * 1.25, 120.0),
            )
            result["toolpaths"] = toolpaths
            result["reach_plan"] = attach_station_plan(toolpaths, tool="tool_4", chain=False)
            result["toolpaths"] = toolpaths
        except Exception as reach_error:  # noqa: BLE001 - a preview must not fail on this
            logger.warning("Table B DXF frame-toolpaths: reach planning skipped: %s", reach_error)

        _write_frame_toolpath_debug(job_id, payload, result, toolpaths_before_reach, toolpaths)

        logger.info(
            "Table B DXF frame-toolpaths result: outlinePts=%s pockets=%s surface3d=%s sections=%s chunks=%s curvedChunks=%s paths=%s strategies=%s",
            len(payload.get("outline_polygon") or []),
            len(payload.get("pocket_polygons") or []),
            len(payload.get("surface3d_polygons") or []),
            len(result.get("sections") or []),
            len(chunks),
            sum(1 for chunk in chunks if chunk.get("curved")),
            len(toolpaths),
            sorted({path.get("path_strategy") for path in toolpaths if path.get("path_strategy")}),
        )
        return jsonify({"success": True, "job_id": job_id, **result})
    except Exception as error:  # noqa: BLE001 — never fail a preview on a geometry hiccup
        logger.exception("Table B DXF Assisted frame-toolpaths failed: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "frame_toolpaths_failed",
            "message": str(error),
        }), 500


@table_b_dxf_bp.post("/plan-reach/<job_id>")
def plan_table_b_dxf_reach(job_id: str):
    """Plan 7th-axis stations for ANY tool's toolpaths (pocket edge, zigzag, 3D, frame).

    The frontend generates pocket/3D toolpaths locally, so they never passed through reach
    planning and showed no splits. This endpoint takes whatever paths a tool must run and
    returns them tagged with the station that executes each one, splitting any pass the arm
    cannot reach in a single move.

    Body: { tool: "tool_3"|"tool_1"|"tool_4", toolpaths: [ {path_id, points:[[x,y],...]} ] }
    """
    logger.info("Table B DXF Assisted plan-reach requested: %s", job_id)
    payload = request.get_json(silent=True) or {}
    tool = str(payload.get("tool") or "tool_4")
    toolpaths = payload.get("toolpaths") or []
    if not isinstance(toolpaths, list):
        return jsonify({
            "success": False, "job_id": job_id, "status": "invalid_payload",
            "message": "toolpaths must be a list of {path_id, points}.",
        }), 400
    try:
        paths = [dict(tp) for tp in toolpaths if isinstance(tp, dict)]

        # Closed pocket-edge and 3D contour loops may be too large for one J7 station.
        # Keep the normal bottom-right start only when the full loop fits. Otherwise,
        # choose the best reachable loop break and return open arcs that cover the loop
        # once, without forcing every split to start at the original corner.
        if tool in {"tool_1", "tool_3"}:
            planned_paths = []
            for path in paths:
                pts = path.get("points") or []
                if not path.get("closed") or len(pts) < 3 or station_window_for_path(tool, pts) is not None:
                    planned_paths.append(path)
                    continue
                loop_plan = plan_closed_loop_best_start(tool, pts)
                arcs = loop_plan.get("arcs") or []
                if not arcs:
                    planned_paths.append(path)
                    continue
                for arc_index, arc in enumerate(arcs):
                    clone = dict(path)
                    clone["points"] = arc.get("points") or []
                    clone["closed"] = False
                    clone["path_id"] = f"{path.get('path_id', 'loop')}_arc{arc_index}"
                    clone["split_from_path_id"] = path.get("path_id")
                    clone["reach_split_index"] = arc_index
                    clone["reach_split_count"] = len(arcs)
                    clone["closed_loop_start_index"] = loop_plan.get("start_index")
                    planned_paths.append(clone)
            paths = planned_paths

        plan = attach_station_plan(paths, tool=tool)
        return jsonify({
            "success": True, "job_id": job_id, "tool": tool,
            "toolpaths": paths, "reach_plan": plan,
        })
    except Exception as error:  # noqa: BLE001 - a preview must never fail on planning
        logger.exception("Table B DXF Assisted plan-reach failed: %s", error)
        return jsonify({
            "success": False, "job_id": job_id,
            "status": "plan_reach_failed", "message": str(error),
        }), 500


@table_b_dxf_bp.post("/frame-zigzag/<job_id>")
def compute_table_b_dxf_frame_zigzag(job_id: str):
    """Zigzag fill of the whole frame surface, clipped to the door outline (curve-aware).

    For 'Frame Level on the whole door': dense serpentine passes filling
    outer − pockets − 3D, each clipped to the real polygon so they follow a curved edge.
    """
    logger.info("Table B DXF Assisted frame-zigzag requested: %s", job_id)
    payload = request.get_json(silent=True) or {}
    try:
        result = compute_frame_zigzag_fill(
            payload.get("outline_polygon"),
            payload.get("pocket_polygons") or [],
            payload.get("surface3d_polygons") or [],
            pass_width_mm=float(payload.get("pass_width_mm") or 75.0),
            overlap_mm=float(payload.get("overlap_mm") or 0.0),
        )
        try:
            toolpaths = result.get("toolpaths") or []
            result["reach_plan"] = attach_station_plan(toolpaths, tool="tool_4")
            result["toolpaths"] = toolpaths
        except Exception as reach_error:  # noqa: BLE001 - preview should still render
            logger.warning("Table B DXF frame-zigzag: reach planning skipped: %s", reach_error)
        return jsonify({"success": True, "job_id": job_id, **result})
    except Exception as error:  # noqa: BLE001 — never fail a preview on a geometry hiccup
        logger.exception("Table B DXF Assisted frame-zigzag failed: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "frame_zigzag_failed",
            "message": str(error),
        }), 500
