from __future__ import annotations

import logging
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
from .frame_sections import compute_frame_sections_and_toolpaths, compute_frame_zigzag_fill

logger = logging.getLogger(__name__)

table_b_dxf_bp = Blueprint("table_b_dxf", __name__, url_prefix="/api/table-b-dxf")

ALLOWED_DXF_EXTENSIONS = {".dxf"}


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
        return jsonify({"success": True, "job_id": job_id, **result})
    except Exception as error:  # noqa: BLE001 — never fail a preview on a geometry hiccup
        logger.exception("Table B DXF Assisted frame-zigzag failed: %s", error)
        return jsonify({
            "success": False,
            "job_id": job_id,
            "status": "frame_zigzag_failed",
            "message": str(error),
        }), 500
