from __future__ import annotations

import json
import logging
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

logger = logging.getLogger(__name__)

TABLE_B_DXF_ROOT = Path(__file__).resolve().parent
UPLOADS_DIR = TABLE_B_DXF_ROOT / "uploads"
PARSED_DIR = TABLE_B_DXF_ROOT / "parsed"
MAPPINGS_DIR = TABLE_B_DXF_ROOT / "mappings"
TOOLPATHS_DIR = TABLE_B_DXF_ROOT / "toolpaths"
_JOB_ROOTS = (UPLOADS_DIR, PARSED_DIR, MAPPINGS_DIR, TOOLPATHS_DIR)

# Only the most recent jobs are kept on disk. The cell works one DXF at a time, so
# older jobs are just dead weight; a small buffer covers quick re-uploads.
MAX_KEPT_JOBS = 8


def _ensure_base_folders() -> None:
    for folder in (UPLOADS_DIR, PARSED_DIR, MAPPINGS_DIR, TOOLPATHS_DIR):
        folder.mkdir(parents=True, exist_ok=True)
        logger.info("Table B DXF Assisted folder ready: %s", folder)


def _validate_job_id(job_id: str) -> None:
    if not job_id or any(char in job_id for char in ("/", "\\", "..")):
        raise ValueError("Invalid Table B DXF job_id")


def _ensure_job_folders(job_id: str) -> None:
    _validate_job_id(job_id)
    for folder in (
        UPLOADS_DIR / job_id,
        PARSED_DIR / job_id,
        MAPPINGS_DIR / job_id,
        TOOLPATHS_DIR / job_id,
    ):
        folder.mkdir(parents=True, exist_ok=True)
        logger.info("Table B DXF Assisted job folder ready: %s", folder)


def prune_old_table_b_dxf_jobs(keep: int = MAX_KEPT_JOBS) -> None:
    """Delete all but the most recent `keep` jobs across every job folder.

    Each upload mints a new job; without this the uploads/parsed/mappings/toolpaths
    trees grow without bound. Newest is decided by the upload folder's mtime. Best
    effort — a cleanup hiccup must never fail an upload.
    """
    try:
        job_dirs = [d for d in UPLOADS_DIR.iterdir() if d.is_dir()]
    except FileNotFoundError:
        return

    job_dirs.sort(key=lambda d: d.stat().st_mtime, reverse=True)
    for stale in job_dirs[keep:]:
        job_id = stale.name
        for root in _JOB_ROOTS:
            target = root / job_id
            if target.exists():
                shutil.rmtree(target, ignore_errors=True)
        logger.info("Pruned old Table B DXF job: %s", job_id)


def create_table_b_dxf_job() -> dict[str, Any]:
    """Create an isolated Table B DXF Assisted job workspace."""
    _ensure_base_folders()

    job_id = uuid4().hex
    _ensure_job_folders(job_id)

    metadata = {
        "job_id": job_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "mode": "cad_assisted_dxf",
        "status": "created",
    }
    save_job_metadata(job_id, metadata)
    logger.info("Created Table B DXF Assisted job: %s", job_id)

    # Drop stale jobs from earlier uploads so disk usage stays bounded.
    try:
        prune_old_table_b_dxf_jobs()
    except Exception as error:  # noqa: BLE001 - cleanup must never break an upload
        logger.warning("Table B DXF Assisted job prune failed: %s", error)

    return {
        "job_id": job_id,
        "paths": get_table_b_dxf_job_paths(job_id),
        "metadata": metadata,
    }


def get_table_b_dxf_job_paths(job_id: str) -> dict[str, str]:
    """Return the expected file locations for a Table B DXF Assisted job."""
    _validate_job_id(job_id)

    return {
        "upload_dir": str(UPLOADS_DIR / job_id),
        "parsed_dir": str(PARSED_DIR / job_id),
        "mappings_dir": str(MAPPINGS_DIR / job_id),
        "toolpaths_dir": str(TOOLPATHS_DIR / job_id),
        "source_dxf": str(UPLOADS_DIR / job_id / "source.dxf"),
        "parsed_loops": str(PARSED_DIR / job_id / "parsed_loops.json"),
        "mapped_regions": str(MAPPINGS_DIR / job_id / "mapped_regions.json"),
        "frame_geometry": str(TOOLPATHS_DIR / job_id / "frame_geometry.json"),
        "approved_toolpath": str(TOOLPATHS_DIR / job_id / "approved_toolpath.json"),
        "job_metadata": str(MAPPINGS_DIR / job_id / "job_metadata.json"),
    }


def save_job_metadata(job_id: str, metadata: dict[str, Any]) -> None:
    """Persist metadata for a Table B DXF Assisted job."""
    _ensure_job_folders(job_id)
    paths = get_table_b_dxf_job_paths(job_id)
    metadata_path = Path(paths["job_metadata"])
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    logger.info("Saved Table B DXF Assisted job metadata: %s", metadata_path)


def save_approved_toolpath(job_id: str, payload: dict[str, Any]) -> str:
    """Persist the operator-approved toolpath + region corner points as JSON.

    Overwrites any previous file for the job so a re-approval always replaces the
    earlier choice — the operation runs the operator's latest decision.
    """
    _ensure_job_folders(job_id)
    path = Path(get_table_b_dxf_job_paths(job_id)["approved_toolpath"])
    record = {
        **payload,
        "approved_at": datetime.now(timezone.utc).isoformat(),
    }
    path.write_text(json.dumps(record, indent=2, sort_keys=False), encoding="utf-8")
    logger.info("Saved Table B DXF approved toolpath: %s", path)
    return str(path)


def load_approved_toolpath(job_id: str) -> dict[str, Any]:
    """Load the operator-approved toolpath JSON for a job."""
    path = Path(get_table_b_dxf_job_paths(job_id)["approved_toolpath"])
    if not path.exists():
        raise FileNotFoundError(f"Table B DXF approved toolpath not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def load_job_metadata(job_id: str) -> dict[str, Any]:
    """Load metadata for a Table B DXF Assisted job."""
    metadata_path = Path(get_table_b_dxf_job_paths(job_id)["job_metadata"])
    if not metadata_path.exists():
        raise FileNotFoundError(f"Table B DXF job metadata not found: {metadata_path}")

    logger.info("Loaded Table B DXF Assisted job metadata: %s", metadata_path)
    return json.loads(metadata_path.read_text(encoding="utf-8"))
