from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any

from .jobs import get_table_b_dxf_job_paths

logger = logging.getLogger(__name__)

# A closed surface needs at least this many connected edges.
MIN_CLOSED_EDGES = 3


def _dist(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _close(a: list[float], b: list[float], tol: float) -> bool:
    return _dist(a, b) <= tol


def _bbox(points: list[list[float]]) -> dict[str, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {"min_x": min(xs), "min_y": min(ys), "max_x": max(xs), "max_y": max(ys)}


def _area(points: list[list[float]]) -> float:
    n = len(points)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        total += (x1 * y2) - (x2 * y1)
    return abs(total) / 2.0


def _relative_tolerance(points: list[list[float]]) -> float:
    """Coordinate tolerance scaled to the size of the selected geometry."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    return max(diag * 1e-3, 1e-6)


def _chain_segments(segments: list[list[list[float]]], tol: float):
    """Greedily order/orient segments into a single chain by matching endpoints.

    Returns (chain_points, closed, start_point, open_end_point, leftover_indices).
    """
    remaining = list(range(len(segments)))
    first = remaining.pop(0)
    chain: list[list[float]] = [list(pt) for pt in segments[first]]
    start = chain[0]
    end = chain[-1]

    progress = True
    while remaining and progress:
        progress = False
        for idx in list(remaining):
            seg = segments[idx]
            seg_start = seg[0]
            seg_end = seg[-1]
            if _close(end, seg_start, tol):
                chain.extend([list(pt) for pt in seg[1:]])
                end = seg[-1]
                remaining.remove(idx)
                progress = True
                break
            if _close(end, seg_end, tol):
                reversed_seg = list(reversed(seg))
                chain.extend([list(pt) for pt in reversed_seg[1:]])
                end = seg[0]
                remaining.remove(idx)
                progress = True
                break

    closed = (not remaining) and _close(end, start, tol)
    # Drop the duplicated closing vertex if present.
    if closed and len(chain) >= 2 and _close(chain[0], chain[-1], tol):
        chain = chain[:-1]

    return chain, closed, start, end, remaining


def check_selected_lines_closed(
    job_id: str,
    selected_entity_ids: list[str],
    tolerance: float | None = None,
) -> dict[str, Any]:
    """Check whether the selected open guide lines chain into a closed loop.

    Reads parsed_loops.json, matches the selected ids against the open
    line_entity objects, and tries to order them end-to-end. Does NOT save a
    surface or generate toolpaths — this is a pure check.
    """
    paths = get_table_b_dxf_job_paths(job_id)
    parsed_path = Path(paths["parsed_loops"])
    if not parsed_path.exists():
        raise FileNotFoundError(
            f"parsed_loops.json not found for job {job_id}. Parse the DXF first."
        )

    parsed = json.loads(parsed_path.read_text(encoding="utf-8"))
    open_paths = parsed.get("open_paths", [])
    by_id = {path["entity_id"]: path for path in open_paths}

    selected: list[dict[str, Any]] = []
    missing_ids: list[str] = []
    for entity_id in selected_entity_ids:
        path = by_id.get(entity_id)
        if path and len(path.get("points", [])) >= 2:
            selected.append(path)
        else:
            missing_ids.append(entity_id)

    segments = [[list(pt) for pt in path["points"]] for path in selected]

    logger.info(
        "Table B DXF Assisted check-lines-closed: job_id=%s requested=%s valid_lines=%s missing=%s",
        job_id,
        len(selected_entity_ids),
        len(segments),
        len(missing_ids),
    )

    if len(segments) < MIN_CLOSED_EDGES:
        return {
            "closed": False,
            "message": (
                f"Select at least {MIN_CLOSED_EDGES} connected guide lines to form a closed surface "
                f"(got {len(segments)} valid line(s))."
            ),
            "line_count": len(segments),
            "missing_ids": missing_ids,
        }

    all_points = [pt for seg in segments for pt in seg]
    tol = float(tolerance) if tolerance is not None else _relative_tolerance(all_points)

    chain, closed, start, end, remaining = _chain_segments(segments, tol)

    if closed and _area(chain) > 0:
        result = {
            "closed": True,
            "points": chain,
            "area": _area(chain),
            "bbox": _bbox(chain),
            "line_count": len(segments),
        }
        logger.info(
            "Table B DXF Assisted check-lines-closed: job_id=%s CLOSED area=%s vertices=%s",
            job_id,
            result["area"],
            len(chain),
        )
        return result

    if remaining:
        message = (
            "Selected lines do not all connect end-to-end — "
            f"{len(remaining)} line(s) are disconnected from the chain."
        )
    else:
        message = "Selected lines form an open chain — the two ends do not meet."

    logger.info("Table B DXF Assisted check-lines-closed: job_id=%s NOT closed (%s)", job_id, message)

    return {
        "closed": False,
        "message": message,
        "open_endpoints": [start, end],
        "line_count": len(segments),
        "missing_ids": missing_ids,
    }
