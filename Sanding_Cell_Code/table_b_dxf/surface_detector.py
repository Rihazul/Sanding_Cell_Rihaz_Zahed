from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize, snap, unary_union

from .jobs import get_table_b_dxf_job_paths

logger = logging.getLogger(__name__)


def _bbox(points: list[list[float]]) -> dict[str, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {"min_x": min(xs), "min_y": min(ys), "max_x": max(xs), "max_y": max(ys)}


def _relative_tolerance(points: list[list[float]]) -> float:
    if not points:
        return 1e-6
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    return max(diag * 1e-3, 1e-6)


def detect_selected_loops(
    job_id: str,
    selected_entity_ids: list[str],
    tolerance: float | None = None,
) -> dict[str, Any]:
    """Extract every closed loop reachable from the selected guide lines.

    Tolerant of extra/parallel lines: uses shapely polygonize on the selected
    line entities and returns each enclosed face's outer ring as a candidate
    loop. Lines that do not enclose area are simply not part of any loop. Does
    NOT create a surface or toolpath — detection only.
    """
    paths = get_table_b_dxf_job_paths(job_id)
    parsed_path = Path(paths["parsed_loops"])
    if not parsed_path.exists():
        raise FileNotFoundError(
            f"parsed_loops.json not found for job {job_id}. Parse the DXF first."
        )

    parsed = json.loads(parsed_path.read_text(encoding="utf-8"))
    by_id = {path["entity_id"]: path for path in parsed.get("open_paths", [])}

    selected: list[dict[str, Any]] = []
    for entity_id in selected_entity_ids:
        path = by_id.get(entity_id)
        if path and len(path.get("points", [])) >= 2:
            selected.append(path)

    if len(selected) < 2:
        return {"loops": [], "selected_count": len(selected), "candidate_count": 0}

    all_points = [pt for s in selected for pt in s["points"]]
    tol = float(tolerance) if tolerance is not None else _relative_tolerance(all_points)

    # Selection box (the outer boundary the operator drew a box around). The ring is
    # defined ENTIRELY by the lines the operator selected — we never pull in other
    # geometry, so picking the outer edges fills the outer ring and picking the
    # inner edges fills the inner region, exactly as chosen.
    sel_min_x = min(p[0] for p in all_points)
    sel_max_x = max(p[0] for p in all_points)
    sel_min_y = min(p[1] for p in all_points)
    sel_max_y = max(p[1] for p in all_points)
    sel_w = sel_max_x - sel_min_x
    sel_h = sel_max_y - sel_min_y

    lines = [LineString([(float(x), float(y)) for x, y in s["points"]]) for s in selected]

    # Polygonize the selected lines. Hand-drawn DXF boundaries often have tiny
    # gaps between segment endpoints that stop a loop from closing — so snap the
    # geometry to itself within a tolerance to weld those gaps, retrying with a
    # slightly larger tolerance if nothing closed. This is what lets one section
    # that looked identical still form a ring when its lines don't quite meet.
    def _polygonize(snap_tol: float) -> list[Polygon]:
        try:
            merged = unary_union(lines)
            if snap_tol > 0:
                merged = snap(merged, merged, snap_tol)
            return list(polygonize(unary_union(merged)))
        except Exception as error:  # noqa: BLE001 - never let a geometry hiccup fail the request
            logger.warning("Table B DXF Assisted polygonize failed (tol=%s): %s", snap_tol, error)
            return []

    def _outer_complete(candidate: list[Polygon]) -> bool:
        """True if ANY face spans (nearly) the whole selection — i.e. the OUTER
        boundary actually closed. For nested bands polygonize returns the outer as
        an annulus whose AREA is smaller than the inner box, so we must test every
        face's bounding box, not just the largest-area one."""
        for face in candidate:
            minx, miny, maxx, maxy = face.bounds
            if (maxx - minx) >= 0.95 * sel_w and (maxy - miny) >= 0.95 * sel_h:
                return True
        return False

    def _score(candidate: list[Polygon]) -> tuple[int, int]:
        # Prefer results where the outer boundary closed, then those with the most
        # faces (more inner bands / a real ring).
        return (1 if _outer_complete(candidate) else 0, len(candidate))

    # Ring detection from selected lines depends on the OUTER box closing. Hand-
    # drawn DXFs often leave a gap in one section's outer edge, so the outer never
    # polygonizes and only an inner band closes → the ring comes out as just the
    # inner portion. Snapping the geometry to itself welds those gaps; try an
    # increasing ladder and keep whichever best closes the outer boundary.
    faces = _polygonize(0.0)
    for snap_tol in (tol, tol * 3.0, tol * 8.0, tol * 20.0, tol * 50.0):
        candidate = _polygonize(snap_tol)
        if _score(candidate) > _score(faces):
            faces = candidate

    # ALWAYS offer the SELECTION'S BOUNDING RECTANGLE as an outer-boundary candidate.
    # The operator drew a box around the ring, so its bbox is that outer box for the
    # rectangular frames Table B handles. This makes ring detection identical no
    # matter which / how many outer lines were picked: the frontend takes the
    # largest-bbox face as the outer and carves down to the innermost box. Without
    # it, a partial selection that happens to close into an inner band would be
    # mistaken for the outer and the frame band would not fill.
    outer_from_bbox = False
    if sel_w > tol and sel_h > tol and not _outer_complete(faces):
        bbox_ring = Polygon(
            [
                (sel_min_x, sel_min_y),
                (sel_max_x, sel_min_y),
                (sel_max_x, sel_max_y),
                (sel_min_x, sel_max_y),
            ]
        )
        faces = list(faces) + [bbox_ring]
        outer_from_bbox = True
        logger.info(
            "Table B DXF Assisted detect-loops: added selection bbox as outer candidate (job_id=%s)",
            job_id,
        )

    loops: list[dict[str, Any]] = []
    for index, face in enumerate(faces):
        exterior = [[float(x), float(y)] for x, y in list(face.exterior.coords)[:-1]]
        if len(exterior) < 3:
            continue
        area = float(Polygon(exterior).area)
        if area <= tol * tol:
            continue

        # A face may already be a ring (annulus): its holes are the inner
        # boundaries the user's selection enclosed. Carry them through so the
        # frontend can render the ring directly.
        holes = [
            [[float(x), float(y)] for x, y in list(interior.coords)[:-1]]
            for interior in face.interiors
        ]

        boundary = face.exterior
        source_ids: list[str] = []
        for path, line in zip(selected, lines):
            on_boundary = all(
                boundary.distance(Point(float(pt[0]), float(pt[1]))) <= tol for pt in path["points"]
            )
            if on_boundary:
                source_ids.append(path["entity_id"])

        loops.append({
            "loop_id": f"detected_loop_{index + 1}",
            "points": exterior,
            "holes": holes,
            "area": area,
            "net_area": float(face.area),
            "bbox": _bbox(exterior),
            "source_entity_ids": source_ids,
        })

    # Largest boundaries first — the UI picks outer + largest contained inner.
    loops.sort(key=lambda loop: loop["area"], reverse=True)

    logger.info(
        "Table B DXF Assisted detect-loops: job_id=%s selected=%s candidate_loops=%s",
        job_id,
        len(selected),
        len(loops),
    )

    return {
        "loops": loops,
        "selected_count": len(selected),
        "candidate_count": len(loops),
        "outer_from_bbox": outer_from_bbox,
    }
