from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)


def _polygon(points: list[list[float]] | None):
    """Build a valid shapely Polygon from a ring, or None."""
    if not points or len(points) < 3:
        return None
    try:
        from shapely.geometry import Polygon
        from shapely.validation import make_valid

        poly = Polygon([(float(x), float(y)) for x, y in points])
        if not poly.is_valid:
            poly = make_valid(poly)
        return poly if (not poly.is_empty and poly.area > 1e-9) else None
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame_area: bad polygon: %s", error)
        return None


def _rings_of(geom: Any) -> list[dict[str, Any]]:
    """Flatten a shapely (Multi)Polygon into a list of {exterior, holes} rings, mm."""
    rings: list[dict[str, Any]] = []
    if geom is None or geom.is_empty:
        return rings
    polys = []
    gt = geom.geom_type
    if gt == "Polygon":
        polys = [geom]
    elif gt in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in geom.geoms if g.geom_type == "Polygon" and g.area > 1e-6]
    for poly in sorted(polys, key=lambda p: p.area, reverse=True):
        exterior = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
        holes = [
            [[float(x), float(y)] for x, y in interior.coords[:-1]]
            for interior in poly.interiors
        ]
        rings.append({"exterior": exterior, "holes": holes, "area": float(poly.area)})
    return rings


def compute_frame_area(
    outer_polygon: list[list[float]] | None,
    pocket_polygons: list[list[list[float]]] | None,
    surface3d_polygons: list[list[list[float]]] | None,
) -> dict[str, Any]:
    """Frame region = outer door polygon − pocket regions − 3D-contour regions.

    Returns the remaining frame surface as polygon rings (each with exterior + holes),
    in mm, curved edges preserved as flattened segments. This is the true surface the
    frame toolpath must fill; sections and passes are derived from it (later stages).

    Best effort — on any geometry failure returns the outer polygon unchanged so a
    preview is never lost.
    """
    outer = _polygon(outer_polygon)
    if outer is None:
        return {"ok": False, "reason": "no_outer_polygon", "rings": []}

    obstacles = []
    for group in (pocket_polygons or [], surface3d_polygons or []):
        for pts in group:
            p = _polygon(pts)
            if p is not None:
                obstacles.append(p)

    try:
        from shapely.ops import unary_union

        frame_geom = outer
        if obstacles:
            frame_geom = outer.difference(unary_union(obstacles))
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame_area: difference failed: %s", error)
        return {"ok": True, "rings": _rings_of(outer), "note": "difference_failed_used_outer"}

    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "outer_area": float(outer.area),
        "frame_area": sum(r["area"] for r in rings),
        "obstacle_count": len(obstacles),
    }


def _as_polygons(geom: Any) -> list[Any]:
    """Flatten a shapely intersection result into a list of Polygons, largest first."""
    gt = geom.geom_type
    if gt == "Polygon":
        polys = [geom]
    elif gt in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in geom.geoms if g.geom_type == "Polygon" and g.area > 1e-6]
    else:
        polys = []
    return sorted(polys, key=lambda p: p.area, reverse=True)


def _largest_polygon(geom: Any) -> Any:
    """Return the largest Polygon part of any geometry (Polygon/Multi/Collection)."""
    if geom is None or geom.is_empty:
        return geom
    if geom.geom_type == "Polygon":
        return geom
    polys = [g for g in getattr(geom, "geoms", []) if g.geom_type == "Polygon" and not g.is_empty]
    if not polys:
        return geom
    return max(polys, key=lambda g: g.area)
