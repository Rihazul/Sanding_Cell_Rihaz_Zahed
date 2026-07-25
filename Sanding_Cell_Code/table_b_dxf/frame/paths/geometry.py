from __future__ import annotations

from typing import Any

from .constants import MAX_OUTER_EDGE_CONTACT_MM


def _iter_interior_rings(frame_geom: Any):
    """Yield every hole (pocket / 3D region) ring of the frame, across polygon or multi."""
    for interior in getattr(frame_geom, "interiors", []) or []:
        yield interior
    for geom in getattr(frame_geom, "geoms", []) or []:
        for interior in getattr(geom, "interiors", []) or []:
            yield interior


def _iter_exterior_rings(frame_geom: Any):
    """Yield the OUTER ring of the frame, across polygon or multipolygon."""
    exterior = getattr(frame_geom, "exterior", None)
    if exterior is not None:
        yield exterior
    for geom in getattr(frame_geom, "geoms", []) or []:
        ext = getattr(geom, "exterior", None)
        if ext is not None:
            yield ext


def _connector_rides_boundary(points: list[list[float]], frame_geom: Any, clearance_mm: float) -> bool:
    """True when a connector is unsafe against the frame boundary.

    Three distinct cases — the middle one is what makes the outer edge special:
      • POCKET / 3D hole edge: reject overlap OR mere proximity. The tool must never ride a
        pocket edge (it would clip in), so holes keep the full clearance buffer.
      • OUTER door edge: a frame centreline pass runs parallel to and near this edge, so a
        corner turn legitimately TOUCHES it at the apex. That is allowed. What is NOT allowed
        is RUNNING ALONG the outer edge for a length (the tool sanding the raw door edge), so
        we reject only measurable overlap — no proximity buffer.
      • Interior: fine.
    """
    if len(points) < 2 or frame_geom is None or getattr(frame_geom, "is_empty", True):
        return False
    try:
        from shapely.geometry import LineString, LinearRing

        clearance = max(float(clearance_mm), 0.0)
        min_overlap = 1.0
        for a, b in zip(points, points[1:]):
            line = LineString([(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))])
            if line.length <= 1e-9:
                continue
            # Pocket / 3D holes: overlap or proximity is unsafe.
            for interior in _iter_interior_rings(frame_geom):
                ring = LinearRing(interior)
                inter = line.intersection(ring)
                if not inter.is_empty and float(getattr(inter, "length", 0.0)) > min_overlap:
                    return True
                if clearance > 0.0:
                    nearby = line.buffer(clearance, cap_style=2, join_style=2).intersection(ring)
                    if not nearby.is_empty and float(getattr(nearby, "length", 0.0)) > min_overlap:
                        return True
            # Outer door edge: only running ALONG it (a measurable overlap) is unsafe;
            # a corner touch is allowed.
            for exterior in _iter_exterior_rings(frame_geom):
                ring = LinearRing(exterior)
                inter = line.intersection(ring)
                if not inter.is_empty and float(getattr(inter, "length", 0.0)) > MAX_OUTER_EDGE_CONTACT_MM:
                    return True
    except Exception:
        return True
    return False


def _point_near_outer_boundary(point: list[float], frame_geom: Any, clearance_mm: float) -> bool:
    if frame_geom is None or getattr(frame_geom, "is_empty", True):
        return False
    try:
        from shapely.geometry import LinearRing, Point

        pt = Point(float(point[0]), float(point[1]))
        clearance = max(float(clearance_mm), 0.0)
        for exterior in _iter_exterior_rings(frame_geom):
            if pt.distance(LinearRing(exterior)) <= clearance:
                return True
    except Exception:
        return True
    return False


def _polyline_has_outer_boundary_point(points: list[list[float]], frame_geom: Any, clearance_mm: float) -> bool:
    return any(_point_near_outer_boundary(p, frame_geom, clearance_mm) for p in points)


def _frame_geom_from_rings(frame_rings: list[dict[str, Any]] | None) -> Any:
    if not frame_rings:
        return None
    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union

        polys = []
        for ring in frame_rings:
            exterior = ring.get("exterior") or []
            if len(exterior) < 3:
                continue
            holes = [hole for hole in (ring.get("holes") or []) if len(hole) >= 3]
            poly = Polygon(exterior, holes)
            if not poly.is_empty and poly.area > 1e-6:
                polys.append(poly)
        return unary_union(polys) if polys else None
    except Exception:
        return None
