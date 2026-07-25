from __future__ import annotations

from typing import Any

from .constants import FRAME_BOUNDARY_CLEARANCE_MM
from .geometry import _connector_rides_boundary
from .helpers import (
    _dedupe_polyline_points,
    _first_segment_axis,
    _last_segment_axis,
    _same_xy,
)
from ..toolpaths import _polyline_stays_in_polygon


def _rectangular_centerline_corner(
    current_points: list[list[float]],
    next_points: list[list[float]],
    frame_geom: Any,
) -> list[float] | None:
    if len(current_points) < 2 or len(next_points) < 2:
        return None
    current_axis = _last_segment_axis(current_points)
    next_axis = _first_segment_axis(next_points)
    if current_axis is None or next_axis is None or current_axis == next_axis:
        return None

    current_end = current_points[-1]
    next_start = next_points[0]
    if current_axis == "y" and next_axis == "x":
        corner = [float(current_end[0]), float(next_start[1])]
    elif current_axis == "x" and next_axis == "y":
        corner = [float(next_start[0]), float(current_end[1])]
    else:
        return None

    candidate = _dedupe_polyline_points([current_points[-2], corner, next_points[1]])
    if (
        _polyline_stays_in_polygon(candidate, frame_geom, tol=0.25)
        and not _connector_rides_boundary(candidate, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
    ):
        return corner
    return None


def _axis_aligned_connector(a: list[float], b: list[float], frame_geom: Any, first_axis: str | None = None) -> list[list[float]]:
    """Return a safe direct/orthogonal connector from a to b, or [] if unsafe.

    Zigzag frame transitions should look like robot-safe L moves on rectangular doors.
    A diagonal shortcut is only avoided here; the sanding strokes themselves are left
    untouched. The small tolerance accepts numeric noise, but boundary-overlap is rejected.
    """
    start = [float(a[0]), float(a[1])]
    end = [float(b[0]), float(b[1])]
    if _same_xy(start, end):
        return [start]

    connector_tol = 0.25

    def _connector_ok(pts: list[list[float]]) -> bool:
        # Must stay inside the frame band and clear pocket/3D hole edges. A connector may
        # touch an outer-frame endpoint/corner, but it must not ride along that boundary.
        return (
            _polyline_stays_in_polygon(pts, frame_geom, tol=connector_tol)
            and not _connector_rides_boundary(pts, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
        )

    if abs(start[0] - end[0]) <= 1e-6 or abs(start[1] - end[1]) <= 1e-6:
        direct = _dedupe_polyline_points([start, end])
        return direct if _connector_ok(direct) else []

    horizontal_first = _dedupe_polyline_points([start, [end[0], start[1]], end])
    vertical_first = _dedupe_polyline_points([start, [start[0], end[1]], end])
    if first_axis == "y":
        candidates = [vertical_first, horizontal_first]
    elif first_axis == "x":
        candidates = [horizontal_first, vertical_first]
    else:
        candidates = [horizontal_first, vertical_first]
    for candidate in candidates:
        if _connector_ok(candidate):
            return candidate
    return []


def _zigzag_step_over(a: list[float], b: list[float], frame_geom: Any) -> list[list[float]]:
    """Step-over connector between two adjacent zigzag passes.

    A serpentine turns at the frame's OUTER edge, so its step-over legitimately runs along
    the door boundary — unlike a computed-frame connector, which must stay clear of pocket
    edges. So here we require only that the connector stay INSIDE the frame (a step-over
    that would cross a pocket is still rejected), not that it avoid the boundary. That
    boundary-clearance rule was silently killing every step-over on a whole-door frame,
    leaving the zigzag as a stack of disconnected vertical strokes.
    """
    start = [float(a[0]), float(a[1])]
    end = [float(b[0]), float(b[1])]
    if _same_xy(start, end):
        return [start]
    tol = 0.5

    def _clears_pockets(pts: list[list[float]]) -> bool:
        # Inside the frame is necessary but not sufficient: a connector running along a
        # pocket's edge can clip into the pocket. Reject any connector whose interior
        # overlaps a frame HOLE (a pocket / 3D region), while still allowing one that runs
        # along the OUTER door boundary (where a serpentine legitimately turns).
        if not _polyline_stays_in_polygon(pts, frame_geom, tol=tol):
            return False
        try:
            from shapely.geometry import LineString, Polygon

            line = LineString(pts)
            for interior in getattr(frame_geom, "interiors", []) or []:
                hole = Polygon(interior)
                if line.intersection(hole).length > tol:
                    return False
            for geom in getattr(frame_geom, "geoms", []) or []:
                for interior in getattr(geom, "interiors", []) or []:
                    hole = Polygon(interior)
                    if line.intersection(hole).length > tol:
                        return False
        except Exception:
            return False
        return True

    if abs(start[0] - end[0]) <= 1e-6 or abs(start[1] - end[1]) <= 1e-6:
        direct = _dedupe_polyline_points([start, end])
        return direct if _clears_pockets(direct) else []
    for candidate in (
        _dedupe_polyline_points([start, [end[0], start[1]], end]),
        _dedupe_polyline_points([start, [start[0], end[1]], end]),
    ):
        if _clears_pockets(candidate):
            return candidate
    return []
