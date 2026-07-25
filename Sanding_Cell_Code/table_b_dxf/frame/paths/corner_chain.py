from __future__ import annotations

from typing import Any

from .constants import FRAME_BOUNDARY_CLEARANCE_MM
from .connectors import _axis_aligned_connector
from .geometry import _connector_rides_boundary
from .helpers import _dedupe_polyline_points, _last_segment_axis
from ..toolpaths import _polyline_stays_in_polygon


def _chain_outer_corner_frame_paths(
    toolpaths: list[dict[str, Any]],
    frame_geom: Any,
    max_connector_mm: float,
) -> list[dict[str, Any]]:
    """Join computed-frame passes that meet around the same outer door corner.

    This is deliberately narrower than the main clockwise chain. It only joins endpoints
    that are both close to the same outer frame corner and whose L connector stays inside
    the real frame area. That fixes genuine perimeter gaps near the model origin without
    forcing interior grid rails or pocket-separated strokes into one false loop.
    """
    import math

    if len(toolpaths) < 2 or frame_geom is None or frame_geom.is_empty:
        return toolpaths

    minx, miny, maxx, maxy = [float(v) for v in frame_geom.bounds]
    corners = [[minx, miny], [maxx, miny], [maxx, maxy], [minx, maxy]]
    corner_tol = max(20.0, min(float(max_connector_mm), 120.0))

    def points(path: dict[str, Any]) -> list[list[float]]:
        return [[float(p[0]), float(p[1])] for p in path.get("points") or []]

    def compatible(path: dict[str, Any]) -> bool:
        strategy = str(path.get("path_strategy") or "")
        return (
            path.get("region_type") == "computed_frame"
            and path.get("operation_type") == "frame_section_pass"
            and "computed_frame_clockwise_chain" in strategy
            and len(points(path)) >= 2
        )

    def corner_index(pt: list[float]) -> int | None:
        distances = [math.dist(pt, c) for c in corners]
        idx = min(range(len(distances)), key=lambda i: distances[i])
        return idx if distances[idx] <= corner_tol else None

    def same_outer_corner(a: list[float], b: list[float]) -> bool:
        ai = corner_index(a)
        return ai is not None and ai == corner_index(b)

    def safe_connector(a: list[float], b: list[float], first_axis: str | None) -> list[list[float]]:
        if not same_outer_corner(a, b):
            return []
        if math.dist(a, b) > max(float(max_connector_mm), corner_tol):
            return []
        return _axis_aligned_connector(a, b, frame_geom, first_axis=first_axis)

    remaining = [dict(tp) for tp in toolpaths]
    out: list[dict[str, Any]] = []
    while remaining:
        current = remaining.pop(0)
        current_points = points(current)
        if not compatible(current):
            out.append(current)
            continue

        current_ids = list(current.get("chained_path_ids") or [current.get("path_id")])
        changed = True
        while changed:
            changed = False
            for idx, candidate in enumerate(remaining):
                candidate_points = points(candidate)
                if not compatible(candidate):
                    continue

                options = [
                    ("append", candidate_points),
                    ("append", list(reversed(candidate_points))),
                    ("prepend", candidate_points),
                    ("prepend", list(reversed(candidate_points))),
                ]
                merged: list[list[float]] | None = None
                for mode, cand_pts in options:
                    if mode == "append":
                        connector = safe_connector(current_points[-1], cand_pts[0], _last_segment_axis(current_points))
                        if connector:
                            merged = _dedupe_polyline_points([*current_points, *connector[1:], *cand_pts])
                            break
                    else:
                        connector = safe_connector(cand_pts[-1], current_points[0], _last_segment_axis(cand_pts))
                        if connector:
                            merged = _dedupe_polyline_points([*cand_pts, *connector[1:], *current_points])
                            break

                if merged is None:
                    continue
                if (
                    not _polyline_stays_in_polygon(merged, frame_geom, tol=0.5)
                    or _connector_rides_boundary(merged, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
                ):
                    continue
                current_points = merged
                for cid in candidate.get("chained_path_ids") or [candidate.get("path_id")]:
                    if cid not in current_ids:
                        current_ids.append(cid)
                remaining.pop(idx)
                changed = True
                break

        current["points"] = current_points
        current["start_point"] = current_points[0]
        current["end_point"] = current_points[-1]
        current["chained_path_ids"] = current_ids
        current["path_strategy"] = str(current.get("path_strategy") or "computed_frame") + "_outer_corner_connected"
        out.append(current)

    return out
