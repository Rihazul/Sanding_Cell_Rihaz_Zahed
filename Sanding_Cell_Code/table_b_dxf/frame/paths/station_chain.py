from __future__ import annotations

from typing import Any

from .constants import FRAME_BOUNDARY_CLEARANCE_MM
from .connectors import _axis_aligned_connector, _rectangular_centerline_corner
from .geometry import _connector_rides_boundary, _frame_geom_from_rings
from .helpers import (
    _dedupe_polyline_points,
    _last_segment_axis,
    _path_source_ids,
    _remove_short_backtrack_jogs,
)
from ..toolpaths import _polyline_stays_in_polygon


def chain_computed_frame_paths_by_station(
    toolpaths: list[dict[str, Any]],
    frame_rings: list[dict[str, Any]] | None,
    max_connector_mm: float = 160.0,
) -> list[dict[str, Any]]:
    """Merge same-7th-axis computed-frame passes when their endpoints can connect safely.

    Reach planning may split/order paths by station after the frame geometry stage. At that
    point two same-colour passes can be run continuously, but only if we are allowed to
    reverse a pass and the real connector remains inside frame_area. This is computed-frame
    only; pocket and 3D contour closed-loop planning stays separate.
    """
    import math

    frame_geom = _frame_geom_from_rings(frame_rings)
    if len(toolpaths) < 2 or frame_geom is None or frame_geom.is_empty:
        return toolpaths

    max_connector = max(float(max_connector_mm), 1.0)

    def points(path: dict[str, Any]) -> list[list[float]]:
        return [[float(p[0]), float(p[1])] for p in path.get("points") or []]

    def polyline_length(pts: list[list[float]]) -> float:
        return sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))

    def compatible(a: dict[str, Any], b: dict[str, Any] | None = None) -> bool:
        if a.get("region_type") != "computed_frame" or a.get("operation_type") != "frame_section_pass":
            return False
        pts = points(a)
        if len(pts) != 2:
            return False
        # Wide zigzag chunks already contain their own coverage pattern. Merging them with
        # single-pass bands creates the outer-boundary/hover connectors seen in preview.
        if a.get("source_chunk_id"):
            return False
        strategy = str(a.get("path_strategy") or "")
        if "zigzag" in strategy or "curved" in strategy or "diagonal" in strategy:
            return False
        if b is None:
            return True
        return (
            compatible(b)
            and a.get("station_index") == b.get("station_index")
            and a.get("station_index") is not None
            and a.get("operation_type") == b.get("operation_type")
        )

    def safe_connector(a: list[float], b: list[float], first_axis: str | None) -> list[list[float]]:
        if math.dist(a, b) > max_connector:
            return []
        return _axis_aligned_connector(a, b, frame_geom, first_axis=first_axis)

    def transition_overlaps_existing(
        transition: list[list[float]],
        ignored_path_ids: set[Any],
    ) -> bool:
        if len(transition) < 2:
            return False
        try:
            from shapely.geometry import LineString

            transition_line = LineString(transition)
            if transition_line.length <= 1e-6:
                return False
            for path in toolpaths:
                path_ids = set(path.get("chained_path_ids") or [path.get("path_id")])
                if path_ids & ignored_path_ids:
                    continue
                if path.get("region_type") != "computed_frame" or path.get("operation_type") != "frame_section_pass":
                    continue
                blocker_points = points(path)
                if len(blocker_points) < 2:
                    continue
                inter = transition_line.intersection(LineString(blocker_points))
                if inter.is_empty:
                    continue
                # A real overlap means this transition would sand over a pass that already
                # exists. Touching at one point is allowed; sharing a segment is not.
                if float(getattr(inter, "length", 0.0)) > 1.0:
                    return True
        except Exception:
            return True
        return False

    def append_transition(
        connector: list[list[float]],
        cand_pts: list[list[float]],
    ) -> list[list[float]]:
        transition = _dedupe_polyline_points(connector)
        if len(cand_pts) > 1:
            transition = _dedupe_polyline_points([*transition, cand_pts[1]])
        return transition

    def prepend_transition(
        cand_pts: list[list[float]],
        connector: list[list[float]],
    ) -> list[list[float]]:
        transition = _dedupe_polyline_points(connector)
        if len(cand_pts) > 1:
            transition = _dedupe_polyline_points([cand_pts[-2], *transition])
        return transition

    def merge_ids(a: dict[str, Any], b: dict[str, Any]) -> list[Any]:
        ids: list[Any] = []
        for source in (a, b):
            for pid in source.get("chained_path_ids") or [source.get("path_id")]:
                if pid is not None and pid not in ids:
                    ids.append(pid)
        return ids

    remaining = [dict(path) for path in toolpaths]
    output: list[dict[str, Any]] = []
    while remaining:
        current = remaining.pop(0)
        current_points = points(current)
        if not compatible(current):
            output.append(current)
            continue

        changed = True
        while changed:
            changed = False
            best: tuple[float, int, list[list[float]], dict[str, Any]] | None = None
            for idx, candidate in enumerate(remaining):
                if not compatible(current, candidate):
                    continue
                candidate_points = points(candidate)
                variants = [
                    candidate_points,
                    list(reversed(candidate_points)),
                ]
                for cand_pts in variants:
                    ignored_ids = set(merge_ids(current, candidate))

                    append_current = [list(p) for p in current_points]
                    append_candidate = [list(p) for p in cand_pts]
                    append_corner = _rectangular_centerline_corner(append_current, append_candidate, frame_geom)
                    if append_corner is not None:
                        trimmed_current = [*append_current[:-1], append_corner]
                        trimmed_candidate = [append_corner, *append_candidate[1:]]
                        if polyline_length(trimmed_current) > 1.0 and polyline_length(trimmed_candidate) > 1.0:
                            append_current = trimmed_current
                            append_candidate = trimmed_candidate
                            connector = [append_corner]
                        else:
                            connector = safe_connector(append_current[-1], append_candidate[0], _last_segment_axis(append_current))
                    else:
                        connector = safe_connector(append_current[-1], append_candidate[0], _last_segment_axis(append_current))
                    if connector:
                        if transition_overlaps_existing(append_transition(connector, append_candidate), ignored_ids):
                            continue
                        merged = _dedupe_polyline_points([*append_current, *connector[1:], *append_candidate])
                        if (
                            _polyline_stays_in_polygon(merged, frame_geom, tol=0.5)
                            and not _connector_rides_boundary(merged, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
                        ):
                            cost = math.dist(current_points[-1], cand_pts[0])
                            if best is None or cost < best[0]:
                                best = (cost, idx, merged, candidate)

                    prepend_candidate = [list(p) for p in cand_pts]
                    prepend_current = [list(p) for p in current_points]
                    prepend_corner = _rectangular_centerline_corner(prepend_candidate, prepend_current, frame_geom)
                    if prepend_corner is not None:
                        trimmed_candidate = [*prepend_candidate[:-1], prepend_corner]
                        trimmed_current = [prepend_corner, *prepend_current[1:]]
                        if polyline_length(trimmed_candidate) > 1.0 and polyline_length(trimmed_current) > 1.0:
                            prepend_candidate = trimmed_candidate
                            prepend_current = trimmed_current
                            prepend_connector = [prepend_corner]
                        else:
                            prepend_connector = safe_connector(prepend_candidate[-1], prepend_current[0], _last_segment_axis(prepend_candidate))
                    else:
                        prepend_connector = safe_connector(prepend_candidate[-1], prepend_current[0], _last_segment_axis(prepend_candidate))
                    if prepend_connector:
                        if transition_overlaps_existing(prepend_transition(prepend_candidate, prepend_connector), ignored_ids):
                            continue
                        merged = _dedupe_polyline_points([*prepend_candidate, *prepend_connector[1:], *prepend_current])
                        if (
                            _polyline_stays_in_polygon(merged, frame_geom, tol=0.5)
                            and not _connector_rides_boundary(merged, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
                        ):
                            cost = math.dist(cand_pts[-1], current_points[0])
                            if best is None or cost < best[0]:
                                best = (cost, idx, merged, candidate)


            if best is None:
                break
            _, idx, merged_points, candidate = best
            current_points = merged_points
            current["points"] = current_points
            current["start_point"] = current_points[0]
            current["end_point"] = current_points[-1]
            current["chained_path_ids"] = merge_ids(current, candidate)
            current["path_strategy"] = str(current.get("path_strategy") or "computed_frame") + "_same_station_connected"
            remaining.pop(idx)
            changed = True

        cleaned_points = _remove_short_backtrack_jogs(current.get("points") or [])
        if len(cleaned_points) >= 2:
            current["points"] = cleaned_points
            current["start_point"] = cleaned_points[0]
            current["end_point"] = cleaned_points[-1]
        output.append(current)

    for idx, path in enumerate(output, start=1):
        if path.get("region_type") == "computed_frame" and path.get("operation_type") == "frame_section_pass":
            path["path_id"] = f"frame_path_{idx:03d}"
    return output
