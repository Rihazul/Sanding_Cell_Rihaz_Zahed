from __future__ import annotations

from typing import Any

from .constants import FRAME_BOUNDARY_CLEARANCE_MM
from .connectors import _axis_aligned_connector
from .geometry import _connector_rides_boundary, _frame_geom_from_rings
from .helpers import (
    _dedupe_polyline_points,
    _last_segment_axis,
    _remove_collinear_waypoints,
    _path_source_ids,
    _remove_short_backtrack_jogs,
)
from ..toolpaths import _polyline_stays_in_polygon
from ...tool_reach import station_window_for_path


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

    def zigzag_orientation_variants(path: dict[str, Any], pts: list[list[float]]) -> list[list[list[float]]]:
        """Return equivalent serpentine orientations for connecting a frame zigzag."""
        variants = [[list(p) for p in pts], list(reversed([list(p) for p in pts]))]
        if str(path.get("path_strategy") or "") != "oriented_zigzag" or len(pts) < 4 or len(pts) % 2:
            return variants

        flipped: list[list[float]] = []
        for idx in range(0, len(pts), 2):
            # Reverse each sanding stroke while keeping the serpentine stroke order. This
            # moves the entry/exit side from bottom to top, or top to bottom, without
            # changing which frame area is covered or the operator-selected overlap.
            flipped.extend([list(pts[idx + 1]), list(pts[idx])])
        variants.extend([flipped, list(reversed(flipped))])

        unique: list[list[list[float]]] = []
        seen = set()
        for variant in variants:
            key = tuple((round(float(p[0]), 6), round(float(p[1]), 6)) for p in variant)
            if key not in seen:
                unique.append(variant)
                seen.add(key)
        return unique

    def compatible(a: dict[str, Any], b: dict[str, Any] | None = None) -> bool:
        if a.get("region_type") != "computed_frame" or a.get("operation_type") != "frame_section_pass":
            return False
        pts = points(a)
        if len(pts) < 2:
            return False
        strategy = str(a.get("path_strategy") or "")
        is_frame_zigzag = strategy == "oriented_zigzag" and bool(a.get("source_chunk_id"))
        if a.get("source_chunk_id") and not is_frame_zigzag:
            return False
        is_diagonal_band = len(pts) == 2 and strategy in ("diagonal_centerline", "curved_diagonal_chord")
        if "curved" in strategy and not is_diagonal_band:
            return False
        if "diagonal" in strategy and not is_diagonal_band:
            return False
        is_lower_band = len(pts) == 2 and strategy.startswith("structural_lower_band_centerline")
        is_simple_band = len(pts) == 2 and strategy in ("band_centerline", "diagonal_centerline", "curved_diagonal_chord")
        is_computed_chain = len(pts) >= 2 and strategy.startswith("computed_frame_clockwise_chain")
        is_station_chain = len(pts) > 2 and strategy.endswith("_same_station_connected")
        if "zigzag" in strategy and not is_frame_zigzag:
            return False
        if not (is_simple_band or is_lower_band or is_computed_chain or is_station_chain or is_frame_zigzag):
            return False
        if b is None:
            return is_simple_band or is_lower_band or is_computed_chain or is_frame_zigzag

        b_pts = points(b)
        b_strategy = str(b.get("path_strategy") or "")
        b_is_lower_band = len(b_pts) == 2 and b_strategy.startswith("structural_lower_band_centerline")
        b_is_simple_band = len(b_pts) == 2 and b_strategy in ("band_centerline", "diagonal_centerline", "curved_diagonal_chord")
        b_is_computed_chain = len(b_pts) >= 2 and b_strategy.startswith("computed_frame_clockwise_chain")
        b_is_frame_zigzag = b_strategy == "oriented_zigzag" and bool(b.get("source_chunk_id"))

        if is_station_chain and not (b_is_simple_band or b_is_lower_band or b_is_computed_chain or b_is_frame_zigzag):
            return False

        if is_lower_band or b_is_lower_band:
            a_sections = set(a.get("source_section_ids") or [a.get("source_section_id")])
            b_sections = set(b.get("source_section_ids") or [b.get("source_section_id")])
            if not (is_lower_band and b_is_lower_band and a_sections & b_sections):
                return False

        if not compatible(b) or a.get("operation_type") != b.get("operation_type"):
            return False
        if a.get("station_index") == b.get("station_index") and a.get("station_index") is not None:
            return True
        # Diagonal frame chords may be assigned to different greedy stations even when a
        # shared reach window still exists. Let the merge validator prove reachability.
        return path_is_diagonal_band(a) and path_is_diagonal_band(b)

    def path_is_frame_zigzag(path: dict[str, Any]) -> bool:
        strategy = str(path.get("path_strategy") or "")
        return bool(path.get("contains_frame_zigzag")) or (
            strategy == "oriented_zigzag" and bool(path.get("source_chunk_id"))
        )

    def path_is_diagonal_band(path: dict[str, Any]) -> bool:
        strategy = str(path.get("path_strategy") or "")
        return len(points(path)) == 2 and strategy in ("diagonal_centerline", "curved_diagonal_chord")

    def path_is_lower_band(path: dict[str, Any]) -> bool:
        strategy = str(path.get("path_strategy") or "")
        return strategy.startswith("structural_lower_band_centerline") and len(points(path)) >= 2

    def same_lower_band_section(a: dict[str, Any], b: dict[str, Any]) -> bool:
        a_sections = {sid for sid in (a.get("source_section_ids") or [a.get("source_section_id")]) if sid}
        b_sections = {sid for sid in (b.get("source_section_ids") or [b.get("source_section_id")]) if sid}
        return bool(a_sections and b_sections and a_sections & b_sections)

    def merged_reachable(points_to_check: list[list[float]]) -> bool:
        return station_window_for_path("tool_4", points_to_check) is not None
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
                path_ids = set(station_path_ids(path))
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

    def connector_overlaps_any_frame_pass(connector: list[list[float]]) -> bool:
        """Reject zigzag connectors that would re-sand more than a small landing overlap."""
        if len(connector) < 2:
            return False
        try:
            from shapely.geometry import LineString

            connector_line = LineString(connector)
            if connector_line.length <= 1e-6:
                return False
            for path in toolpaths:
                if path.get("region_type") != "computed_frame" or path.get("operation_type") != "frame_section_pass":
                    continue
                blocker_points = points(path)
                if len(blocker_points) < 2:
                    continue
                inter = connector_line.intersection(LineString(blocker_points))
                # A small overlap at the join point is acceptable and often required to
                # make a clean L-entry into a centerline. Larger overlap means the tool
                # would re-sand an existing pass instead of just connecting to it.
                if not inter.is_empty and float(getattr(inter, "length", 0.0)) > 25.0:
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

    def station_path_ids(path: dict[str, Any]) -> list[Any]:
        # Reach splitting can inherit chained_path_ids from a larger pre-reach chain. For
        # station-level validation, each split piece is its own executable rail, so use the
        # actual post-reach path_id unless this function has already created a same-station
        # chain. Otherwise a small split piece is incorrectly required to cover the whole
        # original perimeter chain and safe connectors are rejected.
        strategy = str(path.get("path_strategy") or "")
        if strategy.endswith("_same_station_connected"):
            ids = path.get("chained_path_ids")
            if isinstance(ids, list) and ids:
                return [pid for pid in ids if pid is not None]
        pid = path.get("path_id")
        return [pid] if pid is not None else []

    original_by_path_id: dict[Any, dict[str, Any]] = {}
    for original in toolpaths:
        for pid in station_path_ids(original):
            original_by_path_id[pid] = dict(original)

    def merge_ids(a: dict[str, Any], b: dict[str, Any]) -> list[Any]:
        ids: list[Any] = []
        for source in (a, b):
            for pid in station_path_ids(source):
                if pid is not None and pid not in ids:
                    ids.append(pid)
        return ids

    def merge_source_ids(a: dict[str, Any], b: dict[str, Any]) -> list[Any]:
        ids: list[Any] = []
        for source in (a, b):
            for sid in _path_source_ids(source):
                if sid is not None and sid not in ids:
                    ids.append(sid)
        return ids

    def covers_claimed_originals(merged_points: list[list[float]], claimed_path_ids: set[Any]) -> bool:
        if len(merged_points) < 2:
            return False
        try:
            from shapely.geometry import LineString

            merged_line = LineString(merged_points)
            if merged_line.is_empty or merged_line.length <= 1e-6:
                return False

            # Validate each claimed source rail independently. A connector may add length,
            # so total path length cannot prove coverage; this catches the failure where a
            # full vertical rail was marked chained but only its lower half was present.
            for pid in claimed_path_ids:
                original = original_by_path_id.get(pid)
                if original is None:
                    continue
                original_points = points(original)
                if len(original_points) < 2:
                    continue
                original_line = LineString(original_points)
                if original_line.is_empty or original_line.length <= 1e-6:
                    continue
                covered = merged_line.intersection(original_line.buffer(2.0, cap_style=2, join_style=2))
                covered_len = float(getattr(covered, "length", 0.0))
                if covered_len < original_line.length * 0.98:
                    return False
            return True
        except Exception:
            return False

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
                variants = zigzag_orientation_variants(candidate, candidate_points)
                for cand_pts in variants:
                    ignored_ids = set(merge_ids(current, candidate))
                    has_zigzag_endpoint = path_is_frame_zigzag(current) or path_is_frame_zigzag(candidate)

                    append_current = [list(p) for p in current_points]
                    append_candidate = [list(p) for p in cand_pts]
                    connector = safe_connector(append_current[-1], append_candidate[0], _last_segment_axis(append_current))
                    if connector:
                        if has_zigzag_endpoint and connector_overlaps_any_frame_pass(connector):
                            continue
                        if transition_overlaps_existing(append_transition(connector, append_candidate), ignored_ids):
                            continue
                        merged = _dedupe_polyline_points([*append_current, *connector[1:], *append_candidate])
                        boundary_check_points = connector if has_zigzag_endpoint else merged
                        if (
                            _polyline_stays_in_polygon(merged, frame_geom, tol=0.5)
                            and not _connector_rides_boundary(boundary_check_points, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
                            and (has_zigzag_endpoint or covers_claimed_originals(merged, ignored_ids))
                            and merged_reachable(merged)
                        ):
                            cost = math.dist(current_points[-1], cand_pts[0])
                            if best is None or cost < best[0]:
                                best = (cost, idx, merged, candidate)

                    prepend_candidate = [list(p) for p in cand_pts]
                    prepend_current = [list(p) for p in current_points]
                    prepend_connector = safe_connector(prepend_candidate[-1], prepend_current[0], _last_segment_axis(prepend_candidate))
                    if prepend_connector:
                        if has_zigzag_endpoint and connector_overlaps_any_frame_pass(prepend_connector):
                            continue
                        if transition_overlaps_existing(prepend_transition(prepend_candidate, prepend_connector), ignored_ids):
                            continue
                        merged = _dedupe_polyline_points([*prepend_candidate, *prepend_connector[1:], *prepend_current])
                        boundary_check_points = prepend_connector if has_zigzag_endpoint else merged
                        if (
                            _polyline_stays_in_polygon(merged, frame_geom, tol=0.5)
                            and not _connector_rides_boundary(boundary_check_points, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
                            and (has_zigzag_endpoint or covers_claimed_originals(merged, ignored_ids))
                            and merged_reachable(merged)
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
            current["source_section_ids"] = merge_source_ids(current, candidate)
            current["contains_frame_zigzag"] = path_is_frame_zigzag(current) or path_is_frame_zigzag(candidate)
            current["path_strategy"] = str(current.get("path_strategy") or "computed_frame") + "_same_station_connected"
            remaining.pop(idx)
            changed = True

        cleaned_points = _remove_collinear_waypoints(_remove_short_backtrack_jogs(current.get("points") or []))
        if len(cleaned_points) >= 2:
            current["points"] = cleaned_points
            current["start_point"] = cleaned_points[0]
            current["end_point"] = cleaned_points[-1]
        output.append(current)

    for idx, path in enumerate(output, start=1):
        if path.get("region_type") == "computed_frame" and path.get("operation_type") == "frame_section_pass":
            path["path_id"] = f"frame_path_{idx:03d}"
    return output
