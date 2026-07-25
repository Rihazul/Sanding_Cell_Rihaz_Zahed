from __future__ import annotations

from typing import Any

from .connectors import _zigzag_step_over
from .helpers import _dedupe_polyline_points, _same_xy


def _chain_frame_zigzag_passes(toolpaths: list[dict[str, Any]], frame_geom: Any) -> list[dict[str, Any]]:
    """Connect adjacent frame-zigzag passes with step-overs that stay inside the frame."""
    if len(toolpaths) < 2 or frame_geom is None or frame_geom.is_empty:
        return toolpaths

    chained: list[dict[str, Any]] = []
    current = dict(toolpaths[0])
    current_points = [list(p) for p in current.get("points") or []]
    current_ids = [current.get("path_id")]

    for next_path in toolpaths[1:]:
        next_points = [list(p) for p in next_path.get("points") or []]
        if len(current_points) < 2 or len(next_points) < 2:
            continue

        connector = _zigzag_step_over(current_points[-1], next_points[0], frame_geom)
        if connector:
            current_points.extend(connector[1:])
            if not _same_xy(current_points[-1], next_points[0]):
                current_points.append(next_points[0])
            current_points.extend(next_points[1:])
            current_points = _dedupe_polyline_points(current_points)
            current_ids.append(next_path.get("path_id"))
            continue

        current["points"] = current_points
        current["start_point"] = current_points[0]
        current["end_point"] = current_points[-1]
        current["chained_path_ids"] = current_ids
        current["path_strategy"] = "frame_zigzag_l_connectors_where_safe"
        chained.append(current)

        current = dict(next_path)
        current_points = next_points
        current_ids = [next_path.get("path_id")]

    current["points"] = current_points
    current["start_point"] = current_points[0]
    current["end_point"] = current_points[-1]
    current["chained_path_ids"] = current_ids
    current["path_strategy"] = "frame_zigzag_l_connectors_where_safe"
    chained.append(current)

    for idx, path in enumerate(chained):
        path["path_id"] = f"frame_zigzag_{idx:03d}"
    return chained
