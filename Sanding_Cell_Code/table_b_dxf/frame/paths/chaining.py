from __future__ import annotations

# Compatibility wrapper. Keep existing imports stable while implementation is split
# into focused modules for safer debugging.

from .constants import (
    FRAME_BOUNDARY_CLEARANCE_MM,
    MAX_OUTER_EDGE_CONTACT_MM,
    OFFSET_CORNER_STUB_MM,
)
from .geometry import (
    _connector_rides_boundary,
    _frame_geom_from_rings,
    _iter_exterior_rings,
    _iter_interior_rings,
    _point_near_outer_boundary,
    _polyline_has_outer_boundary_point,
)
from .helpers import (
    _dedupe_polyline_points,
    _first_segment_axis,
    _frame_zigzag_stations,
    _last_segment_axis,
    _path_source_ids,
    _polyline_self_overlaps,
    _remove_short_backtrack_jogs,
    _same_xy,
)
from .connectors import (
    _axis_aligned_connector,
    _rectangular_centerline_corner,
    _zigzag_step_over,
)
from .zigzag_chain import _chain_frame_zigzag_passes
from .corner_chain import _chain_outer_corner_frame_paths
from .computed_chain import _chain_computed_frame_toolpaths, _greedy_join_frame_paths
from .station_chain import chain_computed_frame_paths_by_station

__all__ = [
    "FRAME_BOUNDARY_CLEARANCE_MM",
    "MAX_OUTER_EDGE_CONTACT_MM",
    "OFFSET_CORNER_STUB_MM",
    "_axis_aligned_connector",
    "_chain_computed_frame_toolpaths",
    "_chain_frame_zigzag_passes",
    "_chain_outer_corner_frame_paths",
    "_connector_rides_boundary",
    "_dedupe_polyline_points",
    "_first_segment_axis",
    "_frame_geom_from_rings",
    "_frame_zigzag_stations",
    "_iter_exterior_rings",
    "_iter_interior_rings",
    "_last_segment_axis",
    "_path_source_ids",
    "_point_near_outer_boundary",
    "_polyline_has_outer_boundary_point",
    "_polyline_self_overlaps",
    "_rectangular_centerline_corner",
    "_remove_short_backtrack_jogs",
    "_same_xy",
    "_zigzag_step_over",
    "_greedy_join_frame_paths",
    "chain_computed_frame_paths_by_station",
]
