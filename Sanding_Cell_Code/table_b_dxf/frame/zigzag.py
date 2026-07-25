from __future__ import annotations

import logging
from typing import Any

from .geometry import _polygon, _rings_of
from .paths.chaining import _chain_frame_zigzag_passes, _frame_zigzag_stations

logger = logging.getLogger(__name__)

def compute_frame_zigzag_fill(
    outer_polygon: list[list[float]] | None,
    pocket_polygons: list[list[list[float]]] | None,
    surface3d_polygons: list[list[list[float]]] | None,
    pass_width_mm: float = 75.0,
    overlap_mm: float = 0.0,
) -> dict[str, Any]:
    """Zigzag (serpentine) fill of the frame surface, clipped to the real polygon.

    For "Frame Level on the whole door": fill frame_area = outer − pockets − 3D with
    vertical passes stepping in X at (pass_width − overlap), each pass CLIPPED to the
    frame polygon so it stops at the door's curved edge instead of overrunning the
    bounding rectangle. Passes alternate direction (serpentine), matching the frontend
    zigzag pattern but curve-aware.

    Returns:
      { ok, rings, toolpaths: [{points, tool, operation_type, direction}], frame_area }
    where each toolpath is one continuous vertical pass (as flattened points).
    """
    try:
        from shapely.geometry import LineString
        from shapely.ops import unary_union
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame zigzag: shapely unavailable: %s", error)
        return {"ok": False, "reason": "shapely_unavailable", "rings": [], "toolpaths": []}

    outer = _polygon(outer_polygon)
    if outer is None:
        return {"ok": False, "reason": "no_outer_polygon", "rings": [], "toolpaths": []}

    obstacles = []
    for group in (pocket_polygons or [], surface3d_polygons or []):
        for pts in group:
            p = _polygon(pts)
            if p is not None:
                obstacles.append(p)
    frame_geom = outer if not obstacles else outer.difference(unary_union(obstacles))
    if frame_geom.is_empty or frame_geom.area <= 1e-6:
        return {"ok": True, "rings": [], "toolpaths": [], "frame_area": 0.0}

    minx, miny, maxx, maxy = [float(v) for v in frame_geom.bounds]
    step = max(float(pass_width_mm) - max(0.0, float(overlap_mm)), 1.0)

    toolpaths: list[dict[str, Any]] = []
    # In the normalized frame the machine origin (0,0) is the bottom-right corner, which
    # is min-x / min-y. So the fill must START AT THE ORIGIN CORNER (minx, miny): step X
    # from minx up to maxx (physical right -> left) and run each pass BOTTOM -> TOP first
    # (y0 -> y1), then alternate (serpentine). The first toolpath point is the origin
    # corner; the last pass lands on the far (leftmost) side.
    toggle = 0
    seq = 0
    # Always include both frame boundaries. A plain x += step loop can stop short when
    # the width is not an exact multiple of the spacing, leaving the origin-side corner
    # visually/operationally unconnected.
    for x in _frame_zigzag_stations(minx, maxx, step, float(pass_width_mm)):
        scan = LineString([(x, miny - 1.0), (x, maxy + 1.0)])
        clipped = scan.intersection(frame_geom)
        # The scanline may cross the frame in several disjoint spans (e.g. between two
        # pockets); emit each span as its own pass so no move crosses a hole.
        spans = []
        if clipped.geom_type == "LineString" and not clipped.is_empty:
            spans = [clipped]
        elif clipped.geom_type == "MultiLineString":
            spans = [g for g in clipped.geoms if g.length > 1e-6]
        for span in spans:
            ys = [c[1] for c in span.coords]
            y0, y1 = min(ys), max(ys)
            # toggle 0 => bottom->top, toggle 1 => top->bottom (serpentine)
            pass_pts = [[x, y0], [x, y1]] if not toggle else [[x, y1], [x, y0]]
            toolpaths.append({
                "path_id": f"frame_zigzag_{seq:03d}",
                "points": pass_pts,
                "tool": "tool_4_frame",
                "operation_type": "frame_zigzag_pass",
                "direction": "Y",
            })
            seq += 1
        toggle = 1 - toggle

    toolpaths = _chain_frame_zigzag_passes(toolpaths, frame_geom)

    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "toolpaths": toolpaths,
        "frame_area": sum(r["area"] for r in rings),
        "pass_count": len(toolpaths),
    }

