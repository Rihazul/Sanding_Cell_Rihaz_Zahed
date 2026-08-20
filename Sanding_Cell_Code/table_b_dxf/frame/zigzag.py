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
    orientation: str = "vertical",
) -> dict[str, Any]:
    """Zigzag (serpentine) fill of the frame surface, clipped to the real polygon.

    For "Frame Level on the whole door": fill frame_area = outer − pockets − 3D with
    vertical or horizontal passes at (pass_width − overlap), each pass CLIPPED to the
    frame polygon so it stops at the door's curved edge instead of overrunning the
    bounding rectangle. Rectangular spiral is intended for flat whole-door frame level.

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

    selected = str(orientation or "vertical").strip().lower()
    if selected not in {"vertical", "horizontal", "rectspiral"}:
        selected = "vertical"

    toolpaths: list[dict[str, Any]] = []
    seq = 0

    def _line_spans(scan: Any) -> list[Any]:
        clipped = scan.intersection(frame_geom)
        if clipped.is_empty:
            return []
        if clipped.geom_type == "LineString":
            return [clipped] if clipped.length > 1e-6 else []
        if clipped.geom_type == "MultiLineString":
            return [g for g in clipped.geoms if g.length > 1e-6]
        if clipped.geom_type == "GeometryCollection":
            return [g for g in clipped.geoms if g.geom_type == "LineString" and g.length > 1e-6]
        return []

    if selected == "rectspiral":
        left = minx
        right = maxx
        bottom = miny
        top = maxy
        points: list[list[float]] = [[right, bottom]]

        def _append(point: list[float]) -> None:
            if abs(points[-1][0] - point[0]) > 1e-6 or abs(points[-1][1] - point[1]) > 1e-6:
                points.append([float(point[0]), float(point[1])])

        while right - left > step and top - bottom > step:
            _append([right, top])
            _append([left, top])
            _append([left, bottom])
            right -= step
            bottom += step
            if right <= left or top <= bottom:
                break
            _append([right, bottom])
            top -= step
            left += step
            if right <= left or top <= bottom:
                break

        if right > left and top > bottom:
            cx = (left + right) / 2.0
            cy = (bottom + top) / 2.0
            if top - bottom >= right - left:
                _append([cx, points[-1][1]])
                _append([cx, cy])
            else:
                _append([points[-1][0], cy])
                _append([cx, cy])

        toolpaths.append({
            "path_id": "frame_zigzag_000",
            "points": points,
            "tool": "tool_4_frame",
            "operation_type": "frame_zigzag_pass",
            "direction": "SPIRAL",
            "frame_zigzag_orientation": "rectspiral",
        })
    elif selected == "horizontal":
        toggle = 0
        for y in _frame_zigzag_stations(miny, maxy, step, float(pass_width_mm)):
            scan = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
            spans = _line_spans(scan)
            for span in spans:
                xs = [c[0] for c in span.coords]
                x0, x1 = min(xs), max(xs)
                pass_pts = [[x0, y], [x1, y]] if not toggle else [[x1, y], [x0, y]]
                toolpaths.append({
                    "path_id": f"frame_zigzag_{seq:03d}",
                    "points": pass_pts,
                    "tool": "tool_4_frame",
                    "operation_type": "frame_zigzag_pass",
                    "direction": "X",
                    "frame_zigzag_orientation": "horizontal",
                })
                seq += 1
            toggle = 1 - toggle
    else:
        # In the normalized frame the machine origin (0,0) is the bottom-right corner,
        # which is min-x / min-y in this viewer coordinate system. Start there and step X.
        toggle = 0
        for x in _frame_zigzag_stations(minx, maxx, step, float(pass_width_mm)):
            scan = LineString([(x, miny - 1.0), (x, maxy + 1.0)])
            spans = _line_spans(scan)
            for span in spans:
                ys = [c[1] for c in span.coords]
                y0, y1 = min(ys), max(ys)
                pass_pts = [[x, y0], [x, y1]] if not toggle else [[x, y1], [x, y0]]
                toolpaths.append({
                    "path_id": f"frame_zigzag_{seq:03d}",
                    "points": pass_pts,
                    "tool": "tool_4_frame",
                    "operation_type": "frame_zigzag_pass",
                    "direction": "Y",
                    "frame_zigzag_orientation": "vertical",
                })
                seq += 1
            toggle = 1 - toggle

    if selected != "rectspiral":
        toolpaths = _chain_frame_zigzag_passes(toolpaths, frame_geom)

    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "toolpaths": toolpaths,
        "frame_area": sum(r["area"] for r in rings),
        "pass_count": len(toolpaths),
    }

