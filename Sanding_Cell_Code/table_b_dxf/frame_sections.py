from __future__ import annotations

"""Clip computed frame sections to the door outline (shapely).

The frontend builds coarse rectangular frame sections over the door's bounding box.
On a curved door, sections that straddle the curved edge must be trimmed to the real
boundary so their sanding passes follow the curve (as continuous point-to-point moves)
instead of overrunning it. Doing the intersection here with shapely is exact and
handles concave outlines and holes, which a pure-JS clip does not.
"""

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
    x = minx
    toggle = 0
    seq = 0
    # A tiny inset keeps a vertical scanline off the exact left/right edge so the very
    # first/last pass still intersects the polygon interior.
    while x <= maxx + 1e-6:
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
        x += step
        toggle = 1 - toggle

    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "toolpaths": toolpaths,
        "frame_area": sum(r["area"] for r in rings),
        "pass_count": len(toolpaths),
    }


def compute_frame_sections_and_toolpaths(
    outer_polygon: list[list[float]] | None,
    pocket_polygons: list[list[list[float]]] | None,
    surface3d_polygons: list[list[list[float]]] | None,
    pass_width_mm: float = 75.0,
    offset_mm: float = 50.0,
    overlap_mm: float = 0.0,
    reach_x_mm: float = 515.0,
    reach_y_mm: float = 750.0,
) -> dict[str, Any]:
    """Compute frame sections and clipped straight/zigzag toolpaths from frame_area.

    This keeps the previous frontend constraints, but the geometry source is now the
    true frame surface: outer door minus pockets and 3D contour regions.
    """
    try:
        from shapely.geometry import LineString, Polygon, box
        from shapely.ops import unary_union
    except Exception as error:  # noqa: BLE001
        logger.warning("Table B DXF frame toolpath: shapely unavailable: %s", error)
        area = compute_frame_area(outer_polygon, pocket_polygons, surface3d_polygons)
        return {"ok": False, **area, "sections": [], "chunks": [], "toolpaths": [], "reason": "shapely_unavailable"}

    outer = _polygon(outer_polygon)
    if outer is None:
        return {"ok": False, "reason": "no_outer_polygon", "rings": [], "sections": [], "chunks": [], "toolpaths": []}

    obstacles = []
    obstacle_points: list[list[float]] = []
    for group in (pocket_polygons or [], surface3d_polygons or []):
        for pts in group:
            p = _polygon(pts)
            if p is not None:
                obstacles.append(p)
                obstacle_points.extend(pts)

    frame_geom = outer
    if obstacles:
        frame_geom = outer.difference(unary_union(obstacles))
    if frame_geom.is_empty or frame_geom.area <= 1e-6:
        return {"ok": True, "rings": [], "sections": [], "chunks": [], "toolpaths": [], "frame_area": 0.0, "obstacle_count": len(obstacles)}

    minx, miny, maxx, maxy = [float(v) for v in frame_geom.bounds]
    xs = _unique_sorted([minx, maxx, *[float(p[0]) for p in obstacle_points if minx - 1e-6 <= float(p[0]) <= maxx + 1e-6]])
    ys = _unique_sorted([miny, maxy, *[float(p[1]) for p in obstacle_points if miny - 1e-6 <= float(p[1]) <= maxy + 1e-6]])
    if len(xs) < 2 or len(ys) < 2:
        rings = _rings_of(frame_geom)
        return {"ok": True, "rings": rings, "sections": [], "chunks": [], "toolpaths": [], "frame_area": sum(r["area"] for r in rings), "obstacle_count": len(obstacles)}

    full: list[list[bool]] = [[False for _ in range(len(xs) - 1)] for _ in range(len(ys) - 1)]
    used: list[list[bool]] = [[False for _ in range(len(xs) - 1)] for _ in range(len(ys) - 1)]
    partial_geoms: list[Any] = []
    min_area = 1e-5

    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            cell = box(xs[i], ys[j], xs[i + 1], ys[j + 1])
            if cell.area <= min_area:
                continue
            inter = cell.intersection(frame_geom)
            if inter.is_empty or inter.area <= min_area:
                continue
            if abs(inter.area - cell.area) / max(cell.area, min_area) <= 1e-4:
                full[j][i] = True
            else:
                partial_geoms.extend(_as_polygons(inter))

    sections: list[dict[str, Any]] = []
    counter = 0

    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            if not full[j][i] or used[j][i]:
                continue
            i2 = i
            while i2 + 1 < len(xs) - 1 and full[j][i2 + 1] and not used[j][i2 + 1]:
                i2 += 1
            j2 = j
            can_extend = True
            while can_extend and j2 + 1 < len(ys) - 1:
                for k in range(i, i2 + 1):
                    if not full[j2 + 1][k] or used[j2 + 1][k]:
                        can_extend = False
                        break
                if can_extend:
                    j2 += 1
            for jj in range(j, j2 + 1):
                for ii in range(i, i2 + 1):
                    used[jj][ii] = True
            geom = box(xs[i], ys[j], xs[i2 + 1], ys[j2 + 1])
            section = _section_from_polygon(geom, counter + 1, False)
            if section:
                counter += 1
                sections.append(section)

    # Merge adjacent clipped cells back into meaningful curved/diagonal bands.
    # The grid is useful for rectangular frame blocks, but leaving every partial
    # cell separate creates clustered micro-sections on arcs and diagonal pockets.
    merged_partials = _as_polygons(unary_union(partial_geoms)) if partial_geoms else []
    for poly in sorted(merged_partials, key=lambda p: p.area, reverse=True):
        section = _section_from_polygon(poly, counter + 1, True)
        if section:
            counter += 1
            sections.append(section)

    chunks = _split_sections_by_reach(sections, reach_x_mm, reach_y_mm)
    toolpaths = _generate_section_toolpaths(chunks, pass_width_mm, offset_mm, overlap_mm)
    rings = _rings_of(frame_geom)
    return {
        "ok": True,
        "rings": rings,
        "sections": sections,
        "chunks": chunks,
        "toolpaths": toolpaths,
        "outer_area": float(outer.area),
        "frame_area": sum(r["area"] for r in rings),
        "obstacle_count": len(obstacles),
    }


def _unique_sorted(values: list[float]) -> list[float]:
    return sorted(set(round(float(v), 4) for v in values))


def _section_from_polygon(poly: Any, index: int, clipped: bool) -> dict[str, Any] | None:
    if poly.is_empty or poly.area <= 1e-6:
        return None
    if poly.geom_type != "Polygon":
        polys = _as_polygons(poly)
        if not polys:
            return None
        poly = polys[0]
    exterior = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
    if len(exterior) < 3:
        return None
    minx, miny, maxx, maxy = [float(v) for v in poly.bounds]
    width = maxx - minx
    height = maxy - miny
    if width <= 1e-6 or height <= 1e-6:
        return None
    return {
        "section_id": f"frame_section_{index:03d}",
        "points": exterior,
        "bbox": {"min_x": minx, "min_y": miny, "max_x": maxx, "max_y": maxy},
        "width": width,
        "height": height,
        "orientation": "horizontal" if width >= height else "vertical",
        "covered": True,
        "clipped": bool(clipped or len(exterior) > 4),
        "area": float(poly.area),
        "region_type": "computed_frame",
    }


def _split_sections_by_reach(sections: list[dict[str, Any]], reach_x: float, reach_y: float) -> list[dict[str, Any]]:
    from shapely.geometry import Polygon, box

    chunks: list[dict[str, Any]] = []
    counter = 0
    for section in sections:
        pts = section.get("points") or []
        if len(pts) < 3:
            continue
        poly = Polygon([(float(x), float(y)) for x, y in pts])
        if not poly.is_valid:
            from shapely.validation import make_valid
            poly = make_valid(poly)
        bbox = section.get("bbox") or {}
        minx = float(bbox.get("min_x", poly.bounds[0]))
        miny = float(bbox.get("min_y", poly.bounds[1]))
        maxx = float(bbox.get("max_x", poly.bounds[2]))
        maxy = float(bbox.get("max_y", poly.bounds[3]))
        nx = max(1, int(__import__("math").ceil((maxx - minx) / max(reach_x, 1e-6) - 1e-9)))
        ny = max(1, int(__import__("math").ceil((maxy - miny) / max(reach_y, 1e-6) - 1e-9)))
        for ix in range(nx):
            x0 = minx + (maxx - minx) * ix / nx
            x1 = minx + (maxx - minx) * (ix + 1) / nx
            for iy in range(ny):
                y0 = miny + (maxy - miny) * iy / ny
                y1 = miny + (maxy - miny) * (iy + 1) / ny
                inter = poly.intersection(box(x0, y0, x1, y1))
                for piece in _as_polygons(inter):
                    if piece.area <= 1e-6:
                        continue
                    counter += 1
                    chunk = _section_from_polygon(piece, counter, bool(section.get("clipped") or nx > 1 or ny > 1))
                    if not chunk:
                        continue
                    chunk["chunk_id"] = f"frame_chunk_{counter:03d}"
                    chunk["parent_section_id"] = section["section_id"]
                    chunk["requires_axis_position"] = nx > 1 or ny > 1
                    chunks.append(chunk)
    return chunks


def _generate_section_toolpaths(
    chunks: list[dict[str, Any]],
    pass_width_mm: float,
    offset_mm: float,
    overlap_mm: float,
) -> list[dict[str, Any]]:
    from shapely.geometry import LineString, Polygon

    pass_width = max(float(pass_width_mm), 1e-6)
    offset = max(float(offset_mm), 0.0)
    overlap = min(max(float(overlap_mm), 0.0), 100.0)
    step = max(pass_width - overlap, 10.0)
    half_pass = pass_width / 2.0
    toolpaths: list[dict[str, Any]] = []

    for idx, chunk in enumerate(chunks, start=1):
        pts = chunk.get("points") or []
        if len(pts) < 3:
            continue
        poly = Polygon([(float(x), float(y)) for x, y in pts])
        if not poly.is_valid:
            from shapely.validation import make_valid
            poly = make_valid(poly)
        if poly.is_empty or poly.area <= 1e-6:
            continue

        if _should_use_oriented_pass(poly, chunk):
            points, direction, strategy = _oriented_toolpath(poly, pass_width, offset, step, half_pass)
        else:
            points, direction, strategy = _axis_aligned_toolpath(poly, pass_width, offset, step, half_pass)

        if len(points) < 2:
            continue
        toolpaths.append({
            "path_id": f"frame_path_{idx:03d}",
            "source_section_id": chunk.get("parent_section_id") or chunk.get("section_id"),
            "source_chunk_id": chunk.get("chunk_id"),
            "region_type": "computed_frame",
            "operation_type": "frame_section_pass",
            "points": points,
            "direction": direction,
            "path_strategy": strategy,
            "start_point": points[0],
            "end_point": points[-1],
        })
    return toolpaths


def _axis_aligned_toolpath(poly: Any, pass_width: float, offset: float, step: float, half_pass: float) -> tuple[list[list[float]], str, str]:
    from shapely.geometry import LineString

    minx, miny, maxx, maxy = [float(v) for v in poly.bounds]
    width = maxx - minx
    height = maxy - miny
    horizontal = width >= height
    lo_min = minx if horizontal else miny
    lo_max = maxx if horizontal else maxy
    sh_min = miny if horizontal else minx
    sh_max = maxy if horizontal else maxx
    centers = _pass_centers(sh_min, sh_max, pass_width, step, half_pass)
    long_inset = min(offset, max((lo_max - lo_min) / 2.0, 0.0))
    pa0 = lo_min + long_inset
    pa1 = lo_max - long_inset
    if pa1 - pa0 <= 1e-6:
        pa0 = lo_min
        pa1 = lo_max

    points: list[list[float]] = []
    toggle = False
    for center in centers:
        a = (pa0, center) if horizontal else (center, pa0)
        b = (pa1, center) if horizontal else (center, pa1)
        clipped = _clip_line_segment_to_polygon(LineString([a, b]), poly)
        if not clipped:
            continue
        if toggle:
            clipped = list(reversed(clipped))
        points.extend(clipped)
        toggle = not toggle
    return points, "X" if horizontal else "Y", "axis_aligned"


def _oriented_toolpath(poly: Any, pass_width: float, offset: float, step: float, half_pass: float) -> tuple[list[list[float]], str, str]:
    from shapely.geometry import LineString
    import math

    ux, uy = _major_axis(poly)
    vx, vy = -uy, ux
    coords = [(float(x), float(y)) for x, y in poly.exterior.coords[:-1]]
    long_vals = [x * ux + y * uy for x, y in coords]
    short_vals = [x * vx + y * vy for x, y in coords]
    lo_min, lo_max = min(long_vals), max(long_vals)
    sh_min, sh_max = min(short_vals), max(short_vals)
    centers = _pass_centers(sh_min, sh_max, pass_width, step, half_pass)
    long_inset = min(offset, max((lo_max - lo_min) / 2.0, 0.0))
    pa0 = lo_min + long_inset
    pa1 = lo_max - long_inset
    if pa1 - pa0 <= 1e-6:
        pa0 = lo_min
        pa1 = lo_max

    pad = max(pass_width, offset, 1.0)
    points: list[list[float]] = []
    toggle = False
    for center in centers:
        a = (ux * (pa0 - pad) + vx * center, uy * (pa0 - pad) + vy * center)
        b = (ux * (pa1 + pad) + vx * center, uy * (pa1 + pad) + vy * center)
        clipped = _clip_line_segment_to_polygon(LineString([a, b]), poly)
        if not clipped:
            continue
        # Trim the clipped segment by the requested tool offset without losing short bands.
        clipped = _trim_segment(clipped, offset) or clipped
        if toggle:
            clipped = list(reversed(clipped))
        points.extend(clipped)
        toggle = not toggle

    direction = "X" if abs(ux) >= abs(uy) else "Y"
    angle = abs(math.degrees(math.atan2(uy, ux))) % 180.0
    strategy = "diagonal" if 8.0 < angle < 82.0 or 98.0 < angle < 172.0 else "curved_band"
    return points, direction, strategy


def _pass_centers(sh_min: float, sh_max: float, pass_width: float, step: float, half_pass: float) -> list[float]:
    sh_len = sh_max - sh_min
    if sh_len <= pass_width + 1e-9:
        return [(sh_min + sh_max) / 2.0]
    s0 = sh_min + min(half_pass, sh_len / 2.0)
    s1 = sh_max - min(half_pass, sh_len / 2.0)
    if s1 <= s0 + 1e-6:
        return [(sh_min + sh_max) / 2.0]
    import math

    span = s1 - s0
    steps = max(1, int(math.ceil(span / step - 1e-9)))
    adjusted = span / steps
    return [s0 + k * adjusted for k in range(steps + 1)]


def _should_use_oriented_pass(poly: Any, chunk: dict[str, Any]) -> bool:
    if bool(chunk.get("clipped")):
        return True
    coords = list(poly.exterior.coords[:-1])
    if len(coords) != 4:
        return True
    minx, miny, maxx, maxy = [float(v) for v in poly.bounds]
    envelope_area = max((maxx - minx) * (maxy - miny), 1e-9)
    return abs(float(poly.area) - envelope_area) / envelope_area > 1e-4


def _major_axis(poly: Any) -> tuple[float, float]:
    import math

    rect = poly.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)
    best = (1.0, 0.0, -1.0)
    for a, b in zip(coords, coords[1:]):
        dx = float(b[0] - a[0])
        dy = float(b[1] - a[1])
        length = math.hypot(dx, dy)
        if length > best[2]:
            best = (dx, dy, length)
    if best[2] <= 1e-9:
        return (1.0, 0.0)
    return (best[0] / best[2], best[1] / best[2])


def _trim_segment(points: list[list[float]], trim: float) -> list[list[float]] | None:
    if trim <= 1e-9 or len(points) < 2:
        return points
    import math

    x0, y0 = points[0]
    x1, y1 = points[-1]
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length <= trim * 2.2:
        return points
    ux, uy = dx / length, dy / length
    return [[x0 + ux * trim, y0 + uy * trim], [x1 - ux * trim, y1 - uy * trim]]


def _clip_line_segment_to_polygon(line: Any, poly: Any) -> list[list[float]] | None:
    inter = line.intersection(poly)
    segments = []
    if inter.is_empty:
        return None
    if inter.geom_type == "LineString":
        segments = [inter]
    elif inter.geom_type in ("MultiLineString", "GeometryCollection"):
        segments = [g for g in inter.geoms if g.geom_type == "LineString" and g.length > 1e-6]
    if not segments:
        return None
    seg = max(segments, key=lambda g: g.length)
    coords = list(seg.coords)
    if len(coords) < 2:
        return None
    return [[float(coords[0][0]), float(coords[0][1])], [float(coords[-1][0]), float(coords[-1][1])]]
