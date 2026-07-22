from __future__ import annotations

from typing import Any

from .frame_classification import (
    _dedupe_consecutive,
    _dedupe_ring,
    _major_axis,
    _ring_has_curve,
    _ring_has_non_axis_edge,
)
from .frame_geometry import _as_polygons, _largest_polygon


def _section_from_polygon(poly: Any, index: int, clipped: bool) -> dict[str, Any] | None:
    if poly.is_empty or poly.area <= 1e-6:
        return None
    if poly.geom_type != "Polygon":
        polys = _as_polygons(poly)
        if not polys:
            return None
        poly = polys[0]
    raw_exterior = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
    has_curve = _ring_has_curve(raw_exterior)
    has_sloped_edge = bool(clipped) and _ring_has_non_axis_edge(raw_exterior)
    # Rectangular sections should collapse to clean corners, but curved/sloped clipped
    # sections must keep their real boundary so the preview does not become a fake box.
    if not (has_curve or has_sloped_edge):
        simplified = poly.simplify(2.0, preserve_topology=True)
        if simplified is not None and not simplified.is_empty and simplified.geom_type == "Polygon":
            poly = simplified
        exterior = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
        exterior = _dedupe_ring(exterior)
    else:
        exterior = _dedupe_consecutive(raw_exterior)
    holes = []
    for interior in poly.interiors:
        hole = [[float(x), float(y)] for x, y in interior.coords[:-1]]
        hole = _dedupe_ring(hole)
        if len(hole) >= 3:
            holes.append(hole)
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
        "holes": holes,
        "bbox": {"min_x": minx, "min_y": miny, "max_x": maxx, "max_y": maxy},
        "width": width,
        "height": height,
        "orientation": "horizontal" if width >= height else "vertical",
        "covered": True,
        "clipped": bool(clipped or len(exterior) > 4),
        "curved": bool(has_curve),
        "area": float(poly.area),
        "region_type": "computed_frame",
    }


def _polygon_from_section(section: dict[str, Any]) -> Any:
    from shapely.geometry import Polygon
    from shapely.validation import make_valid

    pts = section.get("points") or []
    if len(pts) < 3:
        return None
    holes = []
    for hole in section.get("holes") or []:
        if len(hole) >= 3:
            holes.append([(float(x), float(y)) for x, y in hole])
    poly = Polygon([(float(x), float(y)) for x, y in pts], holes)
    if not poly.is_valid:
        poly = make_valid(poly)
    poly = _largest_polygon(poly)
    if poly is None or poly.is_empty or getattr(poly, "area", 0.0) <= 1e-6:
        return None
    return poly


def _split_sections_by_reach(sections: list[dict[str, Any]], reach_x: float, reach_y: float) -> list[dict[str, Any]]:
    from shapely.geometry import box

    chunks: list[dict[str, Any]] = []
    counter = 0
    for section in sections:
        poly = _polygon_from_section(section)
        if poly is None:
            continue
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


def _outer_non_axis_boundary_regions(
    outer_polygon: list[list[float]] | None,
    frame_geom: Any,
    pass_width_mm: float,
    offset_mm: float,
) -> list[Any]:
    """Return real frame bands adjacent to curved/sloped OUTER door edges.

    This is intentionally model-agnostic. If the uploaded outline has a curved/sloped
    boundary, the frame area beside that boundary must remain a curved/sloped polygon;
    it should not be converted into rectangular grid cells. We buffer only those outer
    boundary chains, then clip the buffer to the already-computed frame surface, so the
    result cannot enter pockets or 3D contour regions.
    """
    if not outer_polygon or len(outer_polygon) < 3 or frame_geom is None or getattr(frame_geom, "is_empty", True):
        return []

    try:
        from shapely.geometry import LineString
        from shapely.ops import unary_union
    except Exception:
        return []

    import math

    pts = [[float(x), float(y)] for x, y in outer_polygon]
    if len(pts) >= 2 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) <= 1e-6:
        pts = pts[:-1]
    if len(pts) < 3:
        return []

    # Mark only meaningful non-horizontal/non-vertical boundary segments. Curved DXF
    # edges are flattened into many small sloped segments, so those become one chain.
    flags: list[bool] = []
    edges: list[tuple[list[float], list[float]]] = []
    angle_tol_deg = 1.5
    for idx, a in enumerate(pts):
        b = pts[(idx + 1) % len(pts)]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)
        edges.append((a, b))
        if length < 1.0:
            flags.append(False)
            continue
        angle = abs(math.degrees(math.atan2(dy, dx))) % 180.0
        nearest_axis = min(abs(angle), abs(angle - 90.0), abs(angle - 180.0))
        flags.append(nearest_axis > angle_tol_deg)

    if not any(flags):
        return []

    # Group consecutive non-axis segments, including wrap-around continuity.
    n = len(flags)
    visited = [False] * n
    chains: list[list[list[float]]] = []
    for start in range(n):
        if visited[start] or not flags[start]:
            continue
        chain_edges: list[int] = []
        idx = start
        while flags[idx] and not visited[idx]:
            visited[idx] = True
            chain_edges.append(idx)
            idx = (idx + 1) % n
        if not chain_edges:
            continue
        chain = [edges[chain_edges[0]][0]]
        for edge_idx in chain_edges:
            chain.append(edges[edge_idx][1])
        chain_len = sum(math.hypot(chain[i + 1][0] - chain[i][0], chain[i + 1][1] - chain[i][1]) for i in range(len(chain) - 1))
        # Only arc-like chains belong here. A single sloped/chamfer edge is still a
        # normal frame boundary and should be handled by the rectangular/grid sections.
        if len(chain_edges) >= 3 and chain_len >= max(float(pass_width_mm) * 0.75, 30.0):
            chains.append(chain)

    if not chains:
        return []

    band_depth = max(float(pass_width_mm) * 1.05, float(offset_mm) + 10.0, 80.0)
    regions: list[Any] = []
    for chain in chains:
        if len(chain) < 2:
            continue
        line = LineString([(p[0], p[1]) for p in chain])
        if line.length < max(float(pass_width_mm) * 0.75, 30.0):
            continue
        # Flat caps avoid artificial round lobes at the ends of the frame section.
        band = line.buffer(band_depth, cap_style=2, join_style=2)
        clipped = frame_geom.intersection(band)
        for piece in _as_polygons(clipped):
            if not piece.is_empty and piece.area > max(float(pass_width_mm) * 10.0, 100.0):
                regions.append(piece)

    if not regions:
        return []
    return _as_polygons(unary_union(regions))


def diagonal_zone_box(obstacle_polys: list[Any], frame_geom: Any, margin: float = 0.0) -> Any:
    """Axis-aligned box that tightly encloses every obstacle having a sloped (non-axis)
    edge — the "diagonal zone".

    Splitting the frame into (a) this box and (b) everything outside it isolates the
    triangular/diagonal problem from the ordinary rectangular perimeter strips. Inside the
    box the frame is treated as diagonal bands (one p1->p2 stroke per arm); outside it the
    familiar rectangular section logic applies. Returns None when the model has no sloped
    obstacle (a purely rectangular door), so those models keep the existing behaviour.
    """
    from shapely.geometry import box

    if not obstacle_polys or frame_geom is None or getattr(frame_geom, "is_empty", True):
        return None

    xs0 = ys0 = None
    xs1 = ys1 = None
    for p in obstacle_polys:
        base = _largest_polygon(p)
        if base is None or getattr(base, "is_empty", True) or getattr(base, "geom_type", None) != "Polygon":
            continue
        ring = [[float(x), float(y)] for x, y in base.exterior.coords[:-1]]
        # Only obstacles with a genuinely sloped edge define the diagonal zone. A curved
        # edge is Case C and is handled by the curved-band path, not here.
        if not _ring_has_non_axis_edge(ring) or _ring_has_curve(ring):
            continue
        bx0, by0, bx1, by1 = [float(v) for v in base.bounds]
        xs0 = bx0 if xs0 is None else min(xs0, bx0)
        ys0 = by0 if ys0 is None else min(ys0, by0)
        xs1 = bx1 if xs1 is None else max(xs1, bx1)
        ys1 = by1 if ys1 is None else max(ys1, by1)

    if xs0 is None or xs1 is None or ys0 is None or ys1 is None:
        return None
    if xs1 - xs0 <= 1e-6 or ys1 - ys0 <= 1e-6:
        return None

    zone = box(xs0 - margin, ys0 - margin, xs1 + margin, ys1 + margin)
    # Clamp to the frame's own extent so the zone never reaches outside the door.
    fx0, fy0, fx1, fy1 = [float(v) for v in frame_geom.bounds]
    zone = zone.intersection(box(fx0, fy0, fx1, fy1))
    return zone if not zone.is_empty and zone.area > 1e-6 else None


def split_diagonal_zone_into_arms(zone_frame: Any, zone_box: Any, pass_width: float) -> list[Any]:
    """Split the frame inside the diagonal zone into its individual arms.

    Inside the zone the frame is an X (or V): several straight arms meeting at a junction.
    They form ONE connected polygon, so a single centerline would be a chord across the
    whole X. We cut the zone into quadrants about the junction — each quadrant holds one
    arm — then keep the arm pieces. The junction is the zone-box centre, which is where the
    sloped obstacles meet by construction.
    """
    from shapely.geometry import box

    if zone_frame is None or getattr(zone_frame, "is_empty", True) or zone_box is None:
        return []
    zx0, zy0, zx1, zy1 = [float(v) for v in zone_box.bounds]
    cx, cy = (zx0 + zx1) / 2.0, (zy0 + zy1) / 2.0
    quads = [
        box(zx0, zy0, cx, cy), box(cx, zy0, zx1, cy),
        box(zx0, cy, cx, zy1), box(cx, cy, zx1, zy1),
    ]
    arms: list[Any] = []
    for q in quads:
        piece = zone_frame.intersection(q)
        for g in _as_polygons(piece):
            if g.is_empty or g.area <= max(pass_width * 2.0, 1.0):
                continue
            # Each quadrant piece may still hold a stub of a neighbouring arm; keep the
            # dominant connected part only.
            arms.append(g)
    return arms


def _polygon_is_curved_or_sloped(poly: Any) -> bool:
    if poly is None or getattr(poly, "is_empty", True) or getattr(poly, "geom_type", None) != "Polygon":
        return False
    exterior = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
    return _ring_has_curve(exterior) or _ring_has_non_axis_edge(exterior)


def _merge_blocks_into_curved_regions(partial_regions: list[Any], block_geoms: list[Any], pass_width: float) -> tuple[list[Any], list[Any]]:
    """Attach adjacent rectangular cells to curved/sloped frame regions.

    Without this, a curved side is displayed as one curved sliver plus a separate
    rectangle immediately above it. Operators see that as the algorithm ignoring the real
    shape. This keeps the computed frame shape intact by merging only directly-touching
    blocks into curved/sloped regions; unrelated rectangular frame areas stay rectangular.
    """
    if not partial_regions or not block_geoms:
        return partial_regions, block_geoms

    try:
        from shapely.ops import unary_union
    except Exception:
        return partial_regions, block_geoms

    curved = [p for p in partial_regions if _polygon_is_curved_or_sloped(_largest_polygon(p))]
    straight = [p for p in partial_regions if not _polygon_is_curved_or_sloped(_largest_polygon(p))]
    if not curved:
        return partial_regions, block_geoms

    remaining_blocks = list(block_geoms)
    attached: list[Any] = []
    # One direct layer is intentional. It captures the rectangular frame band sitting on
    # the curved boundary without flood-filling the whole door into one section.
    for region in curved:
        base = _largest_polygon(region)
        if base is None or getattr(base, "is_empty", True):
            continue
        group = [base]
        minx, miny, maxx, maxy = [float(v) for v in base.bounds]
        next_remaining = []
        for block in remaining_blocks:
            b = _largest_polygon(block)
            if b is None or getattr(b, "is_empty", True):
                continue
            bx0, by0, bx1, by1 = [float(v) for v in b.bounds]
            overlaps_axis = not (bx1 < minx - 1e-6 or bx0 > maxx + 1e-6 or by1 < miny - 1e-6 or by0 > maxy + float(pass_width) * 2.0)
            touches_region = b.distance(base) <= 1e-6
            if overlaps_axis and touches_region:
                group.append(b)
            else:
                next_remaining.append(block)
        remaining_blocks = next_remaining
        merged = unary_union(group)
        for piece in _as_polygons(merged):
            if not piece.is_empty and piece.area > 1e-3:
                attached.append(piece)

    return straight + attached, remaining_blocks


def _split_curved_band_by_geometry(poly: Any, pass_width: float, reach_x: float, reach_y: float) -> list[Any]:
    """Split a long curved frame band by its own geometry, not by the uploaded model.

    The frame boolean can return one connected curved band across a door. That is valid
    geometry, but not a useful robot section. This splitter projects the band on its own
    major axis and cuts it into reach-sized windows. No pocket count, model type, or
    obstacle-specific coordinates are used, so it behaves the same for any DXF shape.
    """
    if poly is None or getattr(poly, "is_empty", True):
        return []

    try:
        from shapely.geometry import Polygon
    except Exception:
        return [poly]

    base = _largest_polygon(poly)
    if base is None or getattr(base, "is_empty", True) or getattr(base, "area", 0.0) <= 1e-6:
        return []
    if getattr(base, "geom_type", None) != "Polygon":
        return _as_polygons(base)

    exterior = [[float(x), float(y)] for x, y in base.exterior.coords[:-1]]
    if not _ring_has_curve(exterior):
        return [base]

    import math

    ux, uy = _major_axis(base)
    vx, vy = -uy, ux
    long_vals = [x * ux + y * uy for x, y in exterior]
    short_vals = [x * vx + y * vy for x, y in exterior]
    long_min, long_max = min(long_vals), max(long_vals)
    short_min, short_max = min(short_vals), max(short_vals)
    long_len = long_max - long_min
    short_len = short_max - short_min
    if long_len <= max(float(pass_width) * 3.0, 1.0):
        return [base]

    # Reach available along the section axis. This supports horizontal, vertical, and
    # diagonal bands without model-specific X/Y cuts.
    axis_reach = abs(ux) * max(float(reach_x), 1.0) + abs(uy) * max(float(reach_y), 1.0)
    # Do not let a shallow curved band stay as one long rectangle-looking section just
    # because it is barely below J7 reach. Use a generic max span based on tool width and
    # reach so curved frame bands become a few useful p1->p2 diagonal sections.
    target_span = min(axis_reach, max(float(pass_width) * 5.0, 350.0))
    target_span = max(target_span, float(pass_width) * 3.0)
    if target_span <= 1e-6 or long_len <= target_span:
        return [base]

    split_count = max(2, int(math.ceil(long_len / target_span)))
    # Avoid making short noisy pieces. If the computed split would create sub-tool chunks,
    # keep one band and let the centerline logic produce one straight pass.
    if long_len / split_count < float(pass_width) * 2.0:
        return [base]

    pad = max(long_len, short_len, float(pass_width), float(reach_x), float(reach_y), 1.0) + 20.0
    pieces: list[Any] = []
    for idx in range(split_count):
        a = long_min + long_len * idx / split_count
        b = long_min + long_len * (idx + 1) / split_count
        if b - a <= 1e-6:
            continue
        strip = Polygon([
            (ux * a + vx * (short_min - pad), uy * a + vy * (short_min - pad)),
            (ux * b + vx * (short_min - pad), uy * b + vy * (short_min - pad)),
            (ux * b + vx * (short_max + pad), uy * b + vy * (short_max + pad)),
            (ux * a + vx * (short_max + pad), uy * a + vy * (short_max + pad)),
        ])
        clipped = base.intersection(strip)
        for piece in _as_polygons(clipped):
            if not piece.is_empty and piece.area > 1e-3:
                pieces.append(piece)

    if not pieces:
        return [base]
    split_area = sum(float(p.area) for p in pieces)
    if split_area < float(base.area) * 0.98:
        return [base]

    def _sort_key(g: Any) -> float:
        c = g.centroid
        return float(c.x) * ux + float(c.y) * uy

    return sorted(pieces, key=_sort_key)


def _split_band_into_strips(poly: Any) -> list[Any]:
    """Decompose a merged frame band into individual straight strips, one per arm.

    Diagonal arms that meet at the X centre get unioned into ONE self-crossing polygon
    (a "bowtie" holding two arms). shapely's buffer(0) repairs the self-intersection and
    naturally splits it into the separate simple polygons — one clean arm each — where
    make_valid only degenerates it to lines. Each resulting simple polygon has a single
    well-defined axis, so its centerline follows the arm's true diagonal."""
    if poly is None or getattr(poly, "is_empty", True):
        return []

    # First split any self-INTERSECTION (buffer(0)), then split any point-PINCH where two
    # arms only touch at the X centre. A pinch survives buffer(0) (the polygon stays
    # "valid" as a single ring), so we separate it with a morphological OPENING: erode by
    # a small epsilon to break the 1-point neck, then dilate back. eps is tiny relative to
    # the band so the strips keep their true width.
    from shapely.geometry import Polygon as _P  # noqa: F401 (kept for parity/clarity)

    def _pieces(g: Any) -> list[Any]:
        if g is None or g.is_empty or g.area <= 1e-6:
            return []
        base = g if g.is_valid else g.buffer(0)
        polys = [p for p in _as_polygons(base) if not p.is_empty and p.area > 1e-3]
        out: list[Any] = []
        for p in polys:
            minx, miny, maxx, maxy = p.bounds
            eps = max(min(maxx - minx, maxy - miny) * 0.02, 0.5)
            opened = p.buffer(-eps).buffer(eps * 1.02)
            sub = [q for q in _as_polygons(opened) if not q.is_empty and q.area > 1e-3]
            if len(sub) > 1:
                # opening disconnected the arms → keep each, re-clipped to the original
                for q in sub:
                    clip = q.intersection(p)
                    for r in _as_polygons(clip):
                        if not r.is_empty and r.area > 1e-3:
                            out.append(r)
            else:
                out.append(p)
        return out

    parts: list[Any] = []
    for g in _as_polygons(poly):
        parts.extend(_pieces(g))
    if not parts:
        parts = _as_polygons(poly)

    # A band can also WRAP A CORNER (an L / U shape: e.g. the bottom edge continuing up the
    # right edge). No pinch exists to open, but a single straight centerline cannot cover
    # an L — it exits the polygon and yields NO pass at all, leaving that frame area
    # uncovered. Cut such a band at its bend into straight legs, each of which then gets
    # its own clean p1->p2 stroke.
    legs: list[Any] = []
    for p in parts:
        legs.extend(_split_corner_band_into_legs(p))
    return legs if legs else parts


def _split_corner_band_into_legs(poly: Any) -> list[Any]:
    """Cut an L/U-shaped thin band into its straight legs.

    Uses the band's own bounding box: a corner-wrapping band leaves most of the box empty.
    We slice the band with the axis-aligned half-planes at the bend so each leg becomes a
    simple straight strip. Returns [poly] unchanged when the band is already straight."""
    from shapely.geometry import box

    base = _largest_polygon(poly)
    if base is None or getattr(base, "is_empty", True) or getattr(base, "geom_type", None) != "Polygon":
        return [poly]
    minx, miny, maxx, maxy = [float(v) for v in base.bounds]
    bw, bh = maxx - minx, maxy - miny
    if bw <= 1e-6 or bh <= 1e-6:
        return [poly]
    envelope = bw * bh
    if envelope <= 1e-9:
        return [poly]
    fill = float(base.area) / envelope
    # A straight strip fills most of its (thin) box; an L/U fills only a small fraction of
    # a near-square box. Only treat clearly corner-wrapping bands.
    if fill > 0.55 or min(bw, bh) < 1e-6:
        return [poly]
    # Only AXIS-ALIGNED bands are cut into legs here. A DIAGONAL band (Case B) is a single
    # straight strip that must stay whole — slicing it would chop one arm into several
    # short passes. Curved bands (Case C) likewise stay whole.
    exterior = [[float(x), float(y)] for x, y in base.exterior.coords[:-1]]
    if _ring_has_non_axis_edge(exterior) or _ring_has_curve(exterior):
        return [poly]

    # Slice into the 4 box quadrants and keep the non-empty pieces; each quadrant piece of
    # an L/U band is a straight leg. Merge pieces that are collinear neighbours.
    midx = (minx + maxx) / 2.0
    midy = (miny + maxy) / 2.0
    quads = [
        box(minx, miny, midx, midy), box(midx, miny, maxx, midy),
        box(minx, midy, midx, maxy), box(midx, midy, maxx, maxy),
    ]
    out: list[Any] = []
    for q in quads:
        piece = base.intersection(q)
        for r in _as_polygons(piece):
            if not r.is_empty and r.area > 1e-3:
                out.append(r)
    return out if len(out) > 1 else [poly]
