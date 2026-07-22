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


def build_structural_curved_frame_sections(
    frame_geom: Any,
    outer_poly: Any,
    obstacle_polys: list[Any],
    pass_width: float,
) -> list[dict[str, Any]]:
    """Build operator-intended sections for rectangular pockets with one curved side.

    This path is only for Case C. It uses each pocket's structural curved-side endpoints
    as layout anchors, not every sampled DXF arc point. The output matches the intended
    model: top rail, lower curved rail, vertical gaps between pockets, and big outer side
    sections. The actual polygons are still clipped to frame_geom, so no section enters a
    pocket or 3D contour.
    """
    if frame_geom is None or getattr(frame_geom, "is_empty", True) or not obstacle_polys:
        return []
    if outer_poly is None or getattr(outer_poly, "is_empty", True):
        return []

    try:
        from shapely.geometry import Polygon, box
        from shapely.ops import unary_union
    except Exception:
        return []

    curved_infos: list[dict[str, Any]] = []
    for obstacle in obstacle_polys:
        base = _largest_polygon(obstacle)
        if base is None or getattr(base, "is_empty", True) or getattr(base, "geom_type", None) != "Polygon":
            continue
        ring = [[float(x), float(y)] for x, y in base.exterior.coords[:-1]]
        side = _curved_side_of_ring(ring)
        if not side or len(side) < 4:
            continue
        minx, miny, maxx, maxy = [float(v) for v in base.bounds]
        # Use the extreme points OF THE CURVED SIDE ITSELF, not just the run's two ends.
        # _curved_side_of_ring can return a run whose endpoints include a top corner; taking
        # those directly made curve_left/curve_right land on the pocket TOP, so the rail
        # between two such pockets collapsed to zero height and produced no pass at all.
        curve_left = min(side, key=lambda p: (float(p[0]), float(p[1])))
        curve_right = max(side, key=lambda p: (float(p[0]), float(p[1])))
        endpoints = [curve_left, curve_right]
        curved_infos.append({
            "poly": base,
            "minx": minx,
            "miny": miny,
            "maxx": maxx,
            "maxy": maxy,
            "curve_y": sum(float(p[1]) for p in endpoints) / 2.0,
            "curve_x0": min(float(p[0]) for p in endpoints),
            "curve_x1": max(float(p[0]) for p in endpoints),
            "curve_left": [float(curve_left[0]), float(curve_left[1])],
            "curve_right": [float(curve_right[0]), float(curve_right[1])],
        })

    if not curved_infos:
        return []
    curved_infos.sort(key=lambda item: (item["minx"] + item["maxx"]) / 2.0)

    ox0, oy0, ox1, oy1 = [float(v) for v in outer_poly.bounds]
    frame_height = max(oy1 - oy0, 1e-6)
    pocket_top_y = max(info["maxy"] for info in curved_infos)
    curve_y = max(info["curve_y"] for info in curved_infos)
    lower_cut_y = min(pocket_top_y - 1e-6, curve_y)

    candidates: list[tuple[Any, str, str]] = []

    # Top rail: one long pass across the upper frame rail.
    if oy1 - pocket_top_y > frame_height * 0.03:
        candidates.append((box(ox0, pocket_top_y, ox1, oy1), "top_frame_band", "centerline"))

    # Vertical frame bands between pockets: use each pocket pair's structural
    # top corners and curved-side endpoints instead of a global rectangle.
    for left, right in zip(curved_infos, curved_infos[1:]):
        if right["minx"] - left["maxx"] <= max(float(pass_width) * 0.25, 10.0):
            continue
        bottom_y = min(float(left["curve_right"][1]), float(right["curve_left"][1]))
        top_y = min(float(left["maxy"]), float(right["maxy"]))
        if top_y - bottom_y <= max(float(pass_width) * 0.25, 10.0):
            # Degenerate rail (the two pockets' reference points sit at the same height).
            # Fall back to the full gap between the pockets, clipped to the frame later,
            # rather than emitting nothing for this rib.
            bottom_y = min(float(left["miny"]), float(right["miny"]))
            top_y = max(float(left["maxy"]), float(right["maxy"]))
        if top_y - bottom_y <= 1e-6:
            continue
        rail = box(left["maxx"], bottom_y, right["minx"], top_y)
        candidates.append((rail, "between_pockets_vertical", "centerline"))


    # Lower curved rail: split by pocket spans/gaps so the centerline follows large,
    # meaningful sections instead of tiny grid fragments.
    lower_sections: list[Any] = []
    # One section below each curved pocket.
    for info in curved_infos:
        lower_sections.append(box(info["minx"], oy0, info["maxx"], lower_cut_y))
    # One section in each gap between neighboring pockets.
    for left, right in zip(curved_infos, curved_infos[1:]):
        if right["minx"] - left["maxx"] > max(float(pass_width) * 0.25, 10.0):
            lower_sections.append(box(left["maxx"], oy0, right["minx"], lower_cut_y))
    if lower_sections:
        for piece in _as_polygons(unary_union(lower_sections)):
            candidates.append((piece, "bottom_curved_band", "centerline"))

    # Outer side sections are big sections and must use zigzag fill.
    # The width threshold decides whether a side strip becomes a section at all, so it must
    # be a real "too small to sand" limit. At pass_width*0.5 a 70 mm strip beside a 144 mm
    # tool was dropped for being 2 mm under the bar, silently losing the whole right-hand
    # frame rail. Anything at least a quarter of the tool wide is worth a pass.
    side_min_width = max(float(pass_width) * 0.25, 15.0)
    first = curved_infos[0]
    last = curved_infos[-1]
    if first["minx"] - ox0 > side_min_width:
        candidates.append((box(ox0, oy0, first["minx"], oy1), "outer_left_frame", "zigzag_fill"))
    if ox1 - last["maxx"] > side_min_width:
        candidates.append((box(last["maxx"], oy0, ox1, oy1), "outer_right_frame", "zigzag_fill"))

    sections: list[dict[str, Any]] = []
    claimed = None
    top_claimed = None
    for candidate, strategy, mode in candidates:
        geom = candidate.intersection(frame_geom)
        if strategy == "between_pockets_vertical":
            # These rails must reach their structural curved-side endpoints. Do not
            # subtract the previously claimed lower curved band, or the rail gets cut
            # at an artificial horizontal level and leaves diagonal slivers.
            if top_claimed is not None:
                geom = geom.difference(top_claimed)
        elif claimed is not None:
            geom = geom.difference(claimed)
        pieces = _as_polygons(geom)
        for piece in pieces:
            if piece.is_empty or piece.area <= max(float(pass_width) * 2.0, 50.0):
                continue
            section = _section_from_polygon(piece, len(sections) + 1, True)
            if not section:
                continue
            section["section_strategy"] = strategy
            section["toolpath_mode"] = mode
            if strategy == "bottom_curved_band":
                section["curved"] = True
            sections.append(section)
        if pieces:
            unioned = unary_union(pieces)
            if strategy == "top_frame_band":
                top_claimed = unioned if top_claimed is None else unary_union([top_claimed, unioned])
            claimed = unioned if claimed is None else unary_union([claimed, unioned])

    # Require enough structure to avoid replacing the proven fallback with a partial guess.
    return sections if len(sections) >= max(3, len(curved_infos)) else []

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


def _simplify_ring(pts: list[list[float]], tol: float) -> list[list[float]]:
    """Douglas-Peucker on an open copy of a ring — removes per-vertex sampling noise while
    keeping real corners and overall arc shape."""
    import math

    if len(pts) < 3 or tol <= 0:
        return [list(p) for p in pts]

    def _dp(seq: list[list[float]]) -> list[list[float]]:
        if len(seq) < 3:
            return seq
        ax, ay = seq[0]
        bx, by = seq[-1]
        dx, dy = bx - ax, by - ay
        base = math.hypot(dx, dy)
        dmax, idx = -1.0, 0
        for i in range(1, len(seq) - 1):
            px, py = seq[i]
            d = math.hypot(px - ax, py - ay) if base < 1e-9 else abs(dy * px - dx * py + bx * ay - by * ax) / base
            if d > dmax:
                dmax, idx = d, i
        if dmax > tol:
            return _dp(seq[: idx + 1])[:-1] + _dp(seq[idx:])
        return [seq[0], seq[-1]]

    out = _dp([list(p) for p in pts] + [list(pts[0])])
    if len(out) >= 2 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < 1e-9:
        out = out[:-1]
    return out if len(out) >= 3 else [list(p) for p in pts]


def _curved_side_of_ring(ring: list[list[float]], angle_deg: float = 18.0, min_pts: int = 4) -> list[list[float]] | None:
    """Return the run of vertices forming a CURVED side of a pocket, or None.

    A curved side is a chain of vertices between two sharp corners along which the
    boundary only bends gently (arc flattening). The two bounding corners are exactly the
    anchor points an operator would pick to say "the curved part runs from here to here".
    """
    import math

    raw = [[float(x), float(y)] for x, y in ring]
    if len(raw) >= 2 and math.hypot(raw[0][0] - raw[-1][0], raw[0][1] - raw[-1][1]) < 1e-6:
        raw = raw[:-1]
    if len(raw) < min_pts + 2:
        return None

    # Classify on a DE-NOISED copy of the ring. Real DXF rings carry per-vertex noise
    # (duplicated lines, tiny gaps, offset contours), and that noise — not the true shape —
    # is what previously decided whether a curved side was found. Douglas-Peucker with a
    # small tolerance removes the jitter while preserving genuine corners and arc shape.
    # Anchors are then mapped back onto the ORIGINAL vertices, so the band still uses real
    # geometry.
    span0 = max(
        max(p[0] for p in raw) - min(p[0] for p in raw),
        max(p[1] for p in raw) - min(p[1] for p in raw),
        1.0,
    )
    pts = _simplify_ring(raw, tol=max(span0 * 0.004, 1.5))
    n = len(pts)
    if n < min_pts + 2:
        pts = raw
        n = len(pts)
    if n < min_pts + 2:
        return None

    def turn_at(i: int) -> float:
        a, b, c = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
        v1x, v1y = b[0] - a[0], b[1] - a[1]
        v2x, v2y = c[0] - b[0], c[1] - b[1]
        l1, l2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            return 0.0
        dot = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
        return math.degrees(math.acos(dot))

    def deviation_at(i: int) -> float:
        """Perpendicular distance from vertex i to the chord through its neighbours.

        Angle alone is NOT a safe corner test on real DXF data: a noisy vertex on a short
        segment swings the angle wildly (45 deg from 5 mm of jitter) while barely leaving
        the arc. That shredded smooth arcs into fake corners, so the curved side was found
        on clean files and silently lost on messy ones — the intermittent failure. Distance
        is scale-honest: a real corner departs by millimetres, sampling noise does not."""
        a, b, c = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
        dx, dy = c[0] - a[0], c[1] - a[1]
        chord = math.hypot(dx, dy)
        if chord < 1e-9:
            return 0.0
        return abs(dy * b[0] - dx * b[1] + c[0] * a[1] - c[1] * a[0]) / chord

    # A vertex is a real corner only when it turns sharply AND actually departs from the
    # local boundary. `min_dev` scales with the ring so it adapts to model size.
    span = max(
        max(p[0] for p in pts) - min(p[0] for p in pts),
        max(p[1] for p in pts) - min(p[1] for p in pts),
        1.0,
    )
    min_dev = max(span * 0.01, 3.0)
    is_corner = [turn_at(i) > angle_deg and deviation_at(i) > min_dev for i in range(n)]
    if not any(is_corner):
        return None

    # Longest run of non-corner (gently bending) vertices, inclusive of its end corners.
    best: list[list[float]] | None = None
    for start in range(n):
        if not is_corner[start]:
            continue
        run = [pts[start]]
        k = (start + 1) % n
        guard = 0
        while not is_corner[k] and guard < n:
            run.append(pts[k])
            k = (k + 1) % n
            guard += 1
        run.append(pts[k])  # closing corner
        if len(run) >= min_pts and (best is None or len(run) > len(best)):
            best = run
    return best


def curved_pocket_bands(obstacle_polys: list[Any], outer_poly: Any, frame_geom: Any,
                        pass_width: float) -> list[Any]:
    """Frame bands along pockets' CURVED sides, anchored on the pocket corner points.

    Operator's rule: take the two corner points bounding a pocket's curved side, follow the
    arc between them, and the frame band under that arc is one section whose toolpath is the
    band's centerline. This needs no curvature classification of the frame itself — the
    pocket corners are exact, known anchors — and it works whether or not the surrounding
    rectangular sections happen to contain curved parts.

    Returns one band polygon per curved pocket side (empty list when no pocket is curved).
    """
    from shapely.geometry import Polygon

    if not obstacle_polys or frame_geom is None or getattr(frame_geom, "is_empty", True):
        return []
    if outer_poly is None or getattr(outer_poly, "is_empty", True):
        return []

    outer_ring = list(outer_poly.exterior.coords[:-1]) if getattr(outer_poly, "geom_type", None) == "Polygon" else []
    if len(outer_ring) < 3:
        return []

    bands: list[Any] = []
    for p in obstacle_polys:
        base = _largest_polygon(p)
        if base is None or getattr(base, "is_empty", True) or getattr(base, "geom_type", None) != "Polygon":
            continue
        side = _curved_side_of_ring([[float(x), float(y)] for x, y in base.exterior.coords[:-1]])
        if not side or len(side) < 4:
            continue
        # Project the curved side outward onto the nearest part of the door outline, then
        # close the ring: pocket curved side + matching outline stretch = the frame band.
        try:
            outline = _nearest_outline_run(outer_ring, side)
            if outline is None or len(outline) < 2:
                continue
            band = Polygon(list(side) + list(reversed(outline)))
            if not band.is_valid:
                band = band.buffer(0)
            band = band.intersection(frame_geom)
        except Exception:
            continue
        for g in _as_polygons(band):
            if not g.is_empty and g.area > max(pass_width * 5.0, 100.0):
                bands.append(g)
    return bands

def _nearest_outline_run(outer_ring: list[Any], side: list[list[float]]) -> list[list[float]] | None:
    """The stretch of the door outline that faces `side` (a pocket's curved side)."""
    import math

    ring = [[float(x), float(y)] for x, y in outer_ring]
    if len(ring) < 2 or len(side) < 2:
        return None

    def nearest_index(pt: list[float]) -> int:
        best_i, best_d = 0, None
        for i, q in enumerate(ring):
            d = math.hypot(q[0] - pt[0], q[1] - pt[1])
            if best_d is None or d < best_d:
                best_i, best_d = i, d
        return best_i

    i0 = nearest_index(side[0])
    i1 = nearest_index(side[-1])
    n = len(ring)
    # Walk the shorter way around the ring between the two anchor projections.
    fwd = [(i0 + k) % n for k in range((i1 - i0) % n + 1)]
    bwd = [(i0 - k) % n for k in range((i0 - i1) % n + 1)]
    run_idx = fwd if len(fwd) <= len(bwd) else bwd
    if len(run_idx) < 2:
        return None
    return [ring[i] for i in run_idx]


def diagonal_zone_box(obstacle_polys: list[Any], frame_geom: Any, margin: float = 0.0) -> Any:
    """Axis-aligned box that tightly encloses every obstacle having a sloped (non-axis)
    edge — the "diagonal zone".

    Splitting the frame into (a) this box and (b) everything outside it isolates the
    triangular/diagonal problem from the ordinary rectangular perimeter strips. Inside the
    box the frame is treated as diagonal bands (one p1->p2 stroke per arm); outside it the
    familiar rectangular section logic applies. Returns None when the model has no sloped
    obstacle (a purely rectangular door), so those models keep the existing behaviour.
    """
    from shapely.geometry import Polygon, box

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
    from shapely.geometry import Polygon, box

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


def _band_supports_straight_pass(base: Any, min_frac: float = 0.55) -> bool:
    """True when a straight centerline already covers most of this band's length.

    Splitting is only worth doing for a band that would otherwise emit NO pass. If a
    straight stroke already works, quadrant-slicing it just produces short offcuts that are
    then too short to survive the tool offset — which is how a clean 4-band frame turned
    into 7 sections with 2 of them empty."""
    import math
    from shapely.geometry import LineString

    try:
        ux, uy = _major_axis(base)
        coords = list(base.exterior.coords[:-1])
        longs = [x * ux + y * uy for x, y in coords]
        span = max(longs) - min(longs)
        if span <= 1e-6:
            return True
        cx, cy = base.centroid.x, base.centroid.y
        reach = span * 2.0
        line = LineString([(cx - ux * reach, cy - uy * reach), (cx + ux * reach, cy + uy * reach)])
        inter = line.intersection(base)
        if inter.is_empty:
            return False
        if inter.geom_type == "LineString":
            covered = inter.length
        else:
            parts = [g.length for g in getattr(inter, "geoms", []) if g.geom_type == "LineString"]
            covered = max(parts) if parts else 0.0
        return covered >= span * min_frac
    except Exception:
        return True


def _split_corner_band_into_legs(poly: Any) -> list[Any]:
    """Cut an L/U-shaped thin band into its straight legs.

    Uses the band's own bounding box: a corner-wrapping band leaves most of the box empty.
    We slice the band with the axis-aligned half-planes at the bend so each leg becomes a
    simple straight strip. Returns [poly] unchanged when the band is already straight."""
    from shapely.geometry import Polygon, box

    base = _largest_polygon(poly)
    # Only split when the band genuinely cannot be covered by one straight stroke.
    if base is not None and getattr(base, "geom_type", None) == "Polygon" and _band_supports_straight_pass(base):
        return [poly]
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
