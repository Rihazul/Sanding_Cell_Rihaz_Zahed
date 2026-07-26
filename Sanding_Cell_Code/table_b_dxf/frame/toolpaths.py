from __future__ import annotations

from typing import Any

from .classification import (
    _major_axis,
    _ring_has_curve,
    _ring_has_non_axis_edge,
)
from .geometry import _largest_polygon
from .section_builder import _polygon_from_section


def _generate_section_toolpaths(
    chunks: list[dict[str, Any]],
    pass_width_mm: float,
    offset_mm: float,
    overlap_mm: float,
) -> list[dict[str, Any]]:

    pass_width = max(float(pass_width_mm), 1e-6)
    offset = max(float(offset_mm), 0.0)
    overlap = min(max(float(overlap_mm), 0.0), 100.0)
    step = max(pass_width - overlap, 10.0)
    half_pass = pass_width / 2.0
    toolpaths: list[dict[str, Any]] = []

    for idx, chunk in enumerate(chunks, start=1):
        poly = _polygon_from_section(chunk)
        if poly is None:
            continue
        if poly.is_empty or poly.area <= 1e-6:
            continue

        if _should_use_oriented_pass(poly, chunk):
            points, direction, strategy = _oriented_toolpath(poly, pass_width, offset, step, half_pass)
        else:
            points, direction, strategy = _axis_aligned_toolpath(poly, pass_width, offset, step, half_pass)

        if len(points) < 2 or not _polyline_stays_in_polygon(points, poly) or _polyline_uses_polygon_boundary(points, poly):
            continue
        toolpaths.append({
            "path_id": f"frame_path_{idx:03d}",
            "source_section_id": chunk.get("parent_section_id") or chunk.get("section_id"),
            "source_section_strategy": chunk.get("source_section_strategy") or chunk.get("section_strategy"),
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
        return [], "X" if horizontal else "Y", "axis_aligned_offset_too_short"

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


def _longest_line_part(geom: Any) -> Any:
    """Return the single longest LineString part of a line∩polygon result, or None.

    line.intersection(polygon) can be a LineString, MultiLineString, or a
    GeometryCollection (with stray points where the line touches a vertex). We want the
    one longest continuous segment that lies inside the polygon."""
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type == "LineString":
        return geom if geom.length > 1e-9 else None
    parts = []
    for g in getattr(geom, "geoms", []):
        if g.geom_type == "LineString" and g.length > 1e-9:
            parts.append(g)
    if not parts:
        return None
    return max(parts, key=lambda g: g.length)


def _split_curve_at_apex(points: list[list[float]], min_len: float) -> list[list[list[float]]]:
    """Split a curved lower frame band into two long straight strokes.

    The curved rail should be represented as the two operator-visible passes: left side to
    apex and apex to right side. Do not add a short middle connector; that creates the
    extra small path and weird crossing seen in the preview.

    Returns [] when the path is effectively straight, so a flat band keeps one stroke.
    """
    import math

    if len(points) < 3:
        return []
    ax, ay = points[0]
    bx, by = points[-1]
    dx, dy = bx - ax, by - ay
    chord = math.hypot(dx, dy)
    if chord < max(min_len * 2.0, 1e-6):
        return []

    apex_i, apex_d = 0, -1.0
    for i in range(1, len(points) - 1):
        px, py = points[i]
        d = abs(dy * px - dx * py + bx * ay - by * ax) / chord
        if d > apex_d:
            apex_i, apex_d = i, d
    if apex_d < max(min_len * 0.15, 3.0) or apex_i <= 0 or apex_i >= len(points) - 1:
        return []

    apex = [float(points[apex_i][0]), float(points[apex_i][1])]
    left = [list(points[0]), apex]
    right = [apex, list(points[-1])]
    strokes = []
    for stroke in (left, right):
        if math.dist(stroke[0], stroke[-1]) >= max(min_len * 0.25, 5.0):
            strokes.append(stroke)
    return strokes if len(strokes) == 2 else []


def _curved_band_following_centerline(poly: Any, pass_width: float, offset: float) -> list[list[float]]:
    """Centerline of a curved frame band that FOLLOWS the arc.

    Used for the structural lower band of a curved door. A straight chord at a fixed y
    drifts out of the band near its ends (and can land inside a pocket); sampling the
    band's cross-section centres along its length keeps the stroke inside and bending with
    the curve. Returns [] when no usable stroke can be built, so the caller can skip.
    """
    import math

    base = _largest_polygon(poly)
    if base is None or getattr(base, "is_empty", True) or getattr(base, "geom_type", None) != "Polygon":
        return []
    minx, miny, maxx, maxy = [float(v) for v in base.bounds]
    long_len = max(maxx - minx, 1e-6)

    stations = _curved_band_centerline(base, 1.0, 0.0, long_len, pass_width, 0.0)
    if not stations or len(stations) < 2:
        return []
    stations = _smooth_polyline(stations, window=7)

    # How thick is the band? On a THIN curved band the straight chords between simplified
    # stations bow outside the polygon, the containment check rejects the whole path, and
    # the band silently gets NO pass at all. So scale the simplify tolerance to the band's
    # own thickness and, if the result still leaves the band, retry with progressively
    # finer detail before giving up.
    thickness = float(base.area) / max(long_len, 1e-6)
    eff_offset = min(float(offset), max(long_len * 0.2, 0.0))

    tolerances = [
        max(min(pass_width * 0.06, thickness * 0.25), 1.0),
        max(thickness * 0.12, 0.5),
        max(thickness * 0.05, 0.25),
        0.0,  # last resort: keep every sampled station
    ]
    for tol in tolerances:
        pts = _simplify_polyline(stations, tol=tol) if tol > 0 else [list(p) for p in stations]
        pts = _trim_polyline(pts, eff_offset)
        if len(pts) < 2 or math.dist(pts[0], pts[-1]) <= 1e-6:
            continue
        if _polyline_stays_in_polygon(pts, base):
            return pts
        trimmed = _trim_polyline(pts, max(pass_width * 0.25, 5.0))
        if len(trimmed) >= 2 and _polyline_stays_in_polygon(trimmed, base):
            return trimmed

    # The band is CONCAVE (it dips up between pockets), so a chord between two perfectly
    # valid centre points can still cut outside it. Rejecting the whole path for that left
    # the band with no pass at all. Instead keep the longest run of consecutive stations
    # whose segments all stay inside — a slightly shorter stroke that is actually valid
    # beats emitting nothing.
    best: list[list[float]] = []
    run: list[list[float]] = [list(stations[0])]
    for a, b in zip(stations, stations[1:]):
        if _polyline_stays_in_polygon([a, b], base):
            run.append(list(b))
        else:
            if len(run) > len(best):
                best = run
            run = [list(b)]
    if len(run) > len(best):
        best = run
    if len(best) >= 2:
        best = _simplify_polyline(best, tol=max(thickness * 0.08, 0.5))
        if len(best) >= 2 and math.dist(best[0], best[-1]) > max(pass_width * 0.5, 10.0):
            return best
    return []


def _polyline_stays_in_polygon(points: list[list[float]], poly: Any, tol: float = 1e-5) -> bool:
    if len(points) < 2 or poly is None or poly.is_empty:
        return False
    try:
        from shapely.geometry import LineString

        for a, b in zip(points, points[1:]):
            line = LineString([(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))])
            if line.length <= 1e-9:
                continue
            if not line.difference(poly.buffer(tol)).is_empty:
                return False
        return True
    except Exception:
        return False




def _polyline_uses_polygon_boundary(
    points: list[list[float]],
    poly: Any,
    min_overlap: float = 1.0,
    clearance_mm: float = 0.0,
) -> bool:
    """True when a TCP path rides on, or too close along, a polygon boundary.

    Exact line overlap is always rejected. ``clearance_mm`` is used for checks against the
    real frame area, where pocket and 3D contour edges are holes. Touching one point is
    allowed; following a boundary for a measurable length is not.
    """
    if len(points) < 2 or poly is None or poly.is_empty:
        return False
    try:
        from shapely.geometry import LineString

        boundary = poly.boundary
        clearance = max(float(clearance_mm), 0.0)
        for a, b in zip(points, points[1:]):
            line = LineString([(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))])
            if line.length <= 1e-9:
                continue
            inter = line.intersection(boundary)
            if not inter.is_empty and float(getattr(inter, "length", 0.0)) > min_overlap:
                return True
            if clearance > 0.0:
                nearby = line.buffer(clearance, cap_style=2, join_style=2).intersection(boundary)
                if not nearby.is_empty and float(getattr(nearby, "length", 0.0)) > min_overlap:
                    return True
    except Exception:
        return True
    return False


def _smooth_polyline(points: list[list[float]], window: int = 5) -> list[list[float]]:
    """Moving-average smoothing of a sampled centerline.

    Cross-section midpoints wobble wherever the band's edges step (e.g. beside a pocket
    corner). Sanding wants a CLEAN path that follows the arc, not one that chases every
    sampling artefact, so we average each station with its neighbours. Endpoints are kept
    exactly so the stroke still spans the full band."""
    n = len(points)
    if n < 3 or window < 3:
        return [list(p) for p in points]
    half = max(1, window // 2)
    out: list[list[float]] = [list(points[0])]
    for i in range(1, n - 1):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        xs = sum(p[0] for p in points[lo:hi]) / (hi - lo)
        ys = sum(p[1] for p in points[lo:hi]) / (hi - lo)
        out.append([xs, ys])
    out.append(list(points[-1]))
    return out


def _simplify_polyline(points: list[list[float]], tol: float) -> list[list[float]]:
    """Drop points whose perpendicular distance from the line between their neighbours is
    below `tol` (Douglas–Peucker, iterative). A straight band collapses to its 2 ends; a
    curved band keeps just enough vertices to trace the curve."""
    import math

    if len(points) < 3:
        return [list(p) for p in points]

    def _dp(pts: list[list[float]]) -> list[list[float]]:
        if len(pts) < 3:
            return pts
        ax, ay = pts[0]
        bx, by = pts[-1]
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        dmax, idx = -1.0, 0
        for i in range(1, len(pts) - 1):
            px, py = pts[i]
            if seg < 1e-9:
                dist = math.hypot(px - ax, py - ay)
            else:
                dist = abs(dy * px - dx * py + bx * ay - by * ax) / seg
            if dist > dmax:
                dmax, idx = dist, i
        if dmax > tol:
            left = _dp(pts[: idx + 1])
            right = _dp(pts[idx:])
            return left[:-1] + right
        return [pts[0], pts[-1]]

    return _dp([list(p) for p in points])


def _curved_band_centerline(poly: Any, axis_x: float, axis_y: float, long_len: float,
                            pass_width: float, offset: float) -> list[list[float]]:
    """Centerline that FOLLOWS a curved band: step along the band's axis and, at each
    station, cut a cross-section perpendicular to the axis and take its midpoint. Connect
    the midpoints into a polyline that bends with the band. This is linear point-to-point
    coverage of a curve (the tool is wide, so the line need not be perfectly centered)."""
    from shapely.geometry import LineString
    import math

    px, py = -axis_y, axis_x  # perpendicular to the axis
    coords = list(poly.exterior.coords[:-1])
    long_vals = [x * axis_x + y * axis_y for x, y in coords]
    short_vals = [x * px + y * py for x, y in coords]
    lo_min, lo_max = min(long_vals), max(long_vals)
    sh_min, sh_max = min(short_vals), max(short_vals)
    big = (sh_max - sh_min) + (lo_max - lo_min) + max(pass_width, offset) + 10.0

    # One station per ~half tool width so the polyline follows the curve smoothly.
    n = max(4, int(math.ceil((lo_max - lo_min) / max(pass_width * 0.5, 1.0))))
    inset = min(offset, (lo_max - lo_min) / 2.0)
    pts: list[list[float]] = []
    for k in range(n + 1):
        L = (lo_min + inset) + (lo_max - lo_min - 2 * inset) * (k / n)
        # anchor a point on the axis at long-coord L, then the perpendicular cut-line
        ax = axis_x * L + px * 0.0
        ay = axis_y * L + py * 0.0
        p0 = (ax + px * (sh_min - big), ay + py * (sh_min - big))
        p1 = (ax + px * (sh_max + big), ay + py * (sh_max + big))
        seg = _clip_line_segment_to_polygon(LineString([p0, p1]), poly)
        if not seg or len(seg) < 2:
            continue
        mid = ((seg[0][0] + seg[-1][0]) / 2.0, (seg[0][1] + seg[-1][1]) / 2.0)
        if not pts or math.hypot(mid[0] - pts[-1][0], mid[1] - pts[-1][1]) > 1e-6:
            pts.append([mid[0], mid[1]])
    return pts


def _best_curved_chord(poly: Any, axis_x: float, axis_y: float, pass_width: float, offset: float) -> tuple[list[list[float]], str] | None:
    """Choose a single straight stroke for a curved frame band.

    A curved frame band often has a horizontal bounding box, so the minimum-rectangle axis
    returns a rectangle-style horizontal pass. For robot sanding, a diagonal chord can cover
    that band more naturally. We test several safe chords and prefer a diagonal when it is
    long enough and stays fully inside the computed frame polygon.
    """
    from shapely.geometry import LineString
    import math

    minx, miny, maxx, maxy = [float(v) for v in poly.bounds]
    cx, cy = float(poly.centroid.x), float(poly.centroid.y)
    span = max(maxx - minx, maxy - miny, pass_width, offset, 1.0) * 3.0

    axis_angle = math.atan2(axis_y, axis_x)
    candidate_angles: list[float] = []
    for a in (axis_angle, 0.0, math.pi / 2.0):
        candidate_angles.append(a)
    for deg in (18, 25, 35, 45, 55, 65, 72, 108, 115, 125, 135, 145, 155, 162):
        candidate_angles.append(math.radians(deg))
    for delta in (-math.pi / 4.0, -math.pi / 6.0, math.pi / 6.0, math.pi / 4.0):
        candidate_angles.append(axis_angle + delta)

    # Try both centroid chords and bbox-corner diagonals. The corner diagonals help curved
    # side bands where the useful p1->p2 stroke should lean from the curve toward the
    # straight frame boundary instead of passing through the exact centroid horizontally.
    raw_lines = []
    seen_angles: set[int] = set()
    for angle in candidate_angles:
        ux = math.cos(angle)
        uy = math.sin(angle)
        key = int(round((angle % math.pi) * 1000))
        if key in seen_angles:
            continue
        seen_angles.add(key)
        raw_lines.append((angle, LineString([(cx - ux * span, cy - uy * span), (cx + ux * span, cy + uy * span)])))

    for a, b in (((minx, miny), (maxx, maxy)), ((minx, maxy), (maxx, miny))):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if math.hypot(dx, dy) > 1e-6:
            raw_lines.append((math.atan2(dy, dx), LineString([a, b])))

    candidates: list[tuple[float, float, list[list[float]], str]] = []
    for angle, line in raw_lines:
        span_line = _longest_line_part(line.intersection(poly))
        if span_line is None or span_line.length <= max(pass_width * 0.45, 1.0):
            continue
        pts = [[float(x), float(y)] for x, y in span_line.coords]
        pts = _trim_segment(pts, offset)
        if not pts or len(pts) < 2 or math.dist(pts[0], pts[-1]) <= 1e-6:
            continue
        if not _polyline_stays_in_polygon(pts, poly):
            continue
        length = math.dist(pts[0], pts[-1])
        normalized = abs(math.degrees(angle) % 180.0)
        diagonal = min(abs(normalized - 0.0), abs(normalized - 90.0), abs(normalized - 180.0))
        is_diagonal = diagonal >= 5.0
        candidates.append((length, diagonal, pts, "curved_diagonal_chord" if is_diagonal else "curved_chord_centerline"))

    if not candidates:
        return None

    max_len = max(c[0] for c in candidates)
    diagonal_candidates = [c for c in candidates if c[1] >= 5.0 and c[0] >= max(pass_width * 0.8, max_len * 0.25)]
    if diagonal_candidates:
        # Prefer the longest safe diagonal. This avoids rectangle-looking passes on curved
        # bands while still rejecting tiny diagonal stubs.
        best = max(diagonal_candidates, key=lambda c: (c[0], c[1]))
    else:
        best = max(candidates, key=lambda c: c[0])
    return best[2], best[3]


def _oriented_toolpath(poly: Any, pass_width: float, offset: float, step: float, half_pass: float, force_band: bool = False) -> tuple[list[list[float]], str, str]:
    from shapely.geometry import LineString
    import math

    # A reach-clipped diagonal band near the X crossing can be a bowtie / self-touching
    # ring; normalise to a simple polygon so the projection and clipping below are stable.
    poly = _largest_polygon(poly)
    if poly is None or poly.is_empty or poly.geom_type != "Polygon":
        return [], "X", "curved_band"

    ux, uy = _major_axis(poly)
    vx, vy = -uy, ux
    # Measure the band from the CONVEX HULL so a bent/self-touching exterior can't corrupt
    # the long/short projection used for band detection.
    hull = poly.convex_hull
    hull_coords = list(hull.exterior.coords[:-1]) if hull.geom_type == "Polygon" else list(poly.exterior.coords[:-1])
    coords = [(float(x), float(y)) for x, y in hull_coords]
    long_vals = [x * ux + y * uy for x, y in coords]
    short_vals = [x * vx + y * vy for x, y in coords]
    lo_min, lo_max = min(long_vals), max(long_vals)
    sh_min, sh_max = min(short_vals), max(short_vals)
    long_len = lo_max - lo_min
    short_len = sh_max - sh_min

    direction = "X" if abs(ux) >= abs(uy) else "Y"
    angle = abs(math.degrees(math.atan2(uy, ux))) % 180.0
    is_diagonal = 8.0 < angle < 82.0 or 98.0 < angle < 172.0

    # A frame BAND — any elongated strip — does not need a serpentine fill: the robot
    # moves linearly p1→p2, so ONE straight pass down the band's centerline is the
    # intended toolpath. Detecting a band by long:short RATIO alone fails for the pieces
    # near the X crossing, where the reach-clip leaves BENT / self-intersecting fragments
    # whose bounding box looks blocky (ratio ~1.3-1.5) even though the strip is thin.
    #
    # The reliable band signal is EFFECTIVE THICKNESS = area / long_length: a thin strip
    # (even bent) has a small thickness, a wide section has thickness ≈ its width. The
    # physical test is simply "does ONE tool pass cover the strip across its thickness?":
    #   • thickness ≤ ~1 tool width  → a single centerline pass covers it (thin frame
    #     band / diagonal arm) — one straight p1→p2 stroke.
    #   • thickness >  ~1 tool width → the section is wide and needs a serpentine FILL
    #     (the big edge rectangles) — fall through to the stepped multi-pass path.
    # We deliberately do NOT use the long:short ratio here: a long-but-wide section
    # (e.g. 400×200) must still be filled, not collapsed to one pass.
    try:
        _rect = poly.minimum_rotated_rectangle
        _rc = list(_rect.exterior.coords)
        _edges = [math.hypot(_rc[i + 1][0] - _rc[i][0], _rc[i + 1][1] - _rc[i][1]) for i in range(4)]
        _rect_long = max(_edges)
    except Exception:
        _rect_long = long_len
    eff_thickness = float(poly.area) / max(_rect_long, 1e-6)
    exterior_pts = [[float(x), float(y)] for x, y in poly.exterior.coords[:-1]]
    is_curve_poly = _ring_has_curve(exterior_pts) or _ring_has_non_axis_edge(exterior_pts)
    is_band = force_band or eff_thickness <= pass_width * 1.1
    if is_band:
        # Centerline direction = the min-rotated-rectangle's LONG edge (the band's true
        # axis, robust for diagonal arms). Cast a line along that axis through the band's
        # centroid, extend it well past both ends, then INTERSECT it with the polygon so
        # the pass is guaranteed to stay INSIDE the frame section and can NEVER cross into
        # a pocket. This is the "xy → xy inside the computed section" rule.
        strategy = "band_centerline" if not is_diagonal else "diagonal_centerline"
        try:
            rect = poly.minimum_rotated_rectangle
            rc = list(rect.exterior.coords)
            # longest edge of the rotated rect gives the axis direction
            best = None
            for i in range(4):
                ex, ey = rc[i + 1][0] - rc[i][0], rc[i + 1][1] - rc[i][1]
                elen = math.hypot(ex, ey)
                if best is None or elen > best[0]:
                    best = (elen, ex / max(elen, 1e-9), ey / max(elen, 1e-9))
            axis_x, axis_y = best[1], best[2]
        except Exception:
            axis_x, axis_y = ux, uy

        if is_curve_poly:
            # A genuinely CURVED band must be covered by a stroke that FOLLOWS the arc,
            # running down the middle of the band — not by a straight chord across it
            # (a chord leaves the band's ends unsanded and is what made curved previews
            # look rectangular). Sample cross-section centres along the band's axis and
            # connect them; simplify so the result keeps only the vertices the curve
            # needs. Fall back to the straight-chord search only if that fails.
            # A SHORT section must not be trimmed out of existence. Trimming `offset` off
            # both ends of an 80 mm band leaves nothing, so the section silently produced no
            # pass at all (the uncovered ribs between pockets). Scale the trim down so a
            # short band still keeps a usable stroke — the tool is wide and covers the part.
            eff_offset = offset
            if long_len > 1e-6:
                eff_offset = min(offset, max(long_len * 0.2, 0.0))

            if _ring_has_curve(exterior_pts):
                stations = _curved_band_centerline(poly, axis_x, axis_y, long_len, pass_width, eff_offset)
                if stations and len(stations) >= 2:
                    # Smooth first: raw cross-section midpoints wobble beside pocket corners
                    # and that wobble was showing up as a jagged pass. Then simplify with a
                    # tolerance large enough to drop residual noise but small enough to keep
                    # the real arc, so the result is a few long straight moves along the curve.
                    stations = _smooth_polyline(stations, window=7)
                    simplified = _simplify_polyline(stations, tol=max(pass_width * 0.06, 6.0))
                    simplified = _trim_polyline(simplified, eff_offset)
                    if len(simplified) >= 2 and math.dist(simplified[0], simplified[-1]) > 1e-6:
                        if _polyline_stays_in_polygon(simplified, poly):
                            return simplified, direction, "curved_band_centerline"
            curved = _best_curved_chord(poly, axis_x, axis_y, pass_width, eff_offset)
            if curved is not None:
                return curved[0], direction, curved[1]
            return [], direction, "curved_chord_centerline"

        # Non-curved thin/diagonal bands keep the original centerline rule. Explicit
        # structural centerline sections can be shorter than 2 * offset, so scale their
        # trim down instead of deleting the path.
        centerline_offset = offset
        if force_band and long_len > 1e-6:
            centerline_offset = min(offset, max(long_len * 0.2, 0.0))
        cx, cy = poly.centroid.x, poly.centroid.y
        reach = long_len + short_len + max(pass_width, centerline_offset) + 10.0
        line = LineString([
            (cx - axis_x * reach, cy - axis_y * reach),
            (cx + axis_x * reach, cy + axis_y * reach),
        ])
        span = _longest_line_part(line.intersection(poly))
        if span is not None and span.length > pass_width * 0.35:
            pts = [[float(x), float(y)] for x, y in span.coords]
            pts = _trim_segment(pts, centerline_offset)
            if pts and len(pts) >= 2 and math.dist(pts[0], pts[-1]) > 1e-6 and _polyline_stays_in_polygon(pts, poly):
                return pts, direction, strategy

        # Build the centerline by sampling cross-section CENTRES at stations along the
        # band's axis (a polyline that FOLLOWS the band, straight or curved). Then:
        #   • if the stations are collinear → it's a straight band → collapse to 2 points.
        #   • if they bend → it's a CURVED band → keep the polyline (linear point-to-point
        #     coverage of the curve; the wide tool need not be perfectly centered).
        stations = _curved_band_centerline(poly, axis_x, axis_y, long_len, pass_width, centerline_offset)
        if stations and len(stations) >= 2:
            simplified = _simplify_polyline(stations, tol=pass_width * 0.15)
            # Polyline-aware trim keeps the curve's intermediate vertices.
            simplified = _trim_polyline(simplified, centerline_offset)
            if (
                len(simplified) >= 2
                and math.dist(simplified[0], simplified[-1]) > 1e-6
                and _polyline_stays_in_polygon(simplified, poly)
            ):
                if len(simplified) == 2:
                    return simplified, direction, strategy
                return simplified, direction, "curved_band_centerline"

        # Fallback: straight centerline clipped to the section.
        cx, cy = poly.centroid.x, poly.centroid.y
        reach = long_len + short_len + max(pass_width, offset) + 10.0
        line = LineString([
            (cx - axis_x * reach, cy - axis_y * reach),
            (cx + axis_x * reach, cy + axis_y * reach),
        ])
        span = _longest_line_part(line.intersection(poly))
        if span is not None and span.length > pass_width * 0.5:
            pts = [[float(x), float(y)] for x, y in span.coords]
            pts = _trim_segment(pts, offset)
            if pts and len(pts) >= 2 and math.dist(pts[0], pts[-1]) > 1e-6 and _polyline_stays_in_polygon(pts, poly):
                return pts, direction, strategy
        # If the clip produced nothing usable, fall through to the stepped path below.

    centers = _pass_centers(sh_min, sh_max, pass_width, step, half_pass)
    long_inset = min(offset, max(long_len / 2.0, 0.0))
    pa0 = lo_min + long_inset
    pa1 = lo_max - long_inset
    if pa1 - pa0 <= 1e-6:
        return [], direction, "offset_too_short"

    pad = max(pass_width, offset, 1.0)
    points: list[list[float]] = []
    toggle = False
    for center in centers:
        a = (ux * (pa0 - pad) + vx * center, uy * (pa0 - pad) + vy * center)
        b = (ux * (pa1 + pad) + vx * center, uy * (pa1 + pad) + vy * center)
        clipped = _clip_line_segment_to_polygon(LineString([a, b]), poly)
        if not clipped:
            continue
        # If the requested offset cannot fit, skip this pass instead of violating it.
        clipped = _trim_segment(clipped, offset)
        if not clipped or not _polyline_stays_in_polygon(clipped, poly):
            continue
        if toggle:
            clipped = list(reversed(clipped))
        points.extend(clipped)
        toggle = not toggle

    strategy = "diagonal_zigzag" if is_diagonal else "oriented_zigzag"
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
        return None
    ux, uy = dx / length, dy / length
    return [[x0 + ux * trim, y0 + uy * trim], [x1 - ux * trim, y1 - uy * trim]]


def _trim_polyline(points: list[list[float]], trim: float) -> list[list[float]]:
    """Trim `trim` distance off BOTH ends of a polyline, measured ALONG the polyline, so
    a curved centerline keeps its intermediate curve vertices (unlike _trim_segment which
    collapses to the two endpoints)."""
    import math

    if trim <= 1e-9 or len(points) < 2:
        return [list(p) for p in points]
    seglens = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    total = sum(seglens)
    if total <= trim * 2.2:
        return []

    def _point_at(dist_from_start: float) -> list[float]:
        d = dist_from_start
        for i, L in enumerate(seglens):
            if d <= L or i == len(seglens) - 1:
                t = d / L if L > 1e-9 else 0.0
                ax, ay = points[i]
                bx, by = points[i + 1]
                return [ax + (bx - ax) * t, ay + (by - ay) * t]
            d -= L
        return list(points[-1])

    start_d, end_d = trim, total - trim
    out: list[list[float]] = [_point_at(start_d)]
    acc = 0.0
    for i in range(len(points) - 1):
        acc_next = acc + seglens[i]
        # keep interior vertices strictly between the two trim points
        if start_d < acc_next < end_d and acc_next - acc > 1e-9:
            v = points[i + 1]
            if math.dist(v, out[-1]) > 1e-6:
                out.append(list(v))
        acc = acc_next
    end_pt = _point_at(end_d)
    if math.dist(end_pt, out[-1]) > 1e-6:
        out.append(end_pt)
    return out


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
