from __future__ import annotations

from typing import Any


def _frame_zigzag_stations(minx: float, maxx: float, step: float, pass_width: float) -> list[float]:
    """Return X stations for frame zigzag, including both edges without tiny tail passes."""
    if maxx < minx:
        minx, maxx = maxx, minx

    stations: list[float] = []
    x = float(minx)
    while x <= maxx + 1e-6:
        stations.append(x)
        x += step

    if not stations:
        return [float(minx)]

    if abs(stations[0] - minx) > 1e-6:
        stations.insert(0, float(minx))

    tail = float(maxx) - stations[-1]
    if tail > 1e-6:
        min_tail_spacing = max(10.0, min(float(step) * 0.35, float(pass_width) * 0.5))
        if tail < min_tail_spacing and len(stations) > 1:
            stations[-1] = float(maxx)
        else:
            stations.append(float(maxx))

    deduped: list[float] = []
    for value in stations:
        if not deduped or abs(deduped[-1] - value) > 1e-6:
            deduped.append(float(value))
    return deduped


def _same_xy(a: list[float], b: list[float], tol: float = 1e-6) -> bool:
    return abs(float(a[0]) - float(b[0])) <= tol and abs(float(a[1]) - float(b[1])) <= tol


def _dedupe_polyline_points(points: list[list[float]]) -> list[list[float]]:
    out: list[list[float]] = []
    for p in points:
        pt = [float(p[0]), float(p[1])]
        if not out or not _same_xy(out[-1], pt):
            out.append(pt)
    return out


def _remove_collinear_waypoints(points: list[list[float]], tol: float = 1e-6) -> list[list[float]]:
    """Remove intermediate waypoints that do not change the MoveL direction.

    Chaining can create A -> B -> C where B sits on the same straight line. B is useful
    during planning, but unnecessary for robot execution and operator review. L-corners are
    preserved because their cross product is non-zero.
    """
    pts = _dedupe_polyline_points(points)
    if len(pts) <= 2:
        return pts

    simplified: list[list[float]] = [pts[0]]
    for index in range(1, len(pts) - 1):
        point = pts[index]
        prev = simplified[-1]
        nxt = pts[index + 1]
        abx = float(point[0]) - float(prev[0])
        aby = float(point[1]) - float(prev[1])
        bcx = float(nxt[0]) - float(point[0])
        bcy = float(nxt[1]) - float(point[1])
        acx = float(nxt[0]) - float(prev[0])
        acy = float(nxt[1]) - float(prev[1])
        scale = max((acx * acx + acy * acy) ** 0.5, 1.0)
        cross = abs(abx * bcy - aby * bcx)
        dot = abx * bcx + aby * bcy

        # Same line and same travel direction: B does not change the robot's path.
        if cross <= tol * scale and dot >= -tol:
            continue
        simplified.append(point)
    simplified.append(pts[-1])
    return simplified


def _remove_short_backtrack_jogs(points: list[list[float]], jog_tol: float = 8.0) -> list[list[float]]:
    """Remove tiny connector reversals introduced by trimming/chaining.

    Example: A -> B -> C where B->C is a 2 mm reverse move on the same line.
    The useful robot path is A -> C; B only makes the TCP touch the wrong edge.
    """
    import math

    pts = _dedupe_polyline_points(points)
    if len(pts) < 3:
        return pts

    def axis(a: list[float], b: list[float]) -> str | None:
        dx = abs(float(b[0]) - float(a[0]))
        dy = abs(float(b[1]) - float(a[1]))
        if dx <= 1e-6 and dy <= 1e-6:
            return None
        if dx <= 1e-6:
            return "y"
        if dy <= 1e-6:
            return "x"
        return None

    changed = True
    while changed and len(pts) >= 3:
        changed = False
        cleaned: list[list[float]] = [pts[0]]
        i = 1
        while i < len(pts) - 1:
            a = cleaned[-1]
            b = pts[i]
            c = pts[i + 1]
            ab_axis = axis(a, b)
            bc_axis = axis(b, c)
            if ab_axis is not None and ab_axis == bc_axis:
                ab = math.dist(a, b)
                bc = math.dist(b, c)
                if bc <= jog_tol:
                    # B is a tiny overshoot, C is back on the useful line.
                    changed = True
                    i += 1
                    continue
                if ab <= jog_tol:
                    # A->B is the tiny jog before the real segment; drop B.
                    changed = True
                    i += 1
                    continue
            cleaned.append(b)
            i += 1
        cleaned.append(pts[-1])
        pts = _dedupe_polyline_points(cleaned)
    return pts


def _last_segment_axis(points: list[list[float]]) -> str | None:
    if len(points) < 2:
        return None
    a = points[-2]
    b = points[-1]
    dx = abs(float(b[0]) - float(a[0]))
    dy = abs(float(b[1]) - float(a[1]))
    if dx <= 1e-6 and dy <= 1e-6:
        return None
    return "x" if dx >= dy else "y"


def _first_segment_axis(points: list[list[float]]) -> str | None:
    if len(points) < 2:
        return None
    a = points[0]
    b = points[1]
    dx = abs(float(b[0]) - float(a[0]))
    dy = abs(float(b[1]) - float(a[1]))
    if dx <= 1e-6 and dy <= 1e-6:
        return None
    return "x" if dx >= dy else "y"


def _path_source_ids(path: dict[str, Any]) -> list[Any]:
    ids = path.get("source_section_ids")
    if isinstance(ids, list):
        return [sid for sid in ids if sid is not None]
    sid = path.get("source_section_id")
    return [sid] if sid is not None else []


def _polyline_self_overlaps(pts: list[list[float]], tol: float = 2.0) -> bool:
    """True when the run re-covers its own geometry (re-sanding).

    Checks EVERY pair of segments, including adjacent ones, for a shared length of travel.
    Adjacent pairs matter: a connector that runs back along the stroke it just came from
    (e.g. ...->[35,y] then [35,y]->[535,y] after arriving from [465,y]) re-sands [35..465]
    but is two adjacent segments, so a non-adjacent-only test would miss it. Measuring actual
    overlap length means legitimate offset serpentine rungs (parallel but not collinear) do
    not trip this — only genuine re-coverage does.
    """
    try:
        import math
        from shapely.geometry import LineString

        closed = len(pts) >= 2 and math.dist(pts[0], pts[-1]) <= tol
        segs = [LineString([pts[i], pts[i + 1]]) for i in range(len(pts) - 1)]
        # A thin buffer makes the overlap test robust to floating-point noise: two rails at
        # y=315.9125 vs y=315.9125000000001 are NOT collinear to shapely, so a plain
        # intersection returns empty and a real re-sand slips through. Measuring how much of
        # one segment lies inside the other's thin buffer catches it regardless of that noise.
        eps = 0.5
        buffers = [seg.buffer(eps, cap_style=2) for seg in segs]
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                if closed and i == 0 and j == len(segs) - 1:
                    continue  # a genuine closed loop: first and last segments meet, that's fine
                covered = segs[j].intersection(buffers[i])
                overlap = float(getattr(covered, "length", 0.0))
                if overlap > tol + 2.0 * eps:
                    return True
    except Exception:
        return True
    return False
