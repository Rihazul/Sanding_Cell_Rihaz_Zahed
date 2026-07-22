from __future__ import annotations

from typing import Any

from .frame_geometry import _largest_polygon


def _corner_vertices(pts: list[list[float]], angle_deg: float = 18.0) -> list[list[float]]:
    """Return only the SHARP corners of a ring — vertices where the boundary turns by more
    than `angle_deg`. Smooth arc-flattened edges (many tiny turns) contribute nothing, so
    a curved pocket edge won't spawn stacked grid lines; rectangular/triangular corners
    (~90°/60° turns) are kept."""
    import math

    ring = [[float(x), float(y)] for x, y in pts]
    if len(ring) >= 2 and math.hypot(ring[0][0] - ring[-1][0], ring[0][1] - ring[-1][1]) < 1e-6:
        ring = ring[:-1]
    n = len(ring)
    if n < 3:
        return ring
    thresh = math.radians(angle_deg)
    corners: list[list[float]] = []
    for i in range(n):
        a = ring[(i - 1) % n]
        b = ring[i]
        c = ring[(i + 1) % n]
        v1x, v1y = b[0] - a[0], b[1] - a[1]
        v2x, v2y = c[0] - b[0], c[1] - b[1]
        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        dot = (v1x * v2x + v1y * v2y) / (l1 * l2)
        dot = max(-1.0, min(1.0, dot))
        if math.acos(dot) > thresh:  # turn angle at this vertex
            corners.append(b)
    return corners if corners else ring


def _coarsen_sorted(values: list[float], merge: float) -> list[float]:
    """Sorted unique values with any run closer than `merge` collapsed to a single
    representative. Stops arc-flattening vertices from creating micro grid lines while
    keeping real corners (which are far apart). The domain endpoints (first/last) are
    always kept."""
    vals = sorted(set(round(float(v), 4) for v in values))
    if len(vals) <= 2:
        return vals
    lo, hi = vals[0], vals[-1]
    out = [lo]
    for v in vals[1:-1]:
        if v - out[-1] >= merge:
            out.append(v)
    if hi - out[-1] >= merge or len(out) == 1:
        out.append(hi)
    else:
        out[-1] = hi  # keep the true upper endpoint
    return out


def _dedupe_consecutive(points: list[list[float]], tol: float = 0.5) -> list[list[float]]:
    import math

    cleaned: list[list[float]] = []
    for p in points:
        if not cleaned or math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > tol:
            cleaned.append([float(p[0]), float(p[1])])
    if len(cleaned) >= 2 and math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) <= tol:
        cleaned.pop()
    return cleaned


def _ring_has_curve(points: list[list[float]], angle_deg: float = 8.0) -> bool:
    import math

    if len(points) < 8:
        return False
    small_turns = 0
    thresh = math.radians(angle_deg)
    for i in range(len(points)):
        a = points[(i - 1) % len(points)]
        b = points[i]
        c = points[(i + 1) % len(points)]
        v1x, v1y = b[0] - a[0], b[1] - a[1]
        v2x, v2y = c[0] - b[0], c[1] - b[1]
        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        dot = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
        turn = math.acos(dot)
        if 0.5 * math.pi / 180.0 < turn < thresh:
            small_turns += 1
    return small_turns >= 3


def _ring_has_non_axis_edge(points: list[list[float]], min_len: float = 5.0, angle_tol_deg: float = 6.0) -> bool:
    """True when a ring contains a meaningful edge that is not horizontal/vertical.

    Some DXF arcs or sloped frame boundaries arrive as only a few vertices, so they are
    not detected by _ring_has_curve. They still should not be handled as rectangular
    fill sections; a diagonal p1->p2 stroke is the correct preview for those bands.
    """
    import math

    if len(points) < 3:
        return False
    tol = float(angle_tol_deg)
    for a, b in zip(points, points[1:] + points[:1]):
        dx = float(b[0]) - float(a[0])
        dy = float(b[1]) - float(a[1])
        length = math.hypot(dx, dy)
        if length < min_len:
            continue
        angle = abs(math.degrees(math.atan2(dy, dx))) % 180.0
        nearest_axis = min(abs(angle - 0.0), abs(angle - 90.0), abs(angle - 180.0))
        if nearest_axis > tol:
            return True
    return False


def _dedupe_ring(points: list[list[float]], tol: float = 0.5) -> list[list[float]]:
    """Drop consecutive near-duplicate and collinear vertices from a ring, leaving only
    the real corners. Keeps any genuine bend (angle change > ~1°)."""
    import math

    if len(points) < 3:
        return points
    # First remove consecutive duplicates.
    cleaned: list[list[float]] = []
    for p in points:
        if not cleaned or math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > tol:
            cleaned.append([float(p[0]), float(p[1])])
    if len(cleaned) >= 2 and math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) <= tol:
        cleaned.pop()
    if len(cleaned) < 3:
        return cleaned
    # Then drop collinear middle points (the previous→current→next turn is ~straight).
    out: list[list[float]] = []
    n = len(cleaned)
    for i in range(n):
        a = cleaned[(i - 1) % n]
        b = cleaned[i]
        c = cleaned[(i + 1) % n]
        v1x, v1y = b[0] - a[0], b[1] - a[1]
        v2x, v2y = c[0] - b[0], c[1] - b[1]
        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        cross = (v1x * v2y - v1y * v2x) / (l1 * l2)  # sin(turn angle)
        if abs(cross) > 0.017:  # ~1° — keep real corners, drop collinear
            out.append(b)
    return out if len(out) >= 3 else cleaned


def _section_effective_thickness(poly: Any) -> float:
    """area / longest-rotated-rect-edge — the strip's mean thickness (bend-proof)."""
    import math
    p = _largest_polygon(poly)
    if p is None or p.is_empty or p.geom_type != "Polygon":
        return 1e9
    try:
        rc = list(p.minimum_rotated_rectangle.exterior.coords)
        edges = [math.hypot(rc[i + 1][0] - rc[i][0], rc[i + 1][1] - rc[i][1]) for i in range(4)]
        rect_long = max(edges)
    except Exception:
        minx, miny, maxx, maxy = p.bounds
        rect_long = max(maxx - minx, maxy - miny)
    return float(p.area) / max(rect_long, 1e-6)


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
