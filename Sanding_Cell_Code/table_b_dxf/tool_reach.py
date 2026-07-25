from __future__ import annotations

"""Tool reach envelopes on UCS Table B, and 7th-axis planning.

Geometry model
--------------
Each tool's reachable area is expressed in UCS XY, measured from the UCS origin with the
tool's TCP calibrated to (0, 0) at 7th-axis x = 0. The envelope is piecewise:

  * below a y threshold  -> a plain rectangle  x in [x_lo, x_hi]
  * at/above the threshold -> sampled rows  y -> [x_min, x_max] that narrow as y rises
    (the arm folds in as it reaches further out), linearly interpolated between samples.

The 7th axis translates the robot BASE along X only. Its position resets the base to 0,
so in UCS terms the whole envelope simply SLIDES along X by the 7th-axis position `s`:

    UCS point (x, y) is reachable at station s  <=>  (x - s, y) lies in the envelope.

Planning goal
-------------
Operations run tool by tool in priority order. For each tool we want to cover all of its
toolpaths with as FEW 7th-axis moves as possible - i.e. park the base, sweep as much as the
arm can reach, and only then translate. `plan_stations` does that with a greedy interval
sweep, which is optimal for this shape of problem (see note on the algorithm there).
"""

import logging
import math
from typing import Any, Iterable, Sequence

logger = logging.getLogger(__name__)


# Recorded arm-reach data. y thresholds and rows are in mm, x ranges in mm from UCS origin.
TOOL_REACH: dict[str, dict[str, Any]] = {
    "tool_3": {
        "rect_below_y": 336.0,
        "rect_x": (0.0, 695.0),
        "rows": [(336.0, -1100.0, 695.0), (478.0, -1045.0, 602.0),
                 (801.0, -835.0, 375.0), (915.0, -709.0, 222.0)],
    },
    "tool_1": {
        "rect_below_y": 300.0,
        "rect_x": (0.0, 660.0),
        "rows": [(300.0, -1110.0, 660.0), (530.0, -1035.0, 560.0),
                 (710.0, -960.0, 430.0), (915.0, -720.0, 180.0)],
    },
    "tool_4": {
        "rect_below_y": 305.0,
        "rect_x": (0.0, 630.0),
        "rows": [(305.0, -1100.0, 630.0), (515.0, -1050.0, 585.0),
                 (740.0, -940.0, 420.0), (916.0, -690.0, 205.0)],
    },
}

# Base of robot origin is offset from the UCS origin along X. Kept here so callers that
# need robot-base coordinates can convert; the reach maths itself is all in UCS.
ROBOT_BASE_X_OFFSET_MM = -257.4

# Per-tool sanding offset (mm) applied when the pass geometry is generated. Tools 1 and 3
# run the SAME pass logic - a CCW loop broken only where reach requires it - so the offset
# is the only thing that distinguishes their paths. Mirrors the frontend constants.
#
# Note tool 3's offset is ANISOTROPIC: its TCP sits 38.1 mm in from the pocket edge in X
# but 50.8 mm in Y (each plus a 2.5 mm margin), so it is stored as an (x, y) pair rather
# than a single inset.
TOOL_OFFSET_MM: dict[str, Any] = {
    "tool_3": (38.1 + 2.5, 50.8 + 2.5),  # pocket edge, (x, y)
    "tool_1": 27.0,                      # 3D contour ring
    "tool_4": 72.0,                      # pocket zigzag (frame pass uses 50.0)
}
TOOL4_FRAME_OFFSET_MM = 50.0

# Table B physical limits.
AXIS7_MIN_MM = 0.0        # 7th axis home
AXIS7_MAX_MM = 2310.0     # 7th axis furthest travel
DOOR_MAX_X_MM = 2445.0    # a larger part violates the Table B size constraint
DOOR_MAX_Y_MM = 920.0

# The recorded envelope rows stop at y ~915/916, but a part may legitimately reach
# DOOR_MAX_Y_MM. Rather than declare that sliver unreachable (which would silently drop
# toolpaths along the door's top edge), hold the topmost recorded row for this small
# extrapolation. Anything beyond it is still refused.
TOP_ROW_EXTRAPOLATION_MM = 6.0


def check_part_size(width_x_mm: float, height_y_mm: float) -> list[str]:
    """Return Table B size-constraint violations for a part, empty when it fits."""
    problems: list[str] = []
    if float(width_x_mm) > DOOR_MAX_X_MM + 1e-6:
        problems.append(
            f"part width {float(width_x_mm):.1f} mm exceeds Table B maximum {DOOR_MAX_X_MM:.0f} mm in X"
        )
    if float(height_y_mm) > DOOR_MAX_Y_MM + 1e-6:
        problems.append(
            f"part height {float(height_y_mm):.1f} mm exceeds Table B maximum {DOOR_MAX_Y_MM:.0f} mm in Y"
        )
    return problems


def reach_span_at_y(tool: str, y: float) -> tuple[float, float] | None:
    """Local x-range the tool can reach at height `y`, with the base at x = 0.

    Returns None when `y` is outside the recorded envelope entirely.
    """
    spec = TOOL_REACH.get(tool)
    if spec is None:
        return None
    y = float(y)
    if y < float(spec["rect_below_y"]):
        lo, hi = spec["rect_x"]
        return (float(lo), float(hi))

    rows: Sequence[tuple[float, float, float]] = spec["rows"]
    if y <= rows[0][0]:
        return (float(rows[0][1]), float(rows[0][2]))
    if y >= rows[-1][0]:
        # The recorded rows stop a few mm below the maximum part height. Hold the topmost
        # row across that sliver so a pass along the door's top edge is not declared
        # unreachable; refuse anything meaningfully beyond the recorded data.
        if y <= rows[-1][0] + TOP_ROW_EXTRAPOLATION_MM:
            return (float(rows[-1][1]), float(rows[-1][2]))
        return None

    for (y0, a0, b0), (y1, a1, b1) in zip(rows, rows[1:]):
        if y0 <= y <= y1:
            span = y1 - y0
            t = 0.0 if span <= 1e-9 else (y - y0) / span
            return (a0 + (a1 - a0) * t, b0 + (b1 - b0) * t)
    return None


def station_window_for_point(tool: str, x: float, y: float) -> tuple[float, float] | None:
    """Range of 7th-axis positions from which the tool can reach UCS point (x, y).

    Reachable when (x - s) is inside the local span [lo, hi], i.e. s in [x - hi, x - lo].
    """
    span = reach_span_at_y(tool, y)
    if span is None:
        return None
    lo, hi = span
    return (float(x) - hi, float(x) - lo)


def station_window_for_path(tool: str, points: Iterable[Sequence[float]]) -> tuple[float, float] | None:
    """Station range that reaches EVERY point of one toolpath.

    A single pass must be executed without repositioning, so the path's window is the
    intersection of its points' windows. Returns None when no single station covers the
    whole path (the caller must then split the path).
    """
    lo = None
    hi = None
    for p in points:
        w = station_window_for_point(tool, float(p[0]), float(p[1]))
        if w is None:
            return None
        lo = w[0] if lo is None else max(lo, w[0])
        hi = w[1] if hi is None else min(hi, w[1])
        if lo > hi:
            return None
    if lo is None or hi is None:
        return None
    return (lo, hi)


def simplify_collinear_points(points: Sequence[Sequence[float]], tol: float = 1e-6) -> list[list[float]]:
    """Remove planner-only points that lie on the same straight MoveL segment.

    Reach planning may densify a line so it can find legal split positions. Those
    intermediate points are useful for planning, but they should not be exposed as
    extra robot waypoints when the motion is still one straight line.
    """
    pts = [[float(p[0]), float(p[1])] for p in points]
    if len(pts) <= 2:
        return pts

    simplified: list[list[float]] = [pts[0]]
    for index in range(1, len(pts) - 1):
        p = pts[index]
        a = simplified[-1]
        b = pts[index + 1]
        abx = b[0] - a[0]
        aby = b[1] - a[1]
        apx = p[0] - a[0]
        apy = p[1] - a[1]
        cross = abs(abx * apy - aby * apx)
        scale = max(math.hypot(abx, aby), 1.0)
        if cross > tol * scale:
            simplified.append(p)
            continue
        dot = apx * abx + apy * aby
        if dot < -tol or dot > (abx * abx + aby * aby) + tol:
            simplified.append(p)
    simplified.append(pts[-1])
    return simplified


def split_path_for_reach(tool: str, points: Sequence[Sequence[float]]) -> list[list[list[float]]]:
    """Cut one toolpath into the fewest pieces each reachable from a single station.

    A pass longer than the arm's span at its height (e.g. a 1500 mm rail where the envelope
    is 1470 mm wide) cannot be run in one move no matter where the base parks. Walk the
    path and start a new piece as soon as the running intersection of station windows would
    become empty - that yields the fewest cuts for a given point order. Pieces are joined
    end-to-end so the surface is still fully covered.
    """
    pts = [[float(p[0]), float(p[1])] for p in points]
    if len(pts) < 2:
        return [pts] if pts else []

    # Cuts can only happen AT a vertex, so a long straight pass (just two endpoints) has
    # nowhere to cut and would come back unreachable. Densify first: insert intermediate
    # points along each segment, spaced well inside the narrowest reach span, so the walk
    # below always has somewhere to break. This adds no length - the points are collinear.
    narrowest = None
    for p in pts:
        span = reach_span_at_y(tool, p[1])
        if span is not None:
            w = span[1] - span[0]
            narrowest = w if narrowest is None else min(narrowest, w)
    step = max((narrowest or 400.0) * 0.25, 25.0)
    dense: list[list[float]] = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        seg = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        n = int(seg // step)
        for k in range(1, n + 1):
            t = (k * step) / seg
            if t < 1.0:
                dense.append([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
        dense.append([float(b[0]), float(b[1])])
    pts = dense

    pieces: list[list[list[float]]] = []
    current: list[list[float]] = [pts[0]]
    lo, hi = None, None
    w0 = station_window_for_point(tool, pts[0][0], pts[0][1])
    if w0 is not None:
        lo, hi = w0

    for p in pts[1:]:
        w = station_window_for_point(tool, p[0], p[1])
        if w is None:
            # Point is outside the envelope at any station; keep it so coverage is honest,
            # the caller's unreachable report will flag it.
            current.append(p)
            continue
        nlo = w[0] if lo is None else max(lo, w[0])
        nhi = w[1] if hi is None else min(hi, w[1])
        if nlo > nhi:
            # Adding this point would make the piece unreachable - close it and restart
            # from the previous point so the pieces stay connected.
            pieces.append(simplify_collinear_points(current))
            carried = current[-1]
            current = [carried, p]
            wc = station_window_for_point(tool, carried[0], carried[1])
            if wc is None:
                lo, hi = w
            else:
                lo, hi = max(wc[0], w[0]), min(wc[1], w[1])
        else:
            current.append(p)
            lo, hi = nlo, nhi
    if len(current) >= 2:
        pieces.append(simplify_collinear_points(current))
    return pieces


def chain_touching_paths(toolpaths: list[dict[str, Any]], tool: str,
                         join_tol: float = 1e-6) -> list[dict[str, Any]]:
    """Join passes that meet end-to-start into longer runs the arm can do in one move.

    Frame SECTIONS are only a visual/scaffolding division - they exist so the toolpath
    algorithm can place the tool on each band's centreline. They are NOT execution
    boundaries. Two passes that touch should therefore be run as ONE continuous stroke
    whenever a single 7th-axis station can reach the combined run, which means fewer
    lifts, fewer repositions, and a path that reads as a loop rather than fragments.

    Joining never duplicates geometry: the shared endpoint is written once, so the merged
    run still covers each section exactly once.
    """
    remaining = [dict(tp) for tp in toolpaths if (tp.get("points") or [])]
    out: list[dict[str, Any]] = []

    while remaining:
        current = remaining.pop(0)
        changed = True
        while changed:
            changed = False
            for i, cand in enumerate(remaining):
                cpts = current["points"]
                pts = cand["points"]
                # Only chain compatible operations; mixing a fill into an edge run would
                # change what the pass means.
                if cand.get("operation_type") != current.get("operation_type"):
                    continue
                merged = None
                if math.dist(cpts[-1], pts[0]) <= join_tol:
                    merged = cpts + pts[1:]
                elif math.dist(cpts[-1], pts[-1]) <= join_tol:
                    merged = cpts + list(reversed(pts))[1:]
                elif math.dist(cpts[0], pts[-1]) <= join_tol:
                    merged = pts + cpts[1:]
                elif math.dist(cpts[0], pts[0]) <= join_tol:
                    merged = list(reversed(pts)) + cpts[1:]
                if merged is None:
                    continue
                # Only worth joining if the arm can still run the whole thing in one go.
                if station_window_for_path(tool, merged) is None:
                    continue
                current["points"] = merged
                current["start_point"] = merged[0]
                current["end_point"] = merged[-1]
                current.setdefault("chained_path_ids", [current.get("path_id")])
                current["chained_path_ids"].append(cand.get("path_id"))
                remaining.pop(i)
                changed = True
                break
        out.append(current)
    return out


def attach_station_plan(toolpaths: list[dict[str, Any]], tool: str = "tool_4",
                        chain: bool = True) -> dict[str, Any]:
    """Assign a 7th-axis station to every toolpath, splitting any that cannot be reached.

    Mutates each toolpath in place, adding:
      * ``station_index``    - which stop runs it (also drives the preview colour)
      * ``axis7_position_mm``- where the 7th axis must be parked for that stop

    Paths too long for the arm are split into contiguous pieces (no overlap, no gap, so no
    surface is sanded twice) and appended to `toolpaths`; the original is replaced by its
    first piece. Returns the plan summary for the UI/executor.
    """
    if not toolpaths:
        return {"tool": tool, "stations": [], "station_count": 0, "total_travel_mm": 0.0}

    # Sections are a visual division, not an execution boundary: first join passes that
    # meet end-to-start into the longest runs one station can reach, THEN split whatever
    # is still too long. Doing it in this order means a run is broken only by real reach
    # limits, never by where a section happened to end.
    source = chain_touching_paths(toolpaths, tool) if chain else list(toolpaths)

    expanded: list[dict[str, Any]] = []
    for tp in source:
        pts = tp.get("points") or []
        if len(pts) < 2:
            expanded.append(tp)
            continue
        if station_window_for_path(tool, pts) is not None:
            expanded.append(tp)
            continue
        pieces = split_path_for_reach(tool, pts)
        if len(pieces) <= 1:
            expanded.append(tp)
            continue
        for k, piece in enumerate(pieces):
            clone = dict(tp)
            clone["points"] = piece
            clone["start_point"] = piece[0]
            clone["end_point"] = piece[-1]
            if k:
                clone["path_id"] = f"{tp.get('path_id', 'path')}_r{k}"
                clone["split_from_path_id"] = tp.get("path_id")
            clone["reach_split_index"] = k
            clone["reach_split_count"] = len(pieces)
            expanded.append(clone)

    plan = plan_stations(tool, [tp.get("points") or [] for tp in expanded])
    for si, station in enumerate(plan["stations"]):
        for idx in station["path_indices"]:
            if 0 <= idx < len(expanded):
                expanded[idx]["station_index"] = si
                expanded[idx]["axis7_position_mm"] = station["position"]
    for idx in plan["unreachable_path_indices"]:
        if 0 <= idx < len(expanded):
            expanded[idx]["station_index"] = None
            expanded[idx]["reach_unreachable"] = True

    for tp in expanded:
        pts = tp.get("points") or []
        if len(pts) >= 2:
            simplified = simplify_collinear_points(pts)
            tp["points"] = simplified
            tp["start_point"] = simplified[0]
            tp["end_point"] = simplified[-1]

    toolpaths[:] = expanded
    return plan


def plan_closed_loop(tool: str, loop: Sequence[Sequence[float]],
                     start_index: int = 0) -> dict[str, Any]:
    """Plan a CCW closed loop (pocket edge / 3D ring) as arcs, each run from one station.

    Tool 1 (3D) and tool 3 (pocket edge) use IDENTICAL pass logic - both walk a CCW loop
    and break only where reach forces it. They differ only in their tool offset, which is
    applied when the loop geometry is generated upstream, and in their reach envelope,
    which is data in TOOL_REACH. So there is deliberately no per-tool branching here.

    Rules this enforces, from the Table B operating spec:

    * NO SECTION IS COVERED TWICE. Once the arm lifts or the 7th axis moves, geometry that
      was already sanded must not be revisited, so the arcs PARTITION the loop exactly -
      they meet end-to-start with no overlap.
    * Each arc must be reachable from a single 7th-axis station (the arm cannot reposition
      mid-stroke).
    * Prefer long arcs: the walk extends an arc for as long as the running intersection of
      station windows stays non-empty, so we get the fewest, longest strokes and therefore
      the fewest 7th-axis moves.

    `start_index` picks where the walk begins. The loop is a cycle, so the break points
    depend on where you start; `plan_closed_loop_best_start` tries every vertex and keeps
    whichever start yields the fewest arcs.
    """
    pts = [[float(p[0]), float(p[1])] for p in loop]
    if len(pts) >= 2 and abs(pts[0][0] - pts[-1][0]) < 1e-9 and abs(pts[0][1] - pts[-1][1]) < 1e-9:
        pts = pts[:-1]
    n = len(pts)
    if n < 2:
        return {"tool": tool, "arcs": [], "arc_count": 0, "unreachable": True}

    order = [pts[(start_index + k) % n] for k in range(n)]
    order.append(list(order[0]))  # close the cycle so the last edge is covered

    # Densify so a break can land anywhere along a long edge, not only at a vertex.
    narrowest = None
    for p in order:
        span = reach_span_at_y(tool, p[1])
        if span is not None:
            w = span[1] - span[0]
            narrowest = w if narrowest is None else min(narrowest, w)
    step = max((narrowest or 400.0) * 0.25, 25.0)
    dense: list[list[float]] = [order[0]]
    for a, b in zip(order, order[1:]):
        seg = math.dist(a, b)
        if seg > 1e-9:
            for k in range(1, int(seg // step) + 1):
                t = (k * step) / seg
                if t < 1.0:
                    dense.append([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
        dense.append([float(b[0]), float(b[1])])

    arcs: list[dict[str, Any]] = []
    current: list[list[float]] = [dense[0]]
    lo, hi = station_window_for_point(tool, dense[0][0], dense[0][1]) or (None, None)
    unreachable_pts = 0
    for p in dense[1:]:
        w = station_window_for_point(tool, p[0], p[1])
        if w is None:
            unreachable_pts += 1
            current.append(p)
            continue
        nlo = w[0] if lo is None else max(lo, w[0])
        nhi = w[1] if hi is None else min(hi, w[1])
        if lo is not None and nlo > nhi:
            arcs.append({"points": simplify_collinear_points(current), "station_window": (lo, hi)})
            # The new arc starts at the previous arc's END (no overlap, no gap), so its
            # window is the intersection of THAT carried-over point and the new one - not
            # the new point alone, which would leave a window the arc cannot actually honour.
            carried = current[-1]
            current = [carried, p]
            wc = station_window_for_point(tool, carried[0], carried[1])
            if wc is None:
                lo, hi = w
            else:
                lo, hi = max(wc[0], w[0]), min(wc[1], w[1])
        else:
            current.append(p)
            lo, hi = nlo, nhi
    if len(current) >= 2:
        arcs.append({"points": simplify_collinear_points(current), "station_window": (lo, hi) if lo is not None else None})

    return {
        "tool": tool,
        "arcs": arcs,
        "arc_count": len(arcs),
        "unreachable": unreachable_pts > 0,
        "closed_in_one_go": len(arcs) == 1,
    }


def plan_closed_loop_best_start(tool: str, loop: Sequence[Sequence[float]]) -> dict[str, Any]:
    """Try every start vertex and keep the plan with the fewest arcs.

    Where a cycle is broken changes how many arcs it needs, so scanning the starts finds
    the split that costs the fewest 7th-axis repositions.
    """
    pts = [[float(p[0]), float(p[1])] for p in loop]
    if len(pts) >= 2 and abs(pts[0][0] - pts[-1][0]) < 1e-9 and abs(pts[0][1] - pts[-1][1]) < 1e-9:
        pts = pts[:-1]
    best: dict[str, Any] | None = None
    for i in range(max(1, len(pts))):
        plan = plan_closed_loop(tool, pts, start_index=i)
        if best is None or (plan["arc_count"], plan["unreachable"]) < (best["arc_count"], best["unreachable"]):
            best = plan
            best["start_index"] = i
        if best["arc_count"] == 1 and not best["unreachable"]:
            break
    return best or {"tool": tool, "arcs": [], "arc_count": 0, "unreachable": True}


def plan_stations(tool: str, paths: Sequence[Sequence[Sequence[float]]]) -> dict[str, Any]:
    """Choose the FEWEST 7th-axis stations that cover every path for this tool.

    Each path yields a closed interval of feasible station positions. Covering all paths
    with the fewest points is the classic "stabbing intervals" problem: sort by interval
    end, place a station at the earliest end not yet covered, repeat. That greedy choice is
    provably minimal, which is exactly the "less 7th-axis movement, more arm coverage" goal.

    Paths that no single station can reach are reported separately rather than silently
    dropped - they need splitting before they can be run.
    """
    windows: list[tuple[float, float, int]] = []
    unreachable: list[int] = []
    for i, path in enumerate(paths):
        w = station_window_for_path(tool, path)
        if w is None:
            unreachable.append(i)
            continue
        # The 7th axis cannot travel outside its physical range, so clip each window to it.
        # A window that falls entirely outside means no legal station reaches that path —
        # report it rather than clamping to a position that would not actually reach.
        lo = max(w[0], AXIS7_MIN_MM)
        hi = min(w[1], AXIS7_MAX_MM)
        if lo > hi:
            unreachable.append(i)
            continue
        windows.append((lo, hi, i))

    stations: list[dict[str, Any]] = []
    for lo, hi, idx in sorted(windows, key=lambda w: w[1]):
        for st in stations:
            if lo <= st["position"] <= hi:
                st["path_indices"].append(idx)
                # Tighten the station's feasible range to the paths it now serves, so the
                # position can be nudged later without losing any of them.
                st["lo"] = max(st["lo"], lo)
                st["hi"] = min(st["hi"], hi)
                break
        else:
            stations.append({"position": hi, "lo": lo, "hi": hi, "path_indices": [idx]})

    stations.sort(key=lambda st: st["position"])

    # The count above is already minimal, but a station may sit anywhere inside its
    # feasible range - the greedy leaves it at the far end, which can park the base well
    # past the part. Pull each station to the point in its range closest to the previous
    # one: same number of stops, less 7th-axis travel.
    previous = 0.0
    for st in stations:
        st["position"] = min(max(previous, st["lo"]), st["hi"])
        previous = st["position"]
    stations.sort(key=lambda st: st["position"])
    total_travel = 0.0
    previous = 0.0
    for st in stations:
        total_travel += abs(st["position"] - previous)
        previous = st["position"]
    for st in stations:
        st.pop("lo", None)
        st.pop("hi", None)
    if unreachable:
        logger.warning(
            "Tool %s: %d toolpath(s) cannot be reached from any single 7th-axis station "
            "and need splitting.", tool, len(unreachable),
        )
    return {
        "tool": tool,
        "stations": stations,
        "station_count": len(stations),
        "total_travel_mm": total_travel,
        "unreachable_path_indices": unreachable,
    }
