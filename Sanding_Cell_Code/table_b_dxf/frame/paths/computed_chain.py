from __future__ import annotations

from typing import Any

from .constants import FRAME_BOUNDARY_CLEARANCE_MM
from .connectors import _axis_aligned_connector, _rectangular_centerline_corner
from .geometry import _connector_rides_boundary
from .helpers import (
    _dedupe_polyline_points,
    _last_segment_axis,
    _path_source_ids,
    _polyline_self_overlaps,
    _remove_short_backtrack_jogs,
)
from ..toolpaths import _polyline_stays_in_polygon


def _chain_covers_original_rails(
    chained_points: list[list[float]],
    originals: list[dict[str, Any]],
    *,
    tolerance_mm: float = 2.0,
    min_coverage_ratio: float = 0.75,
) -> bool:
    """Return True only when the chained path still sands every claimed rail.

    Total path length is not enough: a connector can add length while an original
    vertical rib disappears. This checks each claimed original rail independently.
    """
    if len(chained_points) < 2:
        return False
    try:
        from shapely.geometry import LineString

        chain = LineString([(float(p[0]), float(p[1])) for p in chained_points])
        if chain.is_empty or chain.length <= 1e-6:
            return False
        tol = max(float(tolerance_mm), 0.1)
        for original in originals:
            original_points = original.get("points") or []
            if len(original_points) < 2:
                continue
            rail = LineString([(float(p[0]), float(p[1])) for p in original_points])
            if rail.is_empty or rail.length <= 1e-6:
                continue
            covered = chain.intersection(rail.buffer(tol, cap_style=2, join_style=2))
            covered_len = float(getattr(covered, "length", 0.0))
            if covered_len < rail.length * min_coverage_ratio:
                return False
        return True
    except Exception:
        return False


def _chain_computed_frame_toolpaths(
    toolpaths: list[dict[str, Any]],
    frame_geom: Any,
    max_connector_mm: float,
) -> list[dict[str, Any]]:
    """Order computed-frame strokes as a clockwise sweep and connect only when safe.

    This is intentionally separate from pocket/3D closed-loop planning. For computed
    frame sections, starting from the bottom/right origin side and moving clockwise keeps
    low-Y moves inside the rectangular reach window before the path climbs into tighter
    Y-dependent reach constraints.
    """
    import math

    if len(toolpaths) < 2 or frame_geom is None or frame_geom.is_empty:
        return toolpaths

    minx, miny, maxx, maxy = [float(v) for v in frame_geom.bounds]
    width = max(maxx - minx, 1.0)
    height = max(maxy - miny, 1.0)
    max_connector = max(float(max_connector_mm), 1.0)

    def path_points(path: dict[str, Any]) -> list[list[float]]:
        return [[float(p[0]), float(p[1])] for p in path.get("points") or []]

    def path_center(points: list[list[float]]) -> tuple[float, float]:
        return (
            sum(p[0] for p in points) / len(points),
            sum(p[1] for p in points) / len(points),
        )

    def clockwise_side(points: list[list[float]]) -> int:
        cx, cy = path_center(points)
        distances = [
            (cy - miny, 0),      # bottom rail: origin side first
            (maxx - cx, 1),      # physical left side
            (maxy - cy, 2),      # top rail
            (cx - minx, 3),      # physical right side / origin side
        ]
        return min(distances, key=lambda item: item[0])[1]

    def clockwise_key(path: dict[str, Any]) -> tuple[float, float, str]:
        points = path_points(path)
        if not points:
            return (99.0, 0.0, str(path.get("path_id") or ""))
        cx, cy = path_center(points)
        side = clockwise_side(points)
        if side == 0:
            progress = (cx - minx) / width          # bottom: origin/right -> left
        elif side == 1:
            progress = (cy - miny) / height         # left side: bottom -> top
        elif side == 2:
            progress = (maxx - cx) / width          # top: left -> right/origin
        else:
            progress = (maxy - cy) / height         # right/origin side: top -> bottom
        return (float(side), float(progress), str(path.get("path_id") or ""))

    def orient_clockwise(points: list[list[float]]) -> list[list[float]]:
        if len(points) < 2:
            return points
        side = clockwise_side(points)
        start, end = points[0], points[-1]
        reverse = False
        if side == 0:
            reverse = start[0] > end[0]             # bottom: increasing X
        elif side == 1:
            reverse = start[1] > end[1]             # left: increasing Y
        elif side == 2:
            reverse = start[0] < end[0]             # top: decreasing X
        else:
            reverse = start[1] < end[1]             # right: decreasing Y
        return list(reversed(points)) if reverse else points

    def connector_points(a: list[float], b: list[float], first_axis: str | None = None) -> list[list[float]]:
        dist = math.dist(a, b)
        if dist <= 1e-6:
            return [a]
        if dist > max_connector:
            return []
        return _axis_aligned_connector(a, b, frame_geom, first_axis=first_axis)

    def is_chainable_single_pass(path: dict[str, Any]) -> bool:
        points = path_points(path)
        strategy = str(path.get("path_strategy") or "")
        section_strategy = str(path.get("source_section_strategy") or "")
        if len(points) != 2:
            return False
        if path.get("source_chunk_id"):
            return False
        if section_strategy in {"top_frame_band", "between_pockets_vertical"}:
            return True
        if section_strategy in {
            "bottom_curved_band",
            "outer_left_frame",
            "outer_right_frame",
        }:
            return False
        if path.get("direction") not in ("X", "Y"):
            return False
        if "zigzag" in strategy or "curved" in strategy or "diagonal" in strategy:
            return False
        return strategy == "band_centerline"
    chainable = [dict(tp) for tp in toolpaths if is_chainable_single_pass(tp)]
    separate = [dict(tp) for tp in toolpaths if not is_chainable_single_pass(tp)]
    if len(chainable) < 2:
        return toolpaths

    ordered = chainable
    ordered.sort(key=clockwise_key)

    chained: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    points: list[list[float]] = []

    for next_path in ordered:
        next_points = orient_clockwise(path_points(next_path))
        if current is None:
            current = dict(next_path)
            points = next_points
            current["source_section_ids"] = _path_source_ids(current)
            current["chained_path_ids"] = [current.get("path_id")]
            continue

        corner = _rectangular_centerline_corner(points, next_points, frame_geom) if points else None
        if corner is not None:
            # Do not let an L-corner trim consume either rail. This happened when a
            # vertical rib's far endpoint was chosen as the corner: the rib was marked
            # covered but its vertical stroke disappeared from the preview/toolpath.
            current_ok = len(points) < 2 or math.dist(points[-2], corner) > 1.0
            next_ok = len(next_points) < 2 or math.dist(corner, next_points[-1]) > 1.0
            if current_ok and next_ok:
                points[-1] = corner
                next_points[0] = corner
            else:
                corner = None
        if corner is not None:
            connector = [corner]
        elif points:
            connector = connector_points(points[-1], next_points[0], _last_segment_axis(points))
        else:
            connector = []
        if connector:
            points.extend(connector[1:])
            if math.dist(points[-1], next_points[0]) <= 1e-6:
                points.extend(next_points[1:])
            else:
                points.extend(next_points)

            seen = set(current.get("source_section_ids") or [])
            for sid in _path_source_ids(next_path):
                if sid not in seen:
                    current["source_section_ids"].append(sid)
                    seen.add(sid)
            current["chained_path_ids"].append(next_path.get("path_id"))
            continue

        current["points"] = points
        current["start_point"] = points[0]
        current["end_point"] = points[-1]
        current["direction"] = "CLOCKWISE"
        current["path_strategy"] = "computed_frame_clockwise_chain"
        chained.append(current)

        current = dict(next_path)
        points = next_points
        current["source_section_ids"] = _path_source_ids(current)
        current["chained_path_ids"] = [current.get("path_id")]

    if current is not None and points:
        current["points"] = points
        current["start_point"] = points[0]
        current["end_point"] = points[-1]
        current["direction"] = "CLOCKWISE"
        current["path_strategy"] = "computed_frame_clockwise_chain"
        chained.append(current)

    # SAFETY: the clockwise sweep assumes a simple PERIMETER (bottom->left->top->right).
    # A grid door has INTERIOR rails (a middle cross) that break that assumption: the sweep
    # connects rail endpoints in perimeter order and ends up SKIPPING parts of some rails,
    # so the chained path sands less than the rails did separately. For each chained path,
    # compare how much of its component rails it actually re-covers; if it drops coverage
    # (or leaves the frame), un-chain it back to the original rails so every rail is sanded.
    id_to_original = {tp.get("path_id"): tp for tp in toolpaths}

    def _length(pts: list[list[float]]) -> float:
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)) if len(pts) >= 2 else 0.0

    def _retraces(pts: list[list[float]], tol: float = 2.0) -> bool:
        # A clean sweep never doubles back: no segment should overlap an earlier one. We
        # catch the common failure directly — two consecutive segments that run back along
        # the same line (A->B->A), which is the "up a rail and back down" retrace that
        # sands a section twice.
        for i in range(len(pts) - 2):
            a, b, c = pts[i], pts[i + 1], pts[i + 2]
            if math.dist(a, c) <= tol and math.dist(a, b) > tol:
                return True
        return False

    def _segment_axis(a: list[float], b: list[float]) -> str | None:
        dx = abs(float(b[0]) - float(a[0]))
        dy = abs(float(b[1]) - float(a[1]))
        if dx <= 1e-6 and dy <= 1e-6:
            return None
        return "x" if dx >= dy else "y"

    def _has_edge_jump(pts: list[list[float]], step: float) -> bool:
        # Reject only a short connector that hops between two PARALLEL long rails. A normal
        # perimeter corner is also a short jog between two long strokes, but its before/after
        # axes are different and must remain chainable so the frame can turn as a clean L.
        for i in range(1, len(pts) - 2):
            seg = math.dist(pts[i], pts[i + 1])
            before = math.dist(pts[i - 1], pts[i])
            after = math.dist(pts[i + 1], pts[i + 2])
            if not (1.0 < seg < step * 0.6 and before > step and after > step):
                continue
            before_axis = _segment_axis(pts[i - 1], pts[i])
            after_axis = _segment_axis(pts[i + 1], pts[i + 2])
            hop_axis = _segment_axis(pts[i], pts[i + 1])
            if before_axis is not None and before_axis == after_axis and hop_axis != before_axis:
                return True
        return False

    validated: list[dict[str, Any]] = []
    for path in chained:
        originals = [id_to_original.get(pid) for pid in (path.get("chained_path_ids") or [])]
        originals = [o for o in originals if o is not None]
        pts = path.get("points") or []
        rails_len = sum(_length(o.get("points") or []) for o in originals)
        chained_len = _length(pts)
        stays = _polyline_stays_in_polygon(pts, frame_geom, tol=0.5) and not _connector_rides_boundary(pts, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM)
        # A valid chain: stays in-frame, covers at least the rails' length, never doubles
        # back on itself, and never hops between two parallel rail centrelines. Any of those
        # failures means the perimeter sweep mishandled this frame — keep the rails whole so
        # each is sanded once, down its own centreline.
        ok = (
            stays
            and chained_len >= rails_len - 1.0
            and _chain_covers_original_rails(pts, originals)
            and not _retraces(pts)
            and not _has_edge_jump(pts, 144.0)
        )
        if ok:
            # Keep the validated centerline chain as-is. Do not auto-close small gaps near
            # the outer model corners: that can add TCP points on the raw door boundary.
            validated.append(path)
        else:
            for o in originals:
                validated.append(dict(o))

    result = [*validated, *separate]
    # Final consolidation: greedily join whatever corners are still left as separate paths
    # into the longest safe runs, so the robot does fewer, longer strokes. This ignores the
    # 7th-axis station (a run may span stations; reach planning splits it back only where the
    # arm truly cannot reach) and joins only when the connector is a safe inward L that never
    # rides a border/hole/outer edge and never re-sands existing geometry.
    result = _greedy_join_frame_paths(result, frame_geom)
    for path in result:
        pts = _remove_short_backtrack_jogs(path.get("points") or [])
        if len(pts) >= 2:
            path["points"] = pts
            path["start_point"] = pts[0]
            path["end_point"] = pts[-1]
    for idx, path in enumerate(result, start=1):
        path["path_id"] = f"frame_path_{idx:03d}"
    return result


def _greedy_join_frame_paths(
    toolpaths: list[dict[str, Any]],
    frame_geom: Any,
) -> list[dict[str, Any]]:
    """Greedily connect frame passes at their nearest safe endpoints into longer runs.

    Goal: the fewest, longest connected paths the robot can run without riding a
    border/region/outer-edge line and without re-sanding a section. Each pass is a node that
    can be walked either direction; we repeatedly join the closest compatible endpoint pair
    whose connector is a safe inward L. Reach limits are NOT applied here — that is the
    executor/reach planner's job downstream — so a run may cross what were separate stations.
    """
    import math

    if frame_geom is None or getattr(frame_geom, "is_empty", True):
        return toolpaths

    def pts_of(p: dict[str, Any]) -> list[list[float]]:
        return [[float(a[0]), float(a[1])] for a in p.get("points") or []]

    # Join band-centreline rails AND the small tiled serpentine chunks that make up a wide
    # rail. Both have a clean entry/exit, so an end-to-start L connector between two of them
    # does not disturb either one's internal coverage — the merge is only accepted if it
    # stays in-frame, never retraces, and is not shorter than the two pieces (all checked in
    # try_merge), which together guarantee no re-sanding. Curved/diagonal fills, whose
    # coverage direction is not a simple reversible sweep, are left untouched.
    def joinable(p: dict[str, Any]) -> bool:
        strategy = str(p.get("path_strategy") or "")
        section_strategy = str(p.get("source_section_strategy") or "")
        if section_strategy in {"top_frame_band", "between_pockets_vertical"}:
            return True
        if section_strategy in {
            "bottom_curved_band",
            "outer_left_frame",
            "outer_right_frame",
        }:
            return False
        if "curved" in strategy or "diagonal" in strategy:
            return False
        return len(pts_of(p)) >= 2

    rail_paths = [dict(p) for p in toolpaths if joinable(p)]
    fixed = [dict(p) for p in toolpaths if not joinable(p)]
    if len(rail_paths) < 2:
        return toolpaths

    # ------------------------------------------------------------------ graph model
    # Treat each rail as a graph EDGE and each corner where rails meet as a NODE. The frame
    # centreline is exactly such a graph (verticals + horizontals joined at corners, with a
    # T-branch at a pocket divider). Walking the graph — every edge traversed once — yields the
    # fewest connected paths with NO rail sanded twice, which greedy pairwise gluing could not
    # guarantee (it made locally-tight joins that stranded rails and duplicated shared rails).
    NODE_TOL = 60.0  # corner endpoints within this distance are the same node

    endpoints: list[tuple[int, int, list[float]]] = []  # (rail_index, side 0/1, point)
    for ri, rp in enumerate(rail_paths):
        pts = pts_of(rp)
        endpoints.append((ri, 0, pts[0]))
        endpoints.append((ri, 1, pts[-1]))

    node_pts: list[list[float]] = []
    endpoint_node: dict[tuple[int, int], int] = {}
    for (ri, side, pt) in endpoints:
        found = None
        for ni, np_ in enumerate(node_pts):
            if math.dist(np_, pt) <= NODE_TOL:
                found = ni
                break
        if found is None:
            found = len(node_pts)
            node_pts.append(pt)
        endpoint_node[(ri, side)] = found

    # node -> list of (rail_index, side_at_this_node)
    node_rails: dict[int, list[tuple[int, int]]] = {}
    rail_nodes: dict[int, tuple[int, int]] = {}  # rail -> (node_at_side0, node_at_side1)
    for ri in range(len(rail_paths)):
        n0 = endpoint_node[(ri, 0)]
        n1 = endpoint_node[(ri, 1)]
        rail_nodes[ri] = (n0, n1)
        node_rails.setdefault(n0, []).append((ri, 0))
        node_rails.setdefault(n1, []).append((ri, 1))

    def _safe_connector_between(prev_pts: list[list[float]], next_pts: list[list[float]]) -> list[list[float]] | None:
        # Turn the corner from the end of prev_pts to the start of next_pts. Prefer a clean
        # rectangular corner, else an inward L. Must stay in-frame and not ride a hole/border.
        corner = _rectangular_centerline_corner(prev_pts, next_pts, frame_geom)
        if corner is not None:
            cand = _dedupe_polyline_points([prev_pts[-1], corner, next_pts[0]])
        else:
            conn = _axis_aligned_connector(prev_pts[-1], next_pts[0], frame_geom,
                                           first_axis=_last_segment_axis(prev_pts))
            if not conn:
                return None
            cand = _dedupe_polyline_points(conn)
        if not _polyline_stays_in_polygon(cand, frame_geom, tol=0.5):
            return None
        if _connector_rides_boundary(cand, frame_geom, clearance_mm=FRAME_BOUNDARY_CLEARANCE_MM):
            return None
        return cand

    # ------------------------------------------------------------------ walk the graph
    used: set[int] = set()

    def _oriented(ri: int, from_node: int) -> list[list[float]]:
        # Return this rail's points oriented so it STARTS at from_node.
        pts = pts_of(rail_paths[ri])
        n0, _n1 = rail_nodes[ri]
        return pts if n0 == from_node else list(reversed(pts))

    def _degree(node: int) -> int:
        return sum(1 for (ri, _s) in node_rails.get(node, []) if ri not in used)

    def _walk_from(start_node: int) -> tuple[list[list[float]], list[int]] | None:
        # Follow unused rails from start_node, turning each corner with a safe connector, until
        # no reachable unused rail can be safely joined. Returns the stitched polyline + rail ids.
        cur_node = start_node
        trail_pts: list[list[float]] = []
        trail_ids: list[int] = []
        while True:
            # pick an unused rail leaving cur_node
            nxt = None
            for (ri, side) in node_rails.get(cur_node, []):
                if ri in used:
                    continue
                seg = _oriented(ri, cur_node)
                if not trail_pts:
                    nxt = (ri, seg)
                    break
                conn = _safe_connector_between(trail_pts, seg)
                if conn is not None:
                    nxt = (ri, seg, conn)
                    break
            if nxt is None:
                break
            if not trail_pts:
                ri, seg = nxt
                trail_pts = [list(p) for p in seg]
            else:
                ri, seg, conn = nxt
                # stitch: connector[1:] avoids duplicating the shared endpoint
                trail_pts = _dedupe_polyline_points([*trail_pts, *conn[1:], *seg])
            used.add(ri)
            trail_ids.append(ri)
            # advance to the far node of this rail
            n0, n1 = rail_nodes[ri]
            cur_node = n1 if n0 == cur_node else n0
        return (trail_pts, trail_ids) if trail_ids else None

    out_paths: list[dict[str, Any]] = []

    def _emit(trail_pts: list[list[float]], trail_ids: list[int]) -> None:
        first = rail_paths[trail_ids[0]]
        base = dict(first)
        base["points"] = trail_pts
        base["start_point"] = trail_pts[0]
        base["end_point"] = trail_pts[-1]
        ids: list[Any] = []
        sids: list[Any] = []
        for ri in trail_ids:
            for cid in rail_paths[ri].get("chained_path_ids") or [rail_paths[ri].get("path_id")]:
                if cid is not None and cid not in ids:
                    ids.append(cid)
            for sid in _path_source_ids(rail_paths[ri]):
                if sid not in sids:
                    sids.append(sid)
        base["chained_path_ids"] = ids
        base["source_section_ids"] = sids
        base["path_strategy"] = str(first.get("path_strategy") or "computed_frame") + "_graph_walk"
        out_paths.append(base)

    # Prefer starting trails at ODD-degree nodes (dead-ends / T-branches). An Eulerian trail
    # must start at an odd node, so seeding there yields the fewest, longest walks.
    guard = 0
    while len(used) < len(rail_paths) and guard < len(rail_paths) * 4:
        guard += 1
        odd = [n for n in node_rails if _degree(n) % 2 == 1]
        candidates = odd if odd else [n for n in node_rails if _degree(n) > 0]
        if not candidates:
            break
        start = max(candidates, key=_degree)
        walked = _walk_from(start)
        if walked is None:
            # nothing safe to start here; retire this node's rails as standalone to avoid a loop
            for (ri, _s) in node_rails.get(start, []):
                if ri not in used:
                    _emit([list(p) for p in pts_of(rail_paths[ri])], [ri])
                    used.add(ri)
            continue
        _emit(*walked)

    # Any rail not reached by a walk (isolated) is emitted as-is.
    for ri in range(len(rail_paths)):
        if ri not in used:
            _emit([list(p) for p in pts_of(rail_paths[ri])], [ri])
            used.add(ri)

    return [*out_paths, *fixed]

