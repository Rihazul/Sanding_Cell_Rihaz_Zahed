from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple
import math
import time


Pose = List[float]


@dataclass
class Waypoint2Config:
    speed: float = 150.0
    accel: float = 300.0
    radius: float = 8.0
    min_seg_len: float = 5.0
    min_angle_deg: float = 12.0
    max_angle_deg: float = 170.0
    use_arc: bool = True
    use_wp2_for_line: bool = True
    enforce_orientation: str = "start"  # "start", "prev", "none"
    wait_timeout_s: float = 20.0
    cmd_id_prefix: str = "wp2"
    line_cmd_id_prefix: str = "wp2L"


@dataclass
class SegmentPlan:
    kind: str  # "arc" or "line"
    start_idx: int
    aux_idx: Optional[int]
    end_idx: int
    reason: str
    angle_deg: float
    radius: float
    seg_len_1: float
    seg_len_2: float


def _pad_pose(pose: Sequence[float]) -> Pose:
    padded = list(pose[:6])
    while len(padded) < 6:
        padded.append(0.0)
    return padded


def _pose_with_orientation(pose: Sequence[float], orient: Sequence[float]) -> Pose:
    padded = _pad_pose(pose)
    padded[3] = orient[3]
    padded[4] = orient[4]
    padded[5] = orient[5]
    return padded


def _vec(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _dot(v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> float:
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def _cross(v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    )


def _norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(_dot(v, v))


def _angle_deg(v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> float:
    n1 = _norm(v1)
    n2 = _norm(v2)
    if n1 <= 1e-9 or n2 <= 1e-9:
        return 0.0
    cosang = _dot(v1, v2) / (n1 * n2)
    cosang = max(-1.0, min(1.0, cosang))
    return math.degrees(math.acos(cosang))


def _collinear(p0: Sequence[float], p1: Sequence[float], p2: Sequence[float], tol: float = 1e-6) -> bool:
    v1 = _vec(p0, p1)
    v2 = _vec(p1, p2)
    cr = _cross(v1, v2)
    return _norm(cr) <= tol


def generate_spiral_points_between(
    start_pose: Sequence[float],
    end_pose: Sequence[float],
    *,
    radius: float = 12.0,
    angle_step_deg: float = 45.0,
    max_points: Optional[int] = None,
) -> List[Pose]:
    """
    Generate a helical spiral between two poses, returning [x,y,z,rx,ry,rz].

    The circle center moves linearly from start to end, while the point
    rotates around the start->end axis. This replicates the MovePathL
    "circle while moving linearly" behavior in 3D.

    Note: consecutive triplets are not guaranteed to lie on a single circle,
    so WayPoint2 type-2 arcs should be disabled (use_arc=False) when using
    these points.
    """
    start_pose = _pad_pose(start_pose)
    end_pose = _pad_pose(end_pose)
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, z1, _, _, _ = end_pose[:6]

    axis = _vec(start_pose, end_pose)
    axis_len = _norm(axis)
    if axis_len <= 1e-9:
        return [start_pose[:6]]

    ax = (axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len)
    # Build an orthonormal basis (u, v) perpendicular to axis.
    if abs(ax[2]) < 0.9:
        ref = (0.0, 0.0, 1.0)
    else:
        ref = (0.0, 1.0, 0.0)
    u = _cross(ax, ref)
    u_len = _norm(u)
    if u_len <= 1e-9:
        ref = (1.0, 0.0, 0.0)
        u = _cross(ax, ref)
        u_len = _norm(u)
        if u_len <= 1e-9:
            return [start_pose[:6]]
    u = (u[0] / u_len, u[1] / u_len, u[2] / u_len)
    v = _cross(ax, u)

    dist = axis_len
    turns = max(1, int(math.ceil(dist / (radius * 2.0))))
    total_steps = int(turns * (360.0 / angle_step_deg))
    if max_points is not None:
        total_steps = max(1, min(total_steps, int(max_points)))
    else:
        total_steps = max(1, total_steps)

    points: List[Pose] = [start_pose[:6]]
    for step in range(1, total_steps + 1):
        t = step / total_steps
        cx = x0 + (x1 - x0) * t
        cy = y0 + (y1 - y0) * t
        cz = z0 + (z1 - z0) * t
        theta_deg = step * angle_step_deg
        theta = math.radians(theta_deg)
        ctheta = math.cos(theta)
        stheta = math.sin(theta)
        px = cx + radius * (ctheta * u[0] + stheta * v[0])
        py = cy + radius * (ctheta * u[1] + stheta * v[1])
        pz = cz + radius * (ctheta * u[2] + stheta * v[2])
        points.append([px, py, pz, rx, ry, rz])

    return points


def generate_arc_line_segments_between(
    start_pose: Sequence[float],
    end_pose: Sequence[float],
    *,
    radius: float = 12.0,
    arc_step_deg: float = 120.0,
    pitch: Optional[float] = None,
    clockwise: bool = True,
    max_circles: Optional[int] = None,
    bounds: Optional[Tuple[float, float, float, float]] = None,
    safety_margin: float = 0.0,
    min_radius: float = 1.0,
) -> List[List[Pose]]:
    """
    Generate a helix-like path using true circular arcs (WayPoint2 type-2)
    separated by short linear advances (type-1).

    The motion stays in the XY plane (constant Z), circles are centered on a line
    from start->end, and each full circle is built from multiple arcs with angle
    < 180 degrees. Between circles, a straight line advances by `pitch` along the
    start->end direction.
    """
    start_pose = _pad_pose(start_pose)
    end_pose = _pad_pose(end_pose)
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]

    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return [[start_pose[:6]]]

    dir_xy = (dx / dist, dy / dist)
    perp = (-dir_xy[1], dir_xy[0])

    if pitch is None or pitch <= 1e-9:
        pitch = radius * 2.0

    num_circles = max(1, int(math.floor(dist / pitch)))
    if max_circles is not None:
        num_circles = max(1, min(num_circles, int(max_circles)))

    arc_step_deg = float(max(5.0, min(arc_step_deg, 170.0)))
    num_arcs = max(1, int(math.ceil(360.0 / arc_step_deg)))
    arc_deg = 360.0 / num_arcs
    half_step_deg = arc_deg / 2.0

    sign = -1.0 if clockwise else 1.0

    # In-plane orthonormal basis for the circle.
    u = perp
    v = (-perp[1], perp[0])

    segments: List[List[Pose]] = []
    last_start = (x0, y0)

    for i in range(num_circles):
        axis_x = x0 + dir_xy[0] * pitch * i
        axis_y = y0 + dir_xy[1] * pitch * i

        r_eff = radius
        if bounds is not None:
            xmin, xmax, ymin, ymax = bounds
            xmin += safety_margin
            xmax -= safety_margin
            ymin += safety_margin
            ymax -= safety_margin
            # Iteratively shrink radius so the full circle stays inside bounds.
            for _ in range(3):
                cx = axis_x - r_eff * u[0]
                cy = axis_y - r_eff * u[1]
                max_r = min(cx - xmin, xmax - cx, cy - ymin, ymax - cy)
                if max_r < r_eff:
                    r_eff = max_r
                else:
                    break
            r_eff = max(0.0, r_eff)

        start_i = (axis_x, axis_y)
        if i > 0:
            # Linear advance to the next circle start.
            segments.append(
                [
                    [last_start[0], last_start[1], z0, rx, ry, rz],
                    [start_i[0], start_i[1], z0, rx, ry, rz],
                ]
            )
        last_start = start_i

        if r_eff >= min_radius:
            cx = axis_x - r_eff * u[0]
            cy = axis_y - r_eff * u[1]
            circle_points: List[Pose] = []
            for k in range(0, 2 * num_arcs + 1):
                theta = math.radians(sign * (k * half_step_deg))
                ctheta = math.cos(theta)
                stheta = math.sin(theta)
                px = cx + r_eff * (ctheta * u[0] + stheta * v[0])
                py = cy + r_eff * (ctheta * u[1] + stheta * v[1])
                circle_points.append([px, py, z0, rx, ry, rz])

            segments.append(circle_points)

    # Final linear advance to end_pose.
    segments.append([[last_start[0], last_start[1], z0, rx, ry, rz], [x1, y1, z0, rx, ry, rz]])
    return segments


def expand_spiral_points(
    points: Iterable[Sequence[float]],
    *,
    radius: float = 12.0,
    angle_step_deg: float = 45.0,
    max_points_per_segment: Optional[int] = None,
) -> List[Pose]:
    """
    Expand each consecutive pair of points into a spiral sub-path and
    concatenate them into a single list of poses for WayPoint2.
    """
    pts = [_pad_pose(p) for p in points]
    if len(pts) < 2:
        return pts

    expanded: List[Pose] = []
    for idx in range(len(pts) - 1):
        start = pts[idx]
        end = pts[idx + 1]
        segment = generate_spiral_points_between(
            start,
            end,
            radius=radius,
            angle_step_deg=angle_step_deg,
            max_points=max_points_per_segment,
        )
        if expanded and segment:
            segment = segment[1:]  # drop duplicate start
        expanded.extend(segment)

    return expanded


def _circle_radius(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> float:
    ab = _vec(a, b)
    ac = _vec(a, c)
    bc = _vec(b, c)
    cross_norm = _norm(_cross(ab, ac))
    if cross_norm <= 1e-9:
        return float("inf")
    return (_norm(ab) * _norm(bc) * _norm(ac)) / (2.0 * cross_norm)


def _arc_diagnostics(
    prev: Sequence[float],
    aux: Sequence[float],
    end: Sequence[float],
    *,
    min_seg_len: float,
    min_angle_deg: float,
    max_angle_deg: float,
) -> Tuple[bool, str, float, float, float, float]:
    v1 = _vec(prev, aux)
    v2 = _vec(aux, end)
    seg1 = _norm(v1)
    seg2 = _norm(v2)
    if seg1 < min_seg_len or seg2 < min_seg_len:
        return False, "segment too short", 0.0, float("inf"), seg1, seg2
    if _collinear(prev, aux, end):
        return False, "collinear or coincident", 0.0, float("inf"), seg1, seg2
    ang = _angle_deg(v1, v2)
    if ang < min_angle_deg or ang > max_angle_deg:
        return False, f"angle out of range ({ang:.2f}°)", ang, _circle_radius(prev, aux, end), seg1, seg2
    return True, "ok", ang, _circle_radius(prev, aux, end), seg1, seg2


def plan_waypoint2_segments(
    points: Iterable[Sequence[float]],
    cfg: Waypoint2Config,
) -> List[SegmentPlan]:
    pts = [_pad_pose(p) for p in points]
    if len(pts) < 2:
        return []

    plans: List[SegmentPlan] = []
    idx = 1
    current_idx = 0
    while idx < len(pts):
        if cfg.use_arc and idx + 1 < len(pts):
            ok, reason, ang, radius, seg1, seg2 = _arc_diagnostics(
                pts[current_idx],
                pts[idx],
                pts[idx + 1],
                min_seg_len=cfg.min_seg_len,
                min_angle_deg=cfg.min_angle_deg,
                max_angle_deg=cfg.max_angle_deg,
            )
            if ok:
                plans.append(
                    SegmentPlan(
                        kind="arc",
                        start_idx=current_idx,
                        aux_idx=idx,
                        end_idx=idx + 1,
                        reason=reason,
                        angle_deg=ang,
                        radius=radius,
                        seg_len_1=seg1,
                        seg_len_2=seg2,
                    )
                )
                current_idx = idx + 1
                idx += 2
                continue
            else:
                plans.append(
                    SegmentPlan(
                        kind="line",
                        start_idx=current_idx,
                        aux_idx=None,
                        end_idx=idx,
                        reason=reason,
                        angle_deg=ang,
                        radius=radius,
                        seg_len_1=seg1,
                        seg_len_2=seg2,
                    )
                )
                current_idx = idx
                idx += 1
                continue

        plans.append(
            SegmentPlan(
                kind="line",
                start_idx=current_idx,
                aux_idx=None,
                end_idx=idx,
                reason="end segment",
                angle_deg=0.0,
                radius=float("inf"),
                seg_len_1=_norm(_vec(pts[current_idx], pts[idx])),
                seg_len_2=0.0,
            )
        )
        current_idx = idx
        idx += 1

    return plans


def _should_use_arc(
    prev: Sequence[float],
    aux: Sequence[float],
    end: Sequence[float],
    *,
    min_seg_len: float,
    min_angle_deg: float,
    max_angle_deg: float,
) -> bool:
    v1 = _vec(prev, aux)
    v2 = _vec(aux, end)
    if _norm(v1) < min_seg_len or _norm(v2) < min_seg_len:
        return False
    if _collinear(prev, aux, end):
        return False
    ang = _angle_deg(v1, v2)
    if ang < min_angle_deg or ang > max_angle_deg:
        return False
    return True


def _get_joint_seed(cps) -> Pose:
    result: List[str] = []
    ret = cps.HRIF_ReadActPos(0, 0, result)
    if ret == 0 and len(result) >= 6:
        try:
            return [float(v) for v in result[:6]]
        except (TypeError, ValueError):
            return [0.0] * 6
    return [0.0] * 6


def _wait_motion_done(cps, timeout_s: float) -> bool:
    start = time.time()
    result: List[str] = []
    while True:
        result.clear()
        ret = cps.HRIF_IsMotionDone(0, 0, result)
        if ret == 0 and result:
            last = result[-1]
            done = (isinstance(last, bool) and last) or str(last).strip().lower() in ("1", "true", "ok")
            if done:
                return True
        if time.time() - start > timeout_s:
            return False
        time.sleep(0.02)


def execute_waypoint2_path(
    cps,
    points: Iterable[Sequence[float]],
    *,
    tcp: str,
    ucs: str,
    cfg: Waypoint2Config,
    wait_each: bool = True,
    wait_end: bool = False,
    move_l_fn: Optional[Callable[..., object]] = None,
    move_l_kwargs: Optional[Dict[str, object]] = None,
    box_id: int = 0,
    robot_id: int = 0,
    is_joint: int = 0,
    is_seek: int = 0,
    bit: int = 0,
    state: int = 0,
    logger: Optional[object] = None,
    dry_run: bool = False,
    return_plan: bool = False,
) -> Dict[str, object]:
    pts = [_pad_pose(p) for p in points]
    if len(pts) < 2:
        return {"ok": True, "arcs": 0, "lines": 0, "failed": 0}

    if cfg.enforce_orientation == "start":
        ref_orient = pts[0]
    else:
        ref_orient = None

    plan = plan_waypoint2_segments(pts, cfg)
    if dry_run:
        return {
            "ok": True,
            "arcs": sum(1 for p in plan if p.kind == "arc"),
            "lines": sum(1 for p in plan if p.kind == "line"),
            "failed": 0,
            "plan": [asdict(p) for p in plan] if return_plan else None,
        }

    joint_seed = _get_joint_seed(cps)
    arcs = 0
    lines = 0
    failed = 0

    current = pts[0]
    for seg in plan:
        if seg.kind == "arc":
            prev = pts[seg.start_idx]
            aux = pts[seg.aux_idx] if seg.aux_idx is not None else None
            end = pts[seg.end_idx]

            if cfg.enforce_orientation == "start" and ref_orient is not None:
                prev = _pose_with_orientation(prev, ref_orient)
                aux = _pose_with_orientation(aux, ref_orient)
                end = _pose_with_orientation(end, ref_orient)
            elif cfg.enforce_orientation == "prev":
                prev = _pad_pose(prev)
                aux = _pose_with_orientation(aux, prev)
                end = _pose_with_orientation(end, prev)

            cmd_id = f"{cfg.cmd_id_prefix}-{arcs + 1}"
            ret = cps.HRIF_WayPoint2(
                box_id,
                robot_id,
                2,
                end,
                aux,
                joint_seed,
                tcp,
                ucs,
                cfg.speed,
                cfg.accel,
                cfg.radius,
                is_joint,
                is_seek,
                bit,
                state,
                cmd_id,
            )
            if ret == 0:
                arcs += 1
                if wait_each:
                    _wait_motion_done(cps, cfg.wait_timeout_s)
                current = end
                continue

            failed += 1
            if logger:
                logger.warning(f"[WayPoint2] Move failed ret={ret}; falling back to MoveL.")
            else:
                print(f"[WayPoint2] Move failed ret={ret}; falling back to MoveL.")

            target = end
        else:
            target = pts[seg.end_idx]

        if cfg.enforce_orientation == "start" and ref_orient is not None:
            target = _pose_with_orientation(target, ref_orient)
        elif cfg.enforce_orientation == "prev":
            target = _pose_with_orientation(target, current)

        if cfg.use_wp2_for_line:
            cmd_id = f"{cfg.line_cmd_id_prefix}-{lines + 1}"
            ret = cps.HRIF_WayPoint2(
                box_id,
                robot_id,
                1,
                target,
                target,
                joint_seed,
                tcp,
                ucs,
                cfg.speed,
                cfg.accel,
                cfg.radius,
                is_joint,
                is_seek,
                bit,
                state,
                cmd_id,
            )
            if ret == 0:
                lines += 1
                if wait_each:
                    _wait_motion_done(cps, cfg.wait_timeout_s)
                current = target
                continue
            failed += 1
            if logger:
                logger.warning(f"[WayPoint2] Line move failed ret={ret}; falling back to MoveL.")
            else:
                print(f"[WayPoint2] Line move failed ret={ret}; falling back to MoveL.")

        if move_l_fn:
            kwargs = dict(move_l_kwargs or {})
            kwargs["point"] = target
            move_l_fn(**kwargs)
        else:
            failed += 1
            if logger:
                logger.error("[WayPoint2] No MoveL fallback provided; aborting.")
            return {"ok": False, "arcs": arcs, "lines": lines, "failed": failed}

        lines += 1
        current = target

    if wait_end and failed == 0:
        _wait_motion_done(cps, cfg.wait_timeout_s)

    result = {"ok": failed == 0, "arcs": arcs, "lines": lines, "failed": failed}
    if return_plan:
        result["plan"] = [asdict(p) for p in plan]
    return result
