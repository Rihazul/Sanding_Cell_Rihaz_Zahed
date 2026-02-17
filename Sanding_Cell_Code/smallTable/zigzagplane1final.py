# testcommu.py

import sys
import math
import os
import time
import json
import uuid
from typing import Optional

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Components.RobotState import RobotState
from Server_Better_V2 import (
    communicate,
    setup_logger,
    waitForBlending,
    setSpeed,
    turn_vibration_on,
    turn_vibration_off,
    putForce,
    releaseForce,
    putForceZplus,
    putForceZminus,
)
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
)


def compute_timeout(
    *,
    total_points: Optional[int] = None,
    cap: float = 180.0,
    velocity: float = 300.0,
) -> float:
    """
    Estimate a completion timeout based on total points.

    - Uses a simple model: time = (points / vmax) + overhead
    - Caps the timeout to avoid excessive waits.
    """
    if total_points is None:
        return cap

    distance_per_point = 4.64
    # vmax = 300.0
    # time_factor_max = float(total_points / vmax)
    time_factor = float(total_points * distance_per_point / (velocity))
    # Keep timeout aligned to the estimated runtime from points/speed.
    timeout = min(cap, max(1.0, time_factor))
    print(
        f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
    )
    return timeout


DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0
DEFAULT_MOVEJS_JERK_RATIO = 100
DEFAULT_MOVEJS_TRANSITION_DEG = 25
MOVEJS_MAX_POINTS = 50
USE_WAYPOINT2_ARC = True
WAYPOINT2_ARC_MIN_AREA = 0.2
WAYPOINT2_ARC_MIN_SEGMENT = 0.5
WAYPOINT2_MICRO_ARC_DEG = 10.0
WAYPOINT2_MAX_POSES = 0  # 0 disables downsampling for micro-arc chains
WAYPOINT2_BLEND_RADIUS = 2.0
WAYPOINT2_FORCE_SETTLE_S = 0.6
WAYPOINT2_START_DELAY_S = 0.35
WAYPOINT2_CMD_DELAY_S = 0.004
WAYPOINT2_BATCH_SIZE = 60
WAYPOINT2_BATCH_PAUSE_S = 0.08
WAYPOINT2_WAIT_TIMEOUT_S = 120.0
WAYPOINT2_USE_UCS_ORI = True
WAYPOINT2_FORCE_UCS_TILT = False
WAYPOINT2_UCS_RX = 0.0
WAYPOINT2_UCS_RY = 0.0
WAYPOINT2_DEBUG_ARC = True
WAYPOINT2_SNAP_TO_ACTUAL = True
WAYPOINT2_ORI_TOL = 0.05
WAYPOINT2_DETECT_FRAME = False
WAYPOINT2_ARC_LEADIN_TYPE1 = True
WAYPOINT2_MAX_START_GAP_MM = 120.0
WAYPOINT2_REQUIRE_PLANE1_FRAME = True
WAYPOINT2_SKIP_EDGE_COVERAGE = True
WAYPOINT2_SNAP_XY = False
WAYPOINT2_SNAP_Z = True
WAYPOINT2_USE_GLOBAL_SPIRAL = True
WAYPOINT2_GLOBAL_MARGIN_MM = 2.0
WAYPOINT2_GLOBAL_MIN_AXIS_MM = 8.0
WAYPOINT2_GLOBAL_MIN_POINTS = 40



def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _linear_speed_to_turns(speed_mm_s: float) -> int:
    """Map 100–300 mm/s to 20–10 turns (faster speed -> fewer turns)."""
    speed = _clamp(speed_mm_s, 100.0, 300.0)
    turns = 20.0 - (speed - 100.0) * (10.0 / 200.0)
    print(turns)
    return int(round(_clamp(turns, 10.0, 20.0)))


def apply_spiral_settings(settings: Optional[dict]) -> None:
    """Apply incoming spiral settings to defaults used during spiral generation."""
    global DEFAULT_SPIRAL_LINEAR_SPEED, DEFAULT_SPIRAL_RADIUS
    if not settings:
        return
    # Handle dataclass or plain dict
    radius_val = (
        getattr(settings, "radius_mm", None)
        if not isinstance(settings, dict)
        else settings.get("radius_mm")
    )
    linear_val = (
        getattr(settings, "linear_speed_mm_s", None)
        if not isinstance(settings, dict)
        else settings.get("linear_speed_mm_s")
    )
    DEFAULT_SPIRAL_LINEAR_SPEED = _clamp(
        float(linear_val if linear_val is not None else DEFAULT_SPIRAL_LINEAR_SPEED),
        100.0,
        300.0,
    )
    DEFAULT_SPIRAL_RADIUS = _clamp(
        float(radius_val if radius_val is not None else DEFAULT_SPIRAL_RADIUS),
        10.0,
        15.0,
    )


def generate_spiral_between_points(
    start_pose,
    end_pose,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = None,
    orientation: str = "horizontal",
):
    """
    Build a spiral path between two cartesian poses (X, Y, Z, Rx, Ry, Rz).
    Keeps orientation from start_pose, interpolates center XY between poses,
    and adds radial offsets for each step.
    """
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]
    if radius is None:
        radius = DEFAULT_SPIRAL_RADIUS
    if y0 == y1 and orientation == "vertical":
        turns = 1
    elif x0 == x1 and orientation == "horizontal":
        turns = 1
    else:
        dist = max(abs(x1 - x0), abs(y1 - y0))
        # factor = 1.0
        # if orientation == "vertical":
        #     factor= 1.3
        # turns = int((4.0/260.0) * float(dist) * float(factor))
        turns = math.ceil(dist/(radius*2.0))

    # Always map speed -> turns so higher speed yields fewer turns

    total_steps = int(turns * (360.0 / angle_step_deg))
    if max_points is not None:
        total_steps = max(1, min(total_steps, int(max_points)))
    else:
        total_steps = max(1, total_steps)
    points = list(start_pose[:6])  # start point

    for step in range(1, total_steps + 1):
        t = step / total_steps
        cx = x0 + (x1 - x0) * t
        cy = y0 + (y1 - y0) * t
        theta_deg = step * angle_step_deg
        theta = math.radians(theta_deg)

        px = cx + radius * math.cos(theta)
        py = cy + radius * math.sin(theta)
        pz = z0  # hold Z while spiraling along Y

        points.extend([px, py, pz, rx, ry, rz])

    return points


def run_spiral_between_points(
    cps: CPSClient,
    config: dict,
    start_pose,
    end_pose,
    *,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = None,
    tcp: str = None,
    ucs: str = None,
    track_name: str = None,
    velocity: float = 300.0,
    accel: float = 500.0,
    jerk: float = 10000.0,
    box_id: int = 0,
    robot_id: int = 0,
    init_path: bool = True,
    orientation: str = "horizontal",
):
    """Push a spiral MovePathL between two poses.

    By default (`finalize=False`), the path is only pushed (optionally initialized
    with `init_path`) and execution must be triggered separately via
    `finalize_spiral_path`. Set `finalize=True` to push and execute immediately.
    """
    track_name = track_name
    print(f"[Spiral] Using track name: {track_name}")
    tcp_name = tcp or config["coords"].get("tcptool1plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")

    all_points = generate_spiral_between_points(
        start_pose=start_pose,
        end_pose=end_pose,
        turns=turns,
        radius=DEFAULT_SPIRAL_RADIUS if radius is None else radius,
        angle_step_deg=angle_step_deg,
        max_points=max_points,
        orientation=orientation,
    )
    xs = all_points[0::6]
    ys = all_points[1::6]
    print("spiral x range:", min(xs), max(xs), " y range:", min(ys), max(ys))

    if len(all_points) % 6 != 0:
        print("[Spiral] Point list misaligned (not divisible by 6).")
        return False, None

    count = len(all_points) // 6
    print(f"[Spiral] Total points = {count}")

    if init_path:
        # Raw data type = 1 (PCS), speed ratio = 1.0, blending radius = 0.0
        ret = cps.HRIF_InitPath(
            box_id,
            robot_id,
            1,
            track_name,
            0.50,
            1.00,
            velocity,
            accel,
            jerk,
            ucs_name,
            tcp_name
        )
        print("[Spiral][Init] ret =", ret)
        if ret != 0:
            return False, None

    ret = cps.HRIF_PushPathPoints(box_id, robot_id, track_name, all_points)
    print("[Spiral][Push] ret =", ret)
    if ret != 0:
        return False, None

    return True, count


def finalize_spiral_path(
    cps: CPSClient,
    track_name: str,
    *,
    box_id: int = 0,
    robot_id: int = 0,
    completion_timeout=76.0,
    min_runtime_s: float = 0.0,
    force: Optional[float] = None,
    config: Optional[dict] = None,
    force_settle_s: float = 0.2,
) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False

    # Create a lightweight state helper for motion monitoring.
    # Reuse the existing config/cps to avoid reinitializing connections.
    robot_state = RobotState(config=config , cps_client=cps)

    # Wait for PATH_READY
    start = time.time()
    state = []
    while True:
        st = []
        ret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, st)
        if ret == 0 and len(st) > 3 and st[2] == "3" and st[3] == "0":
            break
        elapsed = time.time() - start
        if elapsed > 120.0:
            print(f"[Spiral] Timeout waiting for PATH_READY after {elapsed:.1f}s:", st)
            return False
        time.sleep(0.1)
        print(f"Waiting for PATH_READY... t={elapsed:.1f}s state={st}\r", end="")
    
    vibration_on = False
    force_applied = False
    ok = False
    motion_last_change = time.time()

    try:
        if force is not None and config is not None:
            # Apply force *after* path is ready
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )
            force_applied = True
            time.sleep(force_settle_s)
    
        ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
        print(f"[Spiral] Robot returned {ret} after MovePathL,")
        if ret != 0:
            return False
        move_start = time.time()
    
        # Wait briefly for motion to start, then turn vibration on.
        motion_started = False
        motion_wait_start = time.time()
        while time.time() - motion_wait_start < 1.0:
            if robot_state.fetch_and_update_flags():
                if robot_state.state["flags"].get("in_motion"):
                    motion_started = True
                    break
            time.sleep(0.02)
        
        turn_vibration_on(cps)
        vibration_on = True

        # Wait for completion (track path state + robot flags)
        ok = True
        start = time.time()
        estimate_timeout = float(completion_timeout)
        # Safety watchdog is only for abnormal hangs; estimate itself remains the target.
        safety_timeout = estimate_timeout + max(8.0, min(30.0, estimate_timeout * 0.3))
        timeout_notice_logged = False
        timeout_stall_start = None
        # Tolerances tuned to avoid waiting on noisy force-hold jitter at the last point.
        position_noise_mm = 0.6
        orientation_noise_deg = 0.8
        settle_window_s = 0.15
        near_end_ratio = 0.93
        motion_seen = motion_started
        last_cart = None
        pre_move_cart = None
        if robot_state.fetch_and_update_pos():
            pre_move_cart = list(robot_state.state.get("cartesian_position", []))
        while True:
            pstate = []
            pret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
            if pret != 0:
                print("HRIF_ReadPathState failed:", pret)
                ok = False
                break
            if len(pstate) > 3 and pstate[3] != "0":
                print(f"[Spiral] Path reported error: {pstate}")
                ok = False
                break
            elapsed_move = time.time() - move_start

            flags = {}
            if robot_state.fetch_and_update_flags():
                flags = robot_state.state.get("flags", {})
                if flags.get("in_motion"):
                    motion_seen = True
                # Fast-path completion check: controller confirms motion done and in-position.
                if (
                    motion_seen
                    and not flags.get("in_motion", True)
                    and flags.get("point_motion_done", False)
                    and flags.get("in_position", False)
                    and (time.time() - move_start) >= max(0.0, float(min_runtime_s))
                ):
                    print("[Spiral] Motion done + in-position flags observed; exiting wait loop.")
                    break

            # Controller-level motion completion can become true slightly earlier than
            # state-flags convergence, which helps reduce end-point dwell.
            motion_done = []
            mret = cps.HRIF_IsMotionDone(box_id, robot_id, motion_done)
            if mret == 0 and motion_done:
                last_done = motion_done[-1]
                done = (
                    (isinstance(last_done, bool) and last_done)
                    or str(last_done).strip().lower() in ("1", "true", "ok")
                )
                if done and motion_seen and elapsed_move >= max(0.0, float(min_runtime_s)):
                    print("[Spiral] IsMotionDone=1; exiting wait loop.")
                    break

            if robot_state.fetch_and_update_pos():
                curr_cart = list(robot_state.state.get("cartesian_position", []))
                if (
                    not motion_seen
                    and pre_move_cart
                    and len(curr_cart) >= 6
                    and len(pre_move_cart) >= 6
                    and (
                        any(
                            abs(a - b) > position_noise_mm
                            for a, b in zip(curr_cart[:3], pre_move_cart[:3])
                        )
                        or any(
                            abs(a - b) > orientation_noise_deg
                            for a, b in zip(curr_cart[3:6], pre_move_cart[3:6])
                        )
                    )
                ):
                    motion_seen = True
                if last_cart and len(curr_cart) >= 6 and len(last_cart) >= 6:
                    moved = (
                        any(abs(a - b) > position_noise_mm for a, b in zip(curr_cart[:3], last_cart[:3]))
                        or any(
                            abs(a - b) > orientation_noise_deg
                            for a, b in zip(curr_cart[3:6], last_cart[3:6])
                        )
                    )
                    if moved:
                        motion_seen = True
                        motion_last_change = time.time()
                else:
                    motion_last_change = time.time()
                last_cart = curr_cart

            # Fallback completion check once motion has started.
            in_motion_flag = flags.get("in_motion") if "in_motion" in flags else None
            near_expected_end = elapsed_move >= max(1.0, estimate_timeout * near_end_ratio)
            progress_recent = motion_seen and ((time.time() - motion_last_change) < settle_window_s)
            if (
                motion_seen
                and near_expected_end
                and (time.time() - motion_last_change) >= settle_window_s
                and (time.time() - move_start) >= max(0.0, float(min_runtime_s))
            ):
                print(f"[Spiral] Cartesian settled for {settle_window_s:.2f}s near end; exiting wait loop.")
                break

            elapsed = time.time() - start
            if elapsed > estimate_timeout:
                if not timeout_notice_logged:
                    print(
                        f"[Spiral] Estimated runtime reached: elapsed={elapsed:.1f}s, "
                        f"estimate={estimate_timeout:.1f}s; waiting for true completion."
                    )
                    timeout_notice_logged = True

                # If motion updates have stopped near the expected end, finish immediately
                # even when controller in-motion flag is stale.
                if motion_seen and near_expected_end and not progress_recent:
                    print("[Spiral] Motion progress stopped near end; exiting wait loop.")
                    break

                if progress_recent:
                    timeout_stall_start = None
                else:
                    if timeout_stall_start is None:
                        timeout_stall_start = time.time()

                stalled_too_long = (
                    timeout_stall_start is not None
                    and (time.time() - timeout_stall_start) >= 3.0
                )
                if elapsed > safety_timeout or stalled_too_long:
                    # Controller states can lag after path completion; if cartesian pose is settled
                    # and we're past the expected end, treat it as completed instead of failing.
                    if (
                        motion_seen
                        and near_expected_end
                        and (time.time() - motion_last_change) >= settle_window_s
                    ):
                        print(
                            f"[Spiral] Safety timeout reached but pose is settled for "
                            f"{settle_window_s:.2f}s; treating as complete."
                        )
                        break
                    print(
                        f"Timeout waiting for idle. elapsed={elapsed:.1f}s, "
                        f"estimate={estimate_timeout:.1f}s, safety={safety_timeout:.1f}s, "
                        f"stalled={stalled_too_long}, Path state: {pstate}"
                    )
                    ok = False
                    break
            time.sleep(0.02)

    finally:
        if vibration_on:
            turn_vibration_off(cps)
        if force_applied:
            releaseForce(cps=cps, config=config, wait_for_blending=False)

    return ok


def _poses_from_flat_points(points_flat):
    if not points_flat or len(points_flat) % 6 != 0:
        return []
    return [points_flat[i : i + 6] for i in range(0, len(points_flat), 6)]


def _distance_xyz(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def _arc_is_valid(p0, p1, p2, *, min_area=WAYPOINT2_ARC_MIN_AREA, min_seg=WAYPOINT2_ARC_MIN_SEGMENT):
    if _distance_xyz(p0, p1) < min_seg or _distance_xyz(p1, p2) < min_seg:
        return False
    area2 = abs(
        (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
    )
    return area2 >= min_area


def _read_current_tcp_rxyz(cps, box_id=0, robot_id=0):
    act = []
    ret = cps.HRIF_ReadActPos(box_id, robot_id, act)
    if ret != 0 or len(act) < 12:
        print("[WayPoint2] Failed to read TCP orientation from controller.")
        return None
    return [float(act[9]), float(act[10]), float(act[11])]


def _read_current_tcp_pose(cps, box_id=0, robot_id=0):
    act = []
    ret = cps.HRIF_ReadActPos(box_id, robot_id, act)
    if ret != 0 or len(act) < 12:
        print(f"[WayPoint2] Failed to read TCP pose from controller (ret={ret}).")
        return None
    return [
        float(act[6]),
        float(act[7]),
        float(act[8]),
        float(act[9]),
        float(act[10]),
        float(act[11]),
    ]


def _read_ucs_pose(cps, ucs_name, box_id=0, robot_id=0):
    if not ucs_name:
        return None
    result = []
    ret = cps.HRIF_ReadUCSByName(box_id, robot_id, ucs_name, result)
    if ret != 0 or len(result) < 6:
        print(f"[WayPoint2] Failed to read UCS '{ucs_name}' (ret={ret}).")
        return None
    try:
        return [float(v) for v in list(result)[:6]]
    except Exception:
        return None


def _read_tcp_pose(cps, tcp_name, box_id=0, robot_id=0):
    if not tcp_name:
        return None
    result = []
    ret = cps.HRIF_ReadTCPByName(box_id, robot_id, tcp_name, result)
    if ret != 0 or len(result) < 6:
        print(f"[WayPoint2] Failed to read TCP '{tcp_name}' (ret={ret}).")
        return None
    try:
        return [float(v) for v in list(result)[:6]]
    except Exception:
        return None


def _convert_base_to_ucs(cps, base_pose, tcp_pose, ucs_pose, box_id=0):
    if not base_pose or not tcp_pose or not ucs_pose:
        return None
    out = []
    ret = cps.HRIF_Base2UcsTcp(box_id, base_pose, tcp_pose, ucs_pose, out)
    if ret != 0 or len(out) < 6:
        print(f"[WayPoint2] Base2UcsTcp failed ret={ret}")
        return None
    try:
        return [float(v) for v in list(out)[:6]]
    except Exception:
        return None


def _override_orientation(poses, rxyz):
    if not rxyz:
        return poses
    rx, ry, rz = rxyz
    return [[p[0], p[1], p[2], rx, ry, rz] for p in poses]


def _downsample_poses(poses, max_count):
    if not poses or max_count is None or max_count <= 0 or len(poses) <= max_count:
        return poses
    stride = max(2, int(math.ceil(len(poses) / float(max_count))))
    down = poses[::stride]
    if down[-1] != poses[-1]:
        down.append(poses[-1])
    return down


def _build_global_spiral_poses(zigzag_points, radius, angle_step_deg):
    """
    Build one continuous inward ellipse-spiral for the full pocket envelope.
    Keeps the old per-segment method available for fallback/reference.
    """
    if not zigzag_points:
        return []
    if len(zigzag_points[0]) < 6:
        return []

    xs = [float(p[0]) for p in zigzag_points]
    ys = [float(p[1]) for p in zigzag_points]
    z = float(zigzag_points[0][2])
    rx, ry, rz = [float(v) for v in zigzag_points[0][3:6]]

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    cx = (xmin + xmax) * 0.5
    cy = (ymin + ymax) * 0.5

    margin = max(0.0, float(WAYPOINT2_GLOBAL_MARGIN_MM))
    a0 = max(
        float(WAYPOINT2_GLOBAL_MIN_AXIS_MM),
        ((xmax - xmin) * 0.5) - margin,
    )
    b0 = max(
        float(WAYPOINT2_GLOBAL_MIN_AXIS_MM),
        ((ymax - ymin) * 0.5) - margin,
    )

    start = zigzag_points[0]
    dx = float(start[0]) - cx
    dy = float(start[1]) - cy
    nx = dx / max(a0, 1e-6)
    ny = dy / max(b0, 1e-6)
    theta0 = math.atan2(ny, nx)

    pitch_mm = max(4.0, float(radius) * 0.8)
    max_axis = max(a0, b0)
    turns = max(1.0, max_axis / max(pitch_mm, 1e-6))
    step_deg = max(3.0, float(angle_step_deg))
    total_steps = max(
        int(float(WAYPOINT2_GLOBAL_MIN_POINTS)),
        int(math.ceil((turns * 360.0) / step_deg)),
    )

    poses = [[float(start[0]), float(start[1]), z, rx, ry, rz]]
    for k in range(1, total_steps + 1):
        t = k / float(total_steps)
        scale = max(0.0, 1.0 - t)
        theta = theta0 + (2.0 * math.pi * turns * t)
        x = cx + (a0 * scale) * math.cos(theta)
        y = cy + (b0 * scale) * math.sin(theta)
        poses.append([x, y, z, rx, ry, rz])

    # Arc chain in run_waypoint2_path expects an even pose count (with lead-in).
    if len(poses) >= 4 and (len(poses) % 2) != 0:
        poses.pop()

    return poses


def wait_for_motion_done(cps, timeout_s=WAYPOINT2_WAIT_TIMEOUT_S):
    start = time.time()
    while True:
        result = []
        ret = cps.HRIF_IsMotionDone(0, 0, result)
        done = False
        if ret == 0 and result:
            last = result[-1]
            if isinstance(last, bool):
                done = last
            else:
                done = str(last).strip().lower() in ("1", "true", "ok")
        if done:
            return True
        if time.time() - start >= float(timeout_s):
            print(f"[WayPoint2] Motion wait timed out after {timeout_s:.1f}s.")
            return False
        time.sleep(0.05)


def run_waypoint2_path(
    cps: CPSClient,
    config: dict,
    poses,
    *,
    velocity: float,
    accel: float,
    radius: float,
    use_arc: bool = False,
    box_id: int = 0,
    robot_id: int = 0,
    tcp: str = None,
    ucs: str = None,
    cmd_prefix: str = "wp2",
):
    if not poses or len(poses) < 2:
        return True

    tcp_name = tcp or config["coords"].get("tcptool1plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")
    # Use current joint pose as IK seed if available; fallback to zeros.
    acs_pos = [0, 0, 0, 0, 0, 0]
    try:
        joint_seed = []
        jret = cps.HRIF_ReadActPos(box_id, robot_id, joint_seed)
        if jret == 0 and isinstance(joint_seed, (list, tuple)) and len(joint_seed) >= 6:
            acs_pos = [float(v) for v in joint_seed[:6]]
    except Exception:
        pass
    is_joint = 0
    is_seek = 0
    bit = 0
    state = 0

    cmd_index = 0
    idx = 0
    last_pos = poses[0]

    # Arc mode may fail if current live orientation (after force contact) does not
    # match the intended arc orientation. A lead-in WayPoint2 type=1 aligns the
    # controller state before the first type=2 segment.
    if use_arc and WAYPOINT2_ARC_LEADIN_TYPE1 and len(poses) >= 3:
        lead_end = poses[1]
        cmd_id = str((cmd_index % 3) + 1)
        seg_len = _distance_xyz(last_pos, lead_end)
        local_radius = 0.0 if seg_len <= 0.8 else min(radius, seg_len * 0.4)
        ret = cps.HRIF_WayPoint2(
            box_id,
            robot_id,
            1,
            lead_end,
            lead_end,
            acs_pos,
            tcp_name,
            ucs_name,
            velocity,
            accel,
            local_radius,
            is_joint,
            is_seek,
            bit,
            state,
            cmd_id,
        )
        print(f"[WayPoint2] ret={ret} type=1 cmd={cmd_prefix}-lead")
        if ret != 0:
            print(f"[WayPoint2] Lead-in move failed ret={ret} cmd={cmd_prefix}-lead")
            return False
        cmd_index += 1
        idx = 1
        last_pos = lead_end
        if WAYPOINT2_CMD_DELAY_S > 0:
            time.sleep(WAYPOINT2_CMD_DELAY_S)

    while idx + 2 < len(poses):
        p0 = poses[idx]
        p1 = poses[idx + 1]
        p2 = poses[idx + 2]

        use_arc_seg = use_arc and _arc_is_valid(p0, p1, p2)
        if use_arc and not use_arc_seg:
            print(f"[WayPoint2] Invalid arc geometry at cmd={cmd_prefix}-{cmd_index}")
            return False
        if use_arc_seg:
            move_type = 2
            end_pos = p2
            aux_pos = p1
            idx += 2
        else:
            move_type = 1
            end_pos = p1
            aux_pos = p1
            idx += 1

        cmd_id = str((cmd_index % 3) + 1)
        seg_len = _distance_xyz(last_pos, end_pos)
        local_radius = radius
        if seg_len <= 0.8:
            local_radius = 0.0
        else:
            local_radius = min(radius, seg_len * 0.4)

        ret = cps.HRIF_WayPoint2(
            box_id,
            robot_id,
            move_type,
            end_pos,
            aux_pos,
            acs_pos,
            tcp_name,
            ucs_name,
            velocity,
            accel,
            local_radius,
            is_joint,
            is_seek,
            bit,
            state,
            cmd_id,
        )
        print(f"[WayPoint2] ret={ret} type={move_type} cmd={cmd_prefix}-{cmd_index}")
        if ret != 0:
            print(f"[WayPoint2] Move failed ret={ret} cmd={cmd_prefix}-{cmd_index}")
            return False
        cmd_index += 1
        last_pos = end_pos
        if WAYPOINT2_CMD_DELAY_S > 0:
            time.sleep(WAYPOINT2_CMD_DELAY_S)
        if WAYPOINT2_BATCH_SIZE and cmd_index % int(WAYPOINT2_BATCH_SIZE) == 0:
            if WAYPOINT2_BATCH_PAUSE_S > 0:
                time.sleep(WAYPOINT2_BATCH_PAUSE_S)

    if idx + 1 < len(poses):
        if use_arc:
            print(f"[WayPoint2] Arc chain ended mid-segment at cmd={cmd_prefix}-{cmd_index}")
            return False
        end_pos = poses[idx + 1]
        cmd_id = str((cmd_index % 3) + 1)
        seg_len = _distance_xyz(last_pos, end_pos)
        local_radius = radius
        if seg_len <= 0.8:
            local_radius = 0.0
        else:
            local_radius = min(radius, seg_len * 0.4)

        ret = cps.HRIF_WayPoint2(
            box_id,
            robot_id,
            1,
            end_pos,
            end_pos,
            acs_pos,
            tcp_name,
            ucs_name,
            velocity,
            accel,
            local_radius,
            is_joint,
            is_seek,
            bit,
            state,
            cmd_id,
        )
        print(f"[WayPoint2] ret={ret} type=1 cmd={cmd_prefix}-tail")
        if ret != 0:
            print(f"[WayPoint2] Final linear move failed ret={ret} cmd={cmd_prefix}-tail")
            return False
        if WAYPOINT2_CMD_DELAY_S > 0:
            time.sleep(WAYPOINT2_CMD_DELAY_S)

    return True


def run_waypoint2_spiral_for_zigzag(
    cps: CPSClient,
    config: dict,
    zigzag_points,
    *,
    radius: float,
    angle_step_deg: float,
    orientation: str,
    velocity: float,
    accel: float,
    blend_radius: float = WAYPOINT2_BLEND_RADIUS,
    force: Optional[float] = None,
    force_settle_s: float = WAYPOINT2_FORCE_SETTLE_S,
    box_id: int = 0,
    robot_id: int = 0,
):
    if not zigzag_points:
        return True

    if WAYPOINT2_USE_GLOBAL_SPIRAL:
        spiral_poses = _build_global_spiral_poses(
            zigzag_points=zigzag_points,
            radius=radius,
            angle_step_deg=angle_step_deg,
        )
        if WAYPOINT2_DEBUG_ARC and spiral_poses:
            xs = [p[0] for p in spiral_poses]
            ys = [p[1] for p in spiral_poses]
            print(
                "[WayPoint2] Global spiral mode: "
                f"points={len(spiral_poses)} x=[{min(xs):.3f},{max(xs):.3f}] "
                f"y=[{min(ys):.3f},{max(ys):.3f}]"
            )
    else:
        # Legacy/reference mode: build a local spiral between each zigzag segment.
        spiral_poses = []
        for index, _ in enumerate(zigzag_points):
            if index + 1 >= len(zigzag_points):
                break
            point_a = zigzag_points[index]
            point_b = zigzag_points[index + 1]
            micro_arc_deg = max(1.0, float(WAYPOINT2_MICRO_ARC_DEG))
            half_step_deg = micro_arc_deg / 2.0
            flat_points = generate_spiral_between_points(
                start_pose=point_a,
                end_pose=point_b,
                radius=radius,
                angle_step_deg=half_step_deg,
                orientation=orientation,
            )
            poses = _poses_from_flat_points(flat_points)
            if poses:
                if spiral_poses:
                    spiral_poses.extend(poses[1:])
                else:
                    spiral_poses.extend(poses)

    if not spiral_poses:
        print("[WayPoint2] No spiral poses generated; skipping WayPoint2 path.")
        return False

    before = len(spiral_poses)
    spiral_poses = _downsample_poses(spiral_poses, WAYPOINT2_MAX_POSES)
    after = len(spiral_poses)
    if after != before:
        print(f"[WayPoint2] Downsampled spiral poses: {before} -> {after}")
    print(f"[WayPoint2] Total spiral poses: {len(spiral_poses)}")

    if force is not None and config is not None:
        putForceZminus(
            cps=cps,
            force=force,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            config=config,
        )
        time.sleep(force_settle_s)
        if WAYPOINT2_START_DELAY_S > 0:
            time.sleep(WAYPOINT2_START_DELAY_S)

    # Capture preferred UCS orientation for diagnostics/fallback only.
    preferred_ucs_rxyz = None
    ucs_name = config["coords"].get("ucsTable1") if config else None
    if WAYPOINT2_USE_UCS_ORI and ucs_name:
        ucs_pose = _read_ucs_pose(cps, ucs_name, box_id=box_id, robot_id=robot_id)
        if ucs_pose:
            preferred_ucs_rxyz = list(ucs_pose[3:6])
            if WAYPOINT2_DEBUG_ARC:
                print(f"[WayPoint2] UCS orientation: {preferred_ucs_rxyz} (ucs={ucs_name})")

    tcp_name = config["coords"].get("tcptool1plane1") if config else None
    ucs_name = config["coords"].get("ucsTable1") if config else None
    _ = _read_tcp_pose(cps, tcp_name, box_id=box_id, robot_id=robot_id)
    _ = _read_ucs_pose(cps, ucs_name, box_id=box_id, robot_id=robot_id)
    act_pose_base = _read_current_tcp_pose(cps, box_id=box_id, robot_id=robot_id)
    act_pose_ucs = None
    if WAYPOINT2_DEBUG_ARC:
        print("[WayPoint2] Base2UcsTcp disabled; assuming points are already in UCS.")

    points_in_base = False
    if WAYPOINT2_DETECT_FRAME and act_pose_base and zigzag_points:
        dist_base = _distance_xyz(act_pose_base, zigzag_points[0])
        if WAYPOINT2_DEBUG_ARC:
            print(
                f"[WayPoint2] Frame check dist_base={dist_base:.3f} "
                "points_in_base=UNKNOWN"
            )

    if points_in_base and WAYPOINT2_REQUIRE_PLANE1_FRAME:
        print(
            "[WayPoint2] Points appear to be in Base while Plane_1 is required. "
            "Abort to avoid unsafe frame mismatch."
        )
        return False

    # Use Plane_1 end-to-end when required.
    path_ucs_name = ucs_name
    if WAYPOINT2_DEBUG_ARC:
        print(f"[WayPoint2] Path UCS: {path_ucs_name}")

    act_pose = act_pose_base if points_in_base else None
    if act_pose and spiral_poses:
        start_gap = _distance_xyz(act_pose, spiral_poses[0])
        if start_gap > float(WAYPOINT2_MAX_START_GAP_MM):
            print(
                f"[WayPoint2] Unsafe start gap {start_gap:.1f}mm "
                f"(limit={WAYPOINT2_MAX_START_GAP_MM:.1f}mm); aborting."
            )
            return False
    if act_pose and WAYPOINT2_SNAP_TO_ACTUAL:
        dx = act_pose[0] - spiral_poses[0][0]
        dy = act_pose[1] - spiral_poses[0][1]
        dz = act_pose[2] - spiral_poses[0][2]
        sx = dx if WAYPOINT2_SNAP_XY else 0.0
        sy = dy if WAYPOINT2_SNAP_XY else 0.0
        sz = dz if WAYPOINT2_SNAP_Z else 0.0
        if abs(sx) > 1e-4 or abs(sy) > 1e-4 or abs(sz) > 1e-4:
            spiral_poses = [
                [p[0] + sx, p[1] + sy, p[2] + sz, p[3], p[4], p[5]]
                for p in spiral_poses
            ]
        if WAYPOINT2_DEBUG_ARC:
            print(
                f"[WayPoint2] Start offset raw(dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}) "
                f"applied(dx={sx:.4f}, dy={sy:.4f}, dz={sz:.4f})"
            )
    # Lock orientation using controller-consistent frame after conversion/snap.
    # If points were converted from Base->UCS, using pose0 orientation avoids
    # safety-space failures from forcing an incompatible Euler representation.
    rxyz = None
    if spiral_poses and len(spiral_poses[0]) >= 6:
        rxyz = list(spiral_poses[0][3:6])
    if (not points_in_base) and preferred_ucs_rxyz is not None:
        rxyz = list(preferred_ucs_rxyz)
    if rxyz is None and zigzag_points and len(zigzag_points[0]) >= 6:
        rxyz = list(zigzag_points[0][3:6])
    if rxyz is None:
        rxyz = [0.0, 0.0, 0.0]
    if WAYPOINT2_FORCE_UCS_TILT:
        rxyz[0] = float(WAYPOINT2_UCS_RX)
        rxyz[1] = float(WAYPOINT2_UCS_RY)
    spiral_poses = _override_orientation(spiral_poses, rxyz)
    if act_pose and rxyz:
        if any(abs(act_pose[i + 3] - rxyz[i]) > WAYPOINT2_ORI_TOL for i in range(3)):
            print(
                "[WayPoint2] Warning: Live orientation differs from arc orientation; "
                "arc may fail under force control."
            )
    if WAYPOINT2_DEBUG_ARC:
        print(f"[WayPoint2] Arc orientation: {rxyz}")
        if len(spiral_poses) >= 3:
            print(f"[WayPoint2] Arc p0={spiral_poses[0]}")
            print(f"[WayPoint2] Arc p1={spiral_poses[1]}")
            print(f"[WayPoint2] Arc p2={spiral_poses[2]}")

    turn_vibration_on(cps)
    ok = run_waypoint2_path(
        cps=cps,
        config=config,
        poses=spiral_poses,
        velocity=velocity,
        accel=accel,
        radius=blend_radius,
        use_arc=True,
        ucs=path_ucs_name,
        box_id=box_id,
        robot_id=robot_id,
    )
    wait_for_motion_done(cps, timeout_s=WAYPOINT2_WAIT_TIMEOUT_S)
    turn_vibration_off(cps)
    if force is not None and config is not None:
        releaseForce(cps=cps, config=config)

    return ok


def generate_zigzag_path(
    x_coords,
    y_coords,
    z_coords,
    innerOffset,
    innerOffsetX,
    orientation="horizontal",
    movement="zigzag",
    innerSandingOffset=50,
    edge_coverage=False,
    preserve_frame_sign=False,
):
    """
    Generate a zigzag/spiral path for sanding.

    Args:
        edge_coverage: If True, prepend a rectangular edge coverage path
                       (P1→P2→P3→P4→P1) before the zigzag pattern. The zigzag
                       then starts immediately from P1 after edge coverage completes.
    """
    prepoint = None
    zigzag_coords = []

    # Parameters (adjust as needed)
    tool3y = 50.8  # Tool offset in Y
    tool3x = 38.1  # Tool offset in X
    xframe_1 = 0
    xframe_2 = 0

    # 1) Collect boundary coordinates as [x, y, z]
    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    # Close the loop by duplicating the first point at the end
    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])  # copy for safety

    # 2) Compute the zigzag path (offset corners + zigzag)
    zigzag_coords = []

    # Ensure we have valid coordinates
    if x_coords and y_coords and z_coords:
        # We'll assume the pocket's Z-level is the same as the first boundary point
        z_zigzag = boundary_coords[0][2]

        # For Pocket4, corners (P13, P14, P15, P16):
        modified_Point2 = [
            (x_coords[1]) / 1 + tool3x + innerOffsetX,
            y_coords[1] - tool3y - (innerOffset),
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            (x_coords[0]) / 1 + tool3x + innerOffsetX,
            y_coords[0] + tool3y + innerOffset,
        ]
        modified_Point4 = [
            x_coords[3] - tool3x - innerOffset,
            y_coords[3] + tool3y + innerOffset,
        ]
        print("modified_Point1:", modified_Point1)
        print("modified_Point2:", modified_Point2)
        print("modified_Point3:", modified_Point3)
        print("modified_Point4:", modified_Point4)

        # Bounding box for alternate orientations
        x_min = min(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        x_max = max(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        y_min = min(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )
        y_max = max(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )

        # Calculate available horizontal dimension
        xinner = abs(x_max - x_min)
        print("xinner=", xinner)

        orientation_mode = (orientation or "vertical").lower()
        movement_mode = (movement or "zigzag").lower()

        # Modified Point Layout:
        # modified_Point1 = Bottom-right
        # modified_Point2 = Top-right
        # modified_Point3 = Top-left
        # modified_Point4 = Bottom-left

        rx_sanding = -0.034
        ry_sanding = 0.556
        rz_sanding = 0.251

        # Generate edge coverage path if enabled (rectangular path around the boundary)
        # The edge coverage must END at the point where zigzag/spiral will START
        # Edge coverage uses ORIGINAL boundary points (without innerOffsetX/innerOffset applied)
        edge_coverage_coords = []
        if edge_coverage:
            # Remove the offsets from modified points to get original boundary positions
            # Modified points already have tool3x, tool3y, innerOffsetX, and innerOffset applied
            # For edge coverage, we only want tool3x and tool3y, not the inner offsets
            edge_Point1 = [
                x_coords[0] + tool3x + 1.0,
                y_coords[0] + tool3y + 1.5,
                z_zigzag,
            ]
            edge_Point2 = [
                x_coords[1] + tool3x + 1.0,
                y_coords[1] - tool3y - 1.0,
                z_zigzag,
            ]
            edge_Point3 = [
                x_coords[2] - tool3x - 1.0,
                y_coords[2] - tool3y - 1.0,
                z_zigzag,
            ]
            edge_Point4 = [
                x_coords[3] - tool3x - 1.0,
                y_coords[3] + tool3y + 1.5,
                z_zigzag,
            ]

            # Edge coverage always uses the same path regardless of orientation
            # Edge coverage: P2 → P3 → P4 → P1 → P2 (starts at top-left like horizontal)
            if orientation_mode == "horizontal":
                # Horizontal zigzag starts at top-left (P2)
                # Edge coverage: P2 → P3 → P4 → P1 → P2 (ends at top-left)
                edge_coverage_coords = [
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # Start P2 (top-left)
                    [
                        edge_Point3[0],
                        edge_Point3[1],
                        edge_Point3[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P3 (top-right)
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P4 (bottom-right)
                    [
                        edge_Point1[0],
                        edge_Point1[1],
                        edge_Point1[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P1 (bottom-left)
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # End P2 (top-left) - zigzag start
                ]
                print(
                    "Edge coverage (horizontal): P2 → P3 → P4 → P1 → P2 (ends at top-left for zigzag start)"
                )
            else:
                # Vertical zigzag starts at bottom-right (P4)
                # Edge coverage: P4 → P1 → P2 → P3 → P4 (ends at bottom-right)
                edge_coverage_coords = [
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # Start P4 (bottom-right)
                    [
                        edge_Point1[0],
                        edge_Point1[1],
                        edge_Point1[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P1 (bottom-left)
                    [
                        edge_Point2[0],
                        edge_Point2[1],
                        edge_Point2[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P2 (top-left)
                    [
                        edge_Point3[0],
                        edge_Point3[1],
                        edge_Point3[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # P3 (top-right)
                    [
                        edge_Point4[0],
                        edge_Point4[1],
                        edge_Point4[2],
                        rx_sanding,
                        ry_sanding,
                        rz_sanding,
                    ],  # End P4 (bottom-right) - zigzag start
                ]
                print(
                    "Edge coverage (vertical): P4 → P1 → P2 → P3 → P4 (ends at bottom-right for zigzag start)"
                )

        if movement_mode == "rect":
            zigzag_coords = [
                [
                    modified_Point1[0],
                    modified_Point1[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point2[0],
                    modified_Point2[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point3[0],
                    modified_Point3[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point4[0],
                    modified_Point4[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
                [
                    modified_Point1[0],
                    modified_Point1[1],
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ],
            ]
        elif orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.floor(yinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1  # Prevent division by zero, ensure at least 1 row
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag rows from top (max y) to bottom (min y)
                # Starts at top-left (x_min, y_max)
                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [
                            x_min,
                            current_y,
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # Start at x_min (left side)
                        [
                            x_max,
                            current_y,
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # Go to x_max (right side)
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                # Fallback: single row when height is too small
                zigzag_coords.extend(
                    [
                        [x_min, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [x_max, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                )
        else:
            # Vertical orientation
            # In real robot, P1/P2 (X = -212.56) are physically RIGHT, P3/P4 (X = -55.1) are physically LEFT
            # Start from RIGHT side (P1/P2) and subtract offset to move toward LEFT (P3/P4)
            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1  # Prevent division by zero
                adjusted_step = xinner / num_steps
                print(
                    f"[Vertical] xinner={xinner}, num_steps={num_steps}, adjusted_step={adjusted_step}"
                )

                offset = 0.0
                toggle = 0

                # Build zigzag path from right (P1/P2) to left (P3/P4)
                # Subtract offset from P1/P2 X values to move toward more negative X
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    row_points = [
                        [
                            modified_Point4[0] - offset,
                            modified_Point4[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # P1 side (bottom-right, moving left)
                        [
                            modified_Point3[0] - offset,
                            modified_Point3[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],  # P2 side (top-right, moving left)
                    ]
                    # Reverse every other column to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                # Fallback: single column when width is too small
                zigzag_coords.extend(
                    [
                        [
                            modified_Point4[0],
                            modified_Point4[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],
                        [
                            modified_Point3[0],
                            modified_Point3[1],
                            z_zigzag,
                            rx_sanding,
                            ry_sanding,
                            rz_sanding,
                        ],
                    ]
                )

        if not preserve_frame_sign:
            # Legacy behavior: force positive coordinates.
            for point in edge_coverage_coords:
                point[1] = abs(point[1])
                point[0] = abs(point[0])
            for point in zigzag_coords:
                point[1] = abs(point[1])
                point[0] = abs(point[0])

        def _norm_xy(xv, yv):
            if preserve_frame_sign:
                return xv, yv
            return abs(xv), abs(yv)

        def _pre_x(xv):
            if preserve_frame_sign:
                return xv + (0.5 if xv >= 0 else -0.5)
            return abs(xv) + 0.5

        if movement_mode == "rect":
            prepoint = [
                _pre_x(zigzag_coords[0][0]),
                zigzag_coords[0][1],
                z_zigzag,
                rx_sanding,
                ry_sanding,
                rz_sanding,
            ]
        elif orientation_mode == "horizontal":
            # Horizontal: edge coverage starts at P2 (top-left), zigzag starts at top-left
            if edge_coverage:
                px, py = _norm_xy(modified_Point2[0], modified_Point2[1])
                prepoint = [
                    _pre_x(px),
                    py,
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
            else:
                px, py = _norm_xy(x_min, y_max)
                prepoint = [
                    _pre_x(px),
                    py,
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
        else:
            # Vertical: edge coverage starts at same position as horizontal (P2/top-left)
            if edge_coverage:
                px, py = _norm_xy(modified_Point4[0], modified_Point4[1])
                prepoint = [
                    _pre_x(px),
                    py,
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]
            else:
                px, py = _norm_xy(modified_Point4[0], modified_Point4[1])
                prepoint = [
                    _pre_x(px),
                    py,
                    z_zigzag,
                    rx_sanding,
                    ry_sanding,
                    rz_sanding,
                ]

        if edge_coverage:
            print(
                f"Edge coverage: {len(edge_coverage_coords)} points (MoveL), Zigzag: {len(zigzag_coords)} points (spiral)"
            )

    return edge_coverage_coords, zigzag_coords, prepoint


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    return config


def load_json_config():
    """Loads configuration from config.json."""
    with open("./configs/cycleData.json", "r") as file:
        config = json.load(file)
    return config


def get_pocket_size(door_number, default_on_error=True):
    """Return pocket size (xlen, ylen) using pocket corner points."""
    points = []
    for corner in range(4):
        point = get_pocket_point(
            door_number, corner, default_on_error=default_on_error
        )
        if not point or len(point) < 2:
            return {"xlen": "null", "ylen": "null"}
        try:
            x_val = float(point[0])
            y_val = float(point[1])
        except (TypeError, ValueError):
            return {"xlen": "null", "ylen": "null"}
        points.append((x_val, y_val))

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {"xlen": max(xs) - min(xs), "ylen": max(ys) - min(ys)}


def smalldoor1zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor1zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(1)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points

        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
            preserve_frame_sign=False,
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False,
            preserve_frame_sign=USE_WAYPOINT2_ARC,
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE:
                has_edge = False
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )
                print("Turned Vibration On")

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0

                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                if USE_WAYPOINT2_ARC:
                    ok = run_waypoint2_spiral_for_zigzag(
                        cps=cps,
                        config=config,
                        zigzag_points=zigzag_points,
                        radius=12.0,
                        angle_step_deg=45.0,
                        orientation=orientation,
                        velocity=150.0,
                        accel=300.0,
                        force=force,
                    )
                    if not ok:
                        raise RuntimeError("[WayPoint2] Spiral path failed for door 1.")
                    return
                # for index, _ in enumerate(zigzag_points):
                #     point_A = zigzag_points[index]
                #     if index + 1 >= len(zigzag_points):
                #         break
                #     point_B = zigzag_points[index + 1]

                #     print("Spiral move from A to B:", point_A, "->", point_B)
                #     success, count = run_spiral_between_points(
                #         cps=cps,
                #         config=config,
                #         start_pose=point_A,
                #         end_pose=point_B,
                #         radius=12.0,
                #         angle_step_deg=45.0,
                #         track_name=spiral_track_name,
                #         velocity=150.0,
                #         accel=300.0,
                #         jerk=3000.0,
                #         init_path=not path_initialized,
                #         orientation=orientation
                #     )
                #     if not success:
                #         push_failed = True
                #         break

                #     path_initialized = True
                #     total_count += count

                # if path_initialized and not push_failed:
                #     timeout = compute_timeout(
                #         total_points=total_count, velocity=300.0 * 10.0 / 45.0
                #     )
                #     # Keep a short anti-false-positive runtime guard without
                #     # adding visible delay at the final point.
                #     min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                #     finalized = finalize_spiral_path(
                #         cps,
                #         spiral_track_name,
                #         box_id=0,
                #         robot_id=0,
                #         completion_timeout=timeout,
                #         min_runtime_s=min_runtime_s,
                #         force= force,
                #         config= config
                #     )
                #     if not finalized:
                #         raise RuntimeError("[Spiral] finalize_spiral_path failed for door 1.")
                    
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )
        
        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if (not split) and not (USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE):
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=zigzag_pathp1,
                force=force,
                run_edge_coverage=not split
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(1, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor1zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor1zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor2zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor2zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(2)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
            preserve_frame_sign=False,
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False,
            preserve_frame_sign=USE_WAYPOINT2_ARC,
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE:
                has_edge = False
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                if USE_WAYPOINT2_ARC:
                    ok = run_waypoint2_spiral_for_zigzag(
                        cps=cps,
                        config=config,
                        zigzag_points=zigzag_points,
                        radius=12.0,
                        angle_step_deg=45.0,
                        orientation=orientation,
                        velocity=150.0,
                        accel=300.0,
                        force=force,
                    )
                    if not ok:
                        raise RuntimeError("[WayPoint2] Spiral path failed for door 2.")
                    return
                
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]

                    print("Spiral move from A to B:", point_A, "->", point_B)
                    success, count = run_spiral_between_points(
                        cps=cps,
                        config=config,
                        start_pose=point_A,
                        end_pose=point_B,
                        radius=12.0,
                        angle_step_deg=45.0,
                        track_name=spiral_track_name,
                        velocity=150.0,
                        accel=300.0,
                        jerk=3000.0,
                        init_path=not path_initialized,
                        orientation=orientation
                    )
                    if not success:
                        push_failed = True
                        break

                    path_initialized = True
                    total_count += count

                if path_initialized and not push_failed:
                    timeout = compute_timeout(
                        total_points=total_count, velocity=300.0 * 10.0 / 45.0
                    )
                    # Keep a short anti-false-positive runtime guard without
                    # adding visible delay at the final point.
                    min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                    finalized = finalize_spiral_path(
                        cps,
                        spiral_track_name,
                        box_id=0,
                        robot_id=0,
                        completion_timeout=timeout,
                        min_runtime_s=min_runtime_s,
                        force=force,
                        config=config
                    )
                    if not finalized:
                        raise RuntimeError("[Spiral] finalize_spiral_path failed for door 2.")

            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed = 1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed= 1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if (not split) and not (USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE):
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=zigzag_pathp1,
                force=force,
                run_edge_coverage=not split
            )
            # Edge coverage + zigzag path
            # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(2, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor2zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor2zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor3zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor3zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(3)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
            preserve_frame_sign=False,
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False,
            preserve_frame_sign=USE_WAYPOINT2_ARC,
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE:
                has_edge = False
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                if USE_WAYPOINT2_ARC:
                    ok = run_waypoint2_spiral_for_zigzag(
                        cps=cps,
                        config=config,
                        zigzag_points=zigzag_points,
                        radius=12.0,
                        angle_step_deg=45.0,
                        orientation=orientation,
                        velocity=150.0,
                        accel=300.0,
                        force=force,
                    )
                    if not ok:
                        raise RuntimeError("[WayPoint2] Spiral path failed for door 3.")
                    return
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]

                    print("Spiral move from A to B:", point_A, "->", point_B)
                    success, count = run_spiral_between_points(
                        cps=cps,
                        config=config,
                        start_pose=point_A,
                        end_pose=point_B,
                        radius=12.0,
                        angle_step_deg=45.0,
                        track_name=spiral_track_name,
                        velocity=150.0,
                        accel=300.0,
                        jerk=3000.0,
                        init_path=not path_initialized,
                        orientation=orientation
                    )
                    if not success:
                        push_failed = True
                        break

                    path_initialized = True
                    total_count += count

                if path_initialized and not push_failed:
                    timeout = compute_timeout(
                        total_points=total_count, velocity=300.0 * 10.0 / 45.0
                    )
                    # Keep a short anti-false-positive runtime guard without
                    # adding visible delay at the final point.
                    min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                    finalized = finalize_spiral_path(
                        cps,
                        spiral_track_name,
                        box_id=0,
                        robot_id=0,
                        completion_timeout=timeout,
                        min_runtime_s=min_runtime_s,
                        force = force,
                        config= config
                    )
                    if not finalized:
                        raise RuntimeError("[Spiral] finalize_spiral_path failed for door 3.")

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)

        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if (not split) and not (USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE):
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=zigzag_pathp1,
                force=force,
                run_edge_coverage=not split
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(3, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor3zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor3zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


def smalldoor4zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor4zizagsmall(force, z, cps, split=False):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(4)
        print("x1:", x1)
        tcx0 = x1
        tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
        seventh_positions = [tcx0, tcx1] if split else [x1]
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        x_coords_path = x_coords1
        if split:
            x_coords_path = [coord / 2 for coord in x_coords1]

        edge_coverage_pathp1, _, edge_prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
            preserve_frame_sign=False,
        )
        _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=False,
            preserve_frame_sign=USE_WAYPOINT2_ARC,
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(
            cps, config, edge_points, zigzag_points, force, run_edge_coverage=True
        ):
            has_edge = run_edge_coverage and edge_points and len(edge_points) > 0
            if USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE:
                has_edge = False
            if has_edge:
                force_seek_linear = 5.0
                force_blending_timeout = 7.0
                if split:
                    force_blending_timeout = 0.4
                putForceZminus(
                    cps=cps,
                    force=force,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    config=config,
                    search_linear_velocity=force_seek_linear,
                    blending_timeout_s=force_blending_timeout,
                )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if has_edge:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                edge_speed = float(json_config["sandingSpeed"]) * 0.3
                for point in edge_points:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=edge_speed,
                        speed_mode="linear",
                        wait=False
                    )
                    if point == edge_points[-1]:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=edge_speed,
                            speed_mode="linear",
                            wait=True
                        )
                    turn_vibration_on(cps)
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
                releaseForce(cps=cps, config=config)
                turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            if zigzag_points and len(zigzag_points) > 0:
                spiral_track_name = f"small_{uuid.uuid4().hex[:6]}"
                path_initialized = False
                push_failed = False
                total_count = 0
                
                # Move to first zigzag point to ensure proper transition from edge coverage
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True
                )
                if USE_WAYPOINT2_ARC:
                    ok = run_waypoint2_spiral_for_zigzag(
                        cps=cps,
                        config=config,
                        zigzag_points=zigzag_points,
                        radius=12.0,
                        angle_step_deg=45.0,
                        orientation=orientation,
                        velocity=150.0,
                        accel=300.0,
                        force=force,
                    )
                    if not ok:
                        raise RuntimeError("[WayPoint2] Spiral path failed for door 4.")
                    return
                for index, _ in enumerate(zigzag_points):
                    point_A = zigzag_points[index]
                    if index + 1 >= len(zigzag_points):
                        break
                    point_B = zigzag_points[index + 1]

                    print("Spiral move from A to B:", point_A, "->", point_B)
                    success, count = run_spiral_between_points(
                        cps=cps,
                        config=config,
                        start_pose=point_A,
                        end_pose=point_B,
                        radius=12.0,
                        angle_step_deg=45.0,
                        track_name=spiral_track_name,
                        velocity=150.0,
                        accel=300.0,
                        jerk=3000.0,
                        init_path=not path_initialized,
                        orientation=orientation
                    )
                    if not success:
                        push_failed = True
                        break

                    path_initialized = True
                    total_count += count

                if path_initialized and not push_failed:
                    timeout = compute_timeout(
                        total_points=total_count, velocity=300.0 * 10.0 / 45.0
                    )
                    # Keep a short anti-false-positive runtime guard without
                    # adding visible delay at the final point.
                    min_runtime_s = max(0.15, min(0.6, timeout * 0.05))
                    finalized = finalize_spiral_path(
                        cps,
                        spiral_track_name,
                        box_id=0,
                        robot_id=0,
                        completion_timeout=timeout,
                        min_runtime_s=min_runtime_s,
                        force=force,
                        config=config
                    )
                    if not finalized:
                        raise RuntimeError("[Spiral] finalize_spiral_path failed for door 4.")

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            #turn_vibration_off(cps)
            # Release force
            #releaseForce(cps=cps, config=config)
        
        if split and edge_coverage_pathp1:
            edge_start = edge_coverage_pathp1[0]
        else:
            edge_start = prepointp1

        if split:
            communicate(
                cps=cps,
                config=config,
                seventh=tcx0,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True
            )
            if edge_prepointp1:
                communicate(
                    cps=cps,
                    config=config,
                    point=edge_prepointp1,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True
                )
            communicate(
                cps=cps,
                config=config,
                point=edge_start,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=1.0,
                wait=True
            )
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1,
                zigzag_points=[],
                force=force,
                run_edge_coverage=True
            )

        for idx, seventh_pos in enumerate(seventh_positions):
            first_split = split and idx == 0
            if not first_split:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_pos,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.8,
                    wait=True
                )
                if (not split) and not (USE_WAYPOINT2_ARC and WAYPOINT2_SKIP_EDGE_COVERAGE):
                    communicate(
                        cps=cps,
                        config=config,
                        point=prepointp1,  # Dynamic prepoint
                        tcp=config["coords"]["tcptool1plane1"],
                        ucs=config["coords"]["ucsTable1"],
                        seventh=-1,
                        speed=1.0,
                        wait=True
                    )
            # # turn_vibration_on(cps)
            perform_process_top(
                cps,
                config,
                edge_points=edge_coverage_pathp1 if not split else [],
                zigzag_points=zigzag_pathp1,
                force=force,
                run_edge_coverage=not split,
            )  # Edge coverage + zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.8,
                wait=True
            )

    pocket_size = get_pocket_size(4, default_on_error=True)
    pocket_y = pocket_size["ylen"]
    pocket_x = pocket_size["xlen"]

    print("pocket_y:", pocket_y)
    print("pocket_x:", pocket_x)

    # Check conditions
    if pocket_y == "null" or pocket_x == "null":
        print("No door data available - skipping operations")
    elif isinstance(pocket_y, (int, float)) and isinstance(pocket_x, (int, float)):
        smalldoor4zizagsmall(force, z, cps, split=pocket_y > 600 and pocket_x > 290)
    elif isinstance(pocket_y, (int, float)) or isinstance(pocket_x, (int, float)):
        smalldoor4zizagsmall(force, z, cps, split=False)
    else:
        print(
            f"Invalid pocket size type: {type(pocket_x)}, {type(pocket_y)} - expected numbers or 'null'"
        )


if __name__ == "__main__":
    smalldoor1zizag(force=5, z=-6.5)
    smalldoor2zizag(force=5, z=-6.5)
    smalldoor3zizag(force=5, z=-6.5)
    # smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
