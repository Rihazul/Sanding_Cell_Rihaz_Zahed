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
from cycle_data_utils import get_inverse_overlap_step
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


DEFAULT_ORIENTATION_TOL = 1.0
# Optional forced orientation (Rx, Ry, Rz in degrees) to keep tool normal
# perpendicular to the sanding plane. When None, we lock to the actual
# orientation measured under force control.
FORCE_SPIRAL_ORIENT = None


def _read_act_coord_pose(cps, tcp: str, ucs: str) -> Optional[list]:
    result = []
    ret = cps.HRIF_ReadActCoord(0, 0, tcp, ucs, result)
    if ret == 0 and len(result) >= 6:
        try:
            return [float(v) for v in result[:6]]
        except (TypeError, ValueError):
            pass
    result = []
    ret = cps.HRIF_ReadActPos(0, 0, result)
    if ret != 0 or len(result) < 12:
        return None
    try:
        return [float(v) for v in result[6:12]]
    except (TypeError, ValueError):
        return None


def _capture_locked_orient(
    cps,
    config,
    *,
    tcp: str,
    ucs: str,
    fallback_orient,
    settle_s: float = 0.05,
):
    time.sleep(max(0.0, float(settle_s)))
    actual = _read_act_coord_pose(cps, tcp, ucs)
    logger = config.get("logger") if config else None
    if actual and len(actual) >= 6:
        orient = list(actual[3:6])
        msg = f"[Orientation] locked to actual under force: {orient}"
        if logger:
            logger.info(msg)
        else:
            print(msg)
        return orient
    msg = f"[Orientation] failed to read actual; using fallback {fallback_orient}"
    if logger:
        logger.warning(msg)
    else:
        print(msg)
    return list(fallback_orient)

DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0
DEFAULT_MOVEJS_JERK_RATIO = 100
DEFAULT_MOVEJS_TRANSITION_DEG = 25
MOVEJS_MAX_POINTS = 50
TOOL4_DIAMETER_MM = 135.0
TOOL4_RADIUS_MM = (TOOL4_DIAMETER_MM +11) / 2.0 
POCKET_MAX_OVERLAP_MM = 100.0



def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _calculate_zigzag_pass_spacing(span_mm: float, requested_step_mm: float):
    """
    Return the interval count and evenly distributed step for a pocket span.

    Using ceil ensures the actual step never exceeds the requested step, so
    increasing UI overlap always increases or preserves the number of passes.
    """
    span = max(0.0, float(span_mm))
    requested_step = max(1.0, float(requested_step_mm))
    intervals = max(1, math.ceil(span / requested_step))
    return intervals, span / intervals


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


def _resolve_sanding_blend_timeout(config, fallback=12.0):
    """
    Sanding zigzag paths can exceed transition/blending defaults.
    Use a longer timeout so force is not released mid-path.
    """
    try:
        door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        value = door_cfg.get(
            "sandingBlendTimeoutSeconds",
            door_cfg.get("blendTimeoutSeconds", fallback),
        )
        return max(2.0, float(value))
    except Exception:
        return float(fallback)


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
    tcp_name = tcp or config["coords"].get("tcptool3plane1")
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
                tcp=config["coords"]["tcptool3plane1"],
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


def generate_zigzag_path(
    x_coords,
    y_coords,
    z_coords,
    innerOffset,
    innerOffsetX,
    orientation="horizontal",
    movement="zigzag",
    innerSandingOffset=67.5,
):
    """Generate zigzag/rect path for sanding pocket area."""
    prepoint = None
    zigzag_coords = []
    # Circular Tool 4: use one radius only; ignore rectangular inner offsets.
    del innerOffset, innerOffsetX
    tool_radius = TOOL4_RADIUS_MM

    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])
    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])

    if x_coords and y_coords and z_coords:
        z_zigzag = boundary_coords[0][2]
        z_zigzag = z_zigzag + 7  # Lift above the surface for zigzag path

        modified_Point2 = [
            x_coords[1] + tool_radius,
            y_coords[1] - tool_radius,
        ]
        modified_Point3 = [
            x_coords[2] - tool_radius,
            y_coords[2] - tool_radius,
        ]
        modified_Point1 = [
            x_coords[0] + tool_radius,
            y_coords[0] + tool_radius,
        ]
        modified_Point4 = [
            x_coords[3] - tool_radius,
            y_coords[3] + tool_radius,
        ]
        print("modified_Point1:", modified_Point1)
        print("modified_Point2:", modified_Point2)
        print("modified_Point3:", modified_Point3)
        print("modified_Point4:", modified_Point4)

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

        xinner = abs(x_max - x_min)
        print("xinner=", xinner)
        step_mm = max(1.0, float(innerSandingOffset))

        orientation_mode = (orientation or "vertical").lower()
        movement_mode = (movement or "zigzag").lower()

        rx_sanding = -0.034
        ry_sanding = 0.556
        rz_sanding = 0.251

        if movement_mode == "rect":
            zigzag_coords = [
                [modified_Point1[0], modified_Point1[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                [modified_Point2[0], modified_Point2[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                [modified_Point3[0], modified_Point3[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                [modified_Point4[0], modified_Point4[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                [modified_Point1[0], modified_Point1[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
            ]
        elif orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps, adjusted_step = _calculate_zigzag_pass_spacing(
                    yinner, step_mm
                )
                print(
                    f"[Horizontal] yinner={yinner}, requested_step={step_mm}, "
                    f"num_steps={num_steps}, passes={num_steps + 1}, "
                    f"adjusted_step={adjusted_step}"
                )

                offset = 0.0
                toggle = 0

                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [x_min, current_y, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [x_max, current_y, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                zigzag_coords.extend(
                    [
                        [x_min, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [x_max, y_max, z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                )
        else:
            if xinner > 0:
                num_steps, adjusted_step = _calculate_zigzag_pass_spacing(
                    xinner, step_mm
                )
                print(
                    f"[Vertical] xinner={xinner}, requested_step={step_mm}, "
                    f"num_steps={num_steps}, passes={num_steps + 1}, "
                    f"adjusted_step={adjusted_step}"
                )

                offset = 0.0
                toggle = 0
                while offset <= xinner + 1e-9:
                    row_points = [
                        [modified_Point4[0] - offset, modified_Point4[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [modified_Point3[0] - offset, modified_Point3[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                zigzag_coords.extend(
                    [
                        [modified_Point4[0], modified_Point4[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                        [modified_Point3[0], modified_Point3[1], z_zigzag, rx_sanding, ry_sanding, rz_sanding],
                    ]
                )

        for point in zigzag_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        if movement_mode == "rect":
            prepoint = [
                abs(zigzag_coords[0][0]) + 0.5,
                zigzag_coords[0][1],
                z_zigzag,
                rx_sanding,
                ry_sanding,
                rz_sanding,
            ]
        elif orientation_mode == "horizontal":
            prepoint = [
                abs(x_min) + 0.5,
                y_max,
                z_zigzag,
                rx_sanding,
                ry_sanding,
                rz_sanding,
            ]
        else:
            prepoint = [
                abs(modified_Point4[0]) + 0.5,
                abs(modified_Point4[1]),
                z_zigzag,
                rx_sanding,
                ry_sanding,
                rz_sanding,
            ]

    return [], zigzag_coords, prepoint
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


def _resolve_inner_sanding_offset(cycle_cfg, default=67.5):
    """
    Convert UI overlap (mm) to sanding step (mm) safely.
    UI range is overlap=0..100; step must never be 0.
    """
    try:
        return float(
            get_inverse_overlap_step(
                cycle_cfg or {},
                key="inverseOverlapping",
                tool_diameter_mm=TOOL4_DIAMETER_MM,
                max_overlap_mm=POCKET_MAX_OVERLAP_MM,
                min_step_mm=1.0,
            )
        )
    except Exception:
        return max(1.0, float(default))


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


def _run_small_door_zigzag(
    door_num,
    force,
    z,
    cps,
    orientation="horizontal",
    movement="zigzag",
    spiral_settings=None,
    *,
    split=False,
):
    apply_spiral_settings(spiral_settings)

    config = load_config()
    json_config = load_json_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    p8 = get_inner_corner_point(door_num, 0)
    p7 = get_inner_corner_point(door_num, 1)
    p6 = get_inner_corner_point(door_num, 2)
    p5 = get_inner_corner_point(door_num, 3)
    print("p8:", p8)
    print("p7:", p7)
    print("p6:", p6)
    print("p5:", p5)

    x1 = p8[0] + get_door_position(door_num)
    print("x1:", x1)
    tcx0 = x1
    tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
    seventh_positions = [tcx0, tcx1] if split else [x1]

    prehoming = [0, 200, 50, 0, 0, 0]
    distance = p6[0] - p8[0]
    print("distance:", distance)

    point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
    point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
    point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
    point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
    print("point5:", point5)
    print("point6:", point6)
    print("point7:", point7)
    print("point8:", point8)

    point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
    point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
    point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
    point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
    print("point5u:", point5u)
    print("point6u:", point6u)
    print("point7u:", point7u)
    print("point8u:", point8u)

    x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
    y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
    z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]
    print("x_coords1 ", x_coords1)
    print("y_coords2 ", y_coords1)
    print("z_coords3", z_coords1)

    x_coords_path = [coord / 2 for coord in x_coords1] if split else x_coords1
    inner_sanding_offset = _resolve_inner_sanding_offset(json_config, default=67.5)
    _, zigzag_pathp1, prepointp1 = generate_zigzag_path(
        x_coords=x_coords_path,
        y_coords=y_coords1,
        z_coords=z_coords1,
        innerOffset=22,
        innerOffsetX=60,
        orientation=orientation,
        movement=movement,
        innerSandingOffset=inner_sanding_offset,
    )
    zigzag_pathp1_left = zigzag_pathp1
    zigzag_pathp1_right = zigzag_pathp1
    if split:
        _, zigzag_pathp1_left, _ = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=22,
            innerOffsetX=60,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=inner_sanding_offset,
        )
        _, zigzag_pathp1_right, _ = generate_zigzag_path(
            x_coords=x_coords_path,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=22,
            innerOffsetX=60,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=inner_sanding_offset,
        )
    print("zigzag_pathp=", zigzag_pathp1)
    print("prepointp:", prepointp1)

    def perform_process_top(zigzag_points):
        force_seek_linear = 5.0
        force_blending_timeout = 0.4 if split else 7.0

        if zigzag_points and len(zigzag_points) > 0:
            print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
            communicate(
                cps=cps,
                config=config,
                point=zigzag_points[0],
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=float(json_config["sandingSpeed"]),
                velocity_profile="sandingspeed",
                wait=True,
            )
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
                search_linear_velocity=force_seek_linear,
                blending_timeout_s=force_blending_timeout,
            )
            turn_vibration_on(cps)

            for index, point_A in enumerate(zigzag_points):
                if index + 1 >= len(zigzag_points):
                    break
                point_B = zigzag_points[index + 1]
                print("Linear move from A to B:", point_A, "->", point_B)
                is_last_segment = index == (len(zigzag_points) - 2)
                communicate(
                    cps=cps,
                    config=config,
                    point=point_B,
                    tcp=config["coords"]["tcptool4plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    velocity_profile="sandingspeed",
                    speed_mode="linear",
                    wait=is_last_segment,
                )

        waitForBlending(
            cps=cps,
            config=config,
            timeout_s=_resolve_sanding_blend_timeout(config),
        )
        turn_vibration_off(cps)
        releaseForce(cps=cps, config=config)

    for idx, seventh_pos in enumerate(seventh_positions):
        first_split = split and idx == 0
        current_zigzag = zigzag_pathp1
        if split:
            current_zigzag = zigzag_pathp1_left if idx == 0 else zigzag_pathp1_right
        if not first_split:
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_pos,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=0.8,
                wait=True,
                require_seventh_ok=True,
            )
            if not split:
                communicate(
                    cps=cps,
                    config=config,
                    point=prepointp1,
                    tcp=config["coords"]["tcptool4plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=1.0,
                    wait=True,
                )

        perform_process_top(current_zigzag)
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.8,
            wait=True,
        )


def smalldoor1zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    print("[zigzag] Door 1 single-pass mode enabled (split disabled).")
    return _run_small_door_zigzag(
        1,
        force,
        z,
        cps,
        orientation=orientation,
        movement=movement,
        spiral_settings=spiral_settings,
        split=False,
    )


def smalldoor2zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    print("[zigzag] Door 2 single-pass mode enabled (split disabled).")
    return _run_small_door_zigzag(
        2,
        force,
        z,
        cps,
        orientation=orientation,
        movement=movement,
        spiral_settings=spiral_settings,
        split=False,
    )


def smalldoor3zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    print("[zigzag] Door 3 single-pass mode enabled (split disabled).")
    return _run_small_door_zigzag(
        3,
        force,
        z,
        cps,
        orientation=orientation,
        movement=movement,
        spiral_settings=spiral_settings,
        split=False,
    )


def smalldoor4zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    print("[zigzag] Door 4 single-pass mode enabled (split disabled).")
    return _run_small_door_zigzag(
        4,
        force,
        z,
        cps,
        orientation=orientation,
        movement=movement,
        spiral_settings=spiral_settings,
        split=False,
    )


if __name__ == "__main__":
    smalldoor1zizag(force=5, z=-6.5)
    smalldoor2zizag(force=5, z=-6.5)
    smalldoor3zizag(force=5, z=-6.5)
    # smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag











