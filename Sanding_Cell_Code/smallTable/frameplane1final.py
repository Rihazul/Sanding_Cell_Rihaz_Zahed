# testcommu.py

import sys
import os
import time
import json
import math
from typing import Optional
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Components.RobotState import RobotState
import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
    get_x_values,
    get_y_values
)

def compute_timeout(
    *,
    total_points: Optional[int] = None,
    cap: float = 72.0,
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

    timeout = min(cap, max(1.0, float(time_factor)))
    print(
        f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
    )
    return timeout


DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0
SANDING_Z_THRESHOLD = 5.0
J7_IDLE_TIMEOUT_S = 45.0
J7_IDLE_POLL_S = 0.02


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wait_for_j7_idle(cps: CPSClient, timeout_s: float = J7_IDLE_TIMEOUT_S, poll_s: float = J7_IDLE_POLL_S) -> bool:
    start_time = time.time()
    result = []
    while True:
        result.clear()
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
        if (nret == 0 or nret is None) and len(result) > 2 and str(result[2]).strip() == "0":
            return True
        if (time.time() - start_time) >= float(timeout_s):
            print(f"[J7] Timeout waiting for idle after {timeout_s:.1f}s. Last state={result}")
            return False
        time.sleep(max(0.005, float(poll_s)))


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
    if y0 == y1:
        dist = abs(x1 - x0)
        turns = math.ceil(dist / (radius * 2))
    elif x0 == x1 :
        dist = abs(y1 - y0)
        turns = math.ceil(dist / (radius * 2))

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
):
    """Push a spiral MovePathL between two poses.

    By default (`finalize=False`), the path is only pushed (optionally initialized
    with `init_path`) and execution must be triggered separately via
    `finalize_spiral_path`. Set `finalize=True` to push and execute immediately.
    """
    track_name = track_name or "spiral_path"
    tcp_name = tcp or config["coords"].get("tcptool3plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")
    for label, value in (("track", track_name), ("tcp", tcp_name), ("ucs", ucs_name)):
        if not isinstance(value, str) or not value.strip():
            print(f"[Spiral] Invalid {label} name: {value!r}")
            return False, None
        if "," in value or ";" in value:
            print(f"[Spiral] {label} name contains invalid separator: {value!r}")
            return False, None
    print(f"[Spiral] Using track name: {track_name}")

    all_points = generate_spiral_between_points(
        start_pose=start_pose,
        end_pose=end_pose,
        turns=turns,
        radius=DEFAULT_SPIRAL_RADIUS if radius is None else radius,
        angle_step_deg=angle_step_deg,
        max_points=max_points,
    )
    cleaned_points = []
    for idx, v in enumerate(all_points):
        try:
            fv = float(v)
        except (TypeError, ValueError):
            print(f"[Spiral] Non-numeric point value at {idx}: {v!r}")
            return False, None
        if not math.isfinite(fv):
            print(f"[Spiral] Non-finite point value at {idx}: {fv!r}")
            return False, None
        cleaned_points.append(fv)
    all_points = cleaned_points
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
            1.00,
            2.0,
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
    vibration: bool = True,
    force: Optional[float] = None,
    force_func=None,
    config: Optional[dict] = None,
    tcp: Optional[str] = None,
    ucs: Optional[str] = None,
    completion_timeout=45.0,
    min_runtime_s: float = 0.0,
) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False

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
        time.sleep(0.02)
        print(f"Waiting for PATH_READY... t={elapsed:.1f}s state={st}\r", end="")

    pass  # vibration handled in finalize

    force_active = False
    if force is not None and force_func is not None:
        if config is None:
            print("[Spiral] Missing config for force control; skipping force.")
        else:
            tcp_name = tcp or config["coords"].get("tcptool3plane1")
            ucs_name = ucs or config["coords"].get("ucsTable1")
            ret = force_func(
                cps=cps,
                force=force,
                tcp=tcp_name,
                ucs=ucs_name,
                config=config,
            )
            print(f"Force control ret code: {ret}")
            force_active = True
    # Create a lightweight state helper for motion monitoring.
    # Reuse the existing config/cps to avoid reinitializing connections.
    robot_state = RobotState(config=config if config is not None else load_config(), cps_client=cps)

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

    if vibration:
        turn_vibration_on(cps)

    # Wait for completion (track path state + robot flags)
    ok = True
    start = time.time()
    estimate_timeout = float(completion_timeout)
    # Safety watchdog is only for abnormal hangs; estimate itself remains the target.
    safety_timeout = estimate_timeout + max(8.0, min(30.0, estimate_timeout * 0.3))
    timeout_notice_logged = False
    timeout_stall_start = None
    safety_extensions = 0
    # Tolerances tuned to avoid waiting on noisy force-hold jitter at the last point.
    position_noise_mm = 0.6
    planar_noise_mm = 0.35
    settle_window_s = 0.15
    near_end_ratio = 0.93
    motion_seen = motion_started
    motion_last_change = time.time()
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
                        abs(a - b) > planar_noise_mm
                        for a, b in zip(curr_cart[:2], pre_move_cart[:2])
                    )
                )
            ):
                motion_seen = True
            if last_cart and len(curr_cart) >= 6 and len(last_cart) >= 6:
                moved = (
                    any(abs(a - b) > planar_noise_mm for a, b in zip(curr_cart[:2], last_cart[:2]))
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
                # If we still see progress, extend the safety timeout instead of failing
                # mid-motion (which can cause the next cycle to collide and error 49622).
                if progress_recent and elapsed > safety_timeout:
                    safety_extensions += 1
                    safety_timeout = elapsed + max(3.0, min(12.0, estimate_timeout * 0.25))
                    if safety_extensions <= 3:
                        print(
                            f"[Spiral] Extending safety timeout to {safety_timeout:.1f}s "
                            "because motion progress is still active."
                        )
                    time.sleep(0.02)
                    continue
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

    return ok


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config

def _run_linear_sanding_process(cps, config, points, force, sanding_speed, *, tcp, ucs):
    sanding_points = []
    for point in points:
        try:
            if float(point[2]) <= SANDING_Z_THRESHOLD:
                sanding_points.append(point)
        except (TypeError, ValueError, IndexError):
            pass

    if len(sanding_points) < 2:
        return

    communicate(
        cps=cps,
        config=config,
        point=sanding_points[0],
        tcp=tcp,
        ucs=ucs,
        seventh=-1,
        speed=sanding_speed,
        velocity_profile="sanding",
        wait=True,
    )

    putForceZminus(cps=cps, force=force, tcp=tcp, ucs=ucs, config=config)
    turn_vibration_on(cps)

    for end_pose in sanding_points[1:]:
        communicate(
            cps=cps,
            config=config,
            point=end_pose,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=sanding_speed,
            velocity_profile="sanding",
            wait=True,
        )

    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    releaseForce(cps=cps, config=config, wait_for_blending=False)


def _run_smalldoor_side_by_ylen(door_num, force, cps, small_runner, big_runner):
    ylen_data = get_y_values(door_num, default_on_error=True)
    ylen = ylen_data['ylen']
    print("ylen:", ylen)

    json_config = load_json_config()
    speed = float(json_config['robotSpeed'])
    sanding_speed = float(json_config.get('sandingSpeed', json_config['robotSpeed']))

    if ylen == "null":
        print("No door data available - skipping operations")
        return

    if not isinstance(ylen, (int, float)):
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")
        return

    if ylen > 600:
        big_runner(force, cps, speed, sanding_speed)
    else:
        small_runner(force, cps, speed, sanding_speed)


def smalldoor1side(force,cps):
    def smalldoor1sidesmall(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])


        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(1, 0)
        print("b0:", b0)
        b1 = get_frame_point(1, 1)
        print("b1:", b1)
        b2= get_frame_point(1, 2)
        print("b2:", b2)
        b3 = get_frame_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        bottom_axis_offset = -50  # Door 1 bottom only
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0] - bottom_axis_offset,b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0] - bottom_axis_offset,b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2 - bottom_axis_offset,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0] - bottom_axis_offset,b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0] - bottom_axis_offset,b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        communicate(cps=cps,config=config,seventh=x1 + bottom_axis_offset,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)

        # cps.HRIF_DisConnect(0)

    def smalldoor1sidebig(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(1, 0)
        print("b0:", b0)
        b1 = get_frame_point(1, 1)
        print("b1:", b1)
        b2= get_frame_point(1, 2)
        print("b2:", b2)
        b3 = get_frame_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        bottom_axis_offset = -50.0  # Door 1 bottom only
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0] - bottom_axis_offset,b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0] - bottom_axis_offset ,b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2 ,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0] ,b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0] ,b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0] - bottom_axis_offset ,b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0] - bottom_axis_offset,b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0] ,b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,15,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0] - bottom_axis_offset,b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,15,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)


        #Bottom Points Modified
        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2 + 95.6,15,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1= x1
        print("conx1:", conx1)
        conxb=b0[0]
        print ("conxb:",conxb)
        conx2=conxb+(b2[0]-b1[0])/2
        print("conx2:", conx2)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)
        #Bottom Points
        bottomdoorpoints=[bottom4pre,bottom4,bottom4down,bottom0,bottom3,bottom5,bottom5pre]
        print("bottomdoorpoints:", bottomdoorpoints)
        #Upper Points
        upperdoorpoints=[toppoint5pre,toppoint5,toppoint5top,toppoint2,toppoint1,toppoint4,toppoint4pre]
        print("upperdoorpoints:", upperdoorpoints)
        
        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def perform_process_top(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def run_single_movement(lift_point, seventh_axis_point, reentry_point, cps, config):
            """
            Transition safely between passes:
            1) complete lift to clearance,
            2) start 7th-axis motion without waiting,
            3) reposition tool while 7th axis is moving,
            4) ensure 7th-axis move is complete before next pass.

            :param lift_point: Clearance lift point before J7 transition.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param reentry_point: Robot point to approach after J7 transition.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """
            communicate(
                cps=cps,
                config=config,
                point=lift_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_axis_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=speed,
                velocity_profile="robot",
                wait=False,
            )
            communicate(
                cps=cps,
                config=config,
                point=reentry_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            if not _wait_for_j7_idle(cps):
                raise RuntimeError(
                    f"J7 did not become idle after async move to {seventh_axis_point}"
                )
        
        communicate(cps=cps,config=config,seventh=conx1 + bottom_axis_offset,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        run_single_movement(bottom5pre, conx2, toppoint5pre, cps, config)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)

        # cps.HRIF_DisConnect(0)

    _run_smalldoor_side_by_ylen(
        1,
        force,
        cps,
        smalldoor1sidesmall,
        smalldoor1sidebig,
    )


def smalldoor2side(force,cps):
    def smalldoor2sidesmall(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(2, 0)
        print("b0:", b0)
        b1 = get_frame_point(2, 1)
        print("b1:", b1)
        b2= get_frame_point(2, 2)
        print("b2:", b2)
        b3 = get_frame_point(2, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor2sidebig(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(2, 0)
        print("b0:", b0)
        b1 = get_frame_point(2, 1)
        print("b1:", b1)
        b2= get_frame_point(2, 2)
        print("b2:", b2)
        b3 = get_frame_point(2, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,15,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,15,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)

        #Bottom Points Modified
        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1=x1
        print("conx1:", conx1)
        conxb=b0[0]
        print ("conxb:",conxb)
        conx2=conx1+conxb+(b2[0]-b1[0])/2
        print("conx2:", conx2)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)
        #Bottom Points
        bottomdoorpoints=[bottom4pre,bottom4,bottom4down,bottom0,bottom3,bottom5,bottom5pre]
        print("bottomdoorpoints:", bottomdoorpoints)
        #Upper Points
        upperdoorpoints=[toppoint5pre,toppoint5,toppoint5top,toppoint2,toppoint1,toppoint4,toppoint4pre]
        print("upperdoorpoints:", upperdoorpoints)
        
        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def perform_process_top(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def run_single_movement(lift_point, seventh_axis_point, reentry_point, cps, config):
            """
            Transition safely between passes:
            1) complete lift to clearance,
            2) start 7th-axis motion without waiting,
            3) reposition tool while 7th axis is moving,
            4) ensure 7th-axis move is complete before next pass.

            :param lift_point: Clearance lift point before J7 transition.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param reentry_point: Robot point to approach after J7 transition.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """
            communicate(
                cps=cps,
                config=config,
                point=lift_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_axis_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=speed,
                velocity_profile="robot",
                wait=False,
            )
            communicate(
                cps=cps,
                config=config,
                point=reentry_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            if not _wait_for_j7_idle(cps):
                raise RuntimeError(
                    f"J7 did not become idle after async move to {seventh_axis_point}"
                )
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        run_single_movement(bottom5pre, conx2, toppoint5pre, cps, config)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)

    _run_smalldoor_side_by_ylen(
        2,
        force,
        cps,
        smalldoor2sidesmall,
        smalldoor2sidebig,
    )


def smalldoor3side(force,cps):
    def smalldoor3sidesmall(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(3, 0)
        print("b0:", b0)
        b1 = get_frame_point(3, 1)
        print("b1:", b1)
        b2= get_frame_point(3, 2)
        print("b2:", b2)
        b3 = get_frame_point(3, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor3sidebig(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(3, 0)
        print("b0:", b0)
        b1 = get_frame_point(3, 1)
        print("b1:", b1)
        b2= get_frame_point(3, 2)
        print("b2:", b2)
        b3 = get_frame_point(3, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,15,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,15,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)


        #Bottom Points Modified
        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1=x1
        print("conx1:", conx1)
        conxb=b0[0]
        print ("conxb:",conxb)
        conx2=conx1+conxb+(b2[0]-b1[0])/2
        print("conx2:", conx2)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)
        #Bottom Points
        bottomdoorpoints=[bottom4pre,bottom4,bottom4down,bottom0,bottom3,bottom5,bottom5pre]
        print("bottomdoorpoints:", bottomdoorpoints)
        #Upper Points
        upperdoorpoints=[toppoint5pre,toppoint5,toppoint5top,toppoint2,toppoint1,toppoint4,toppoint4pre]
        print("upperdoorpoints:", upperdoorpoints)
        
        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def perform_process_top(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def run_single_movement(lift_point, seventh_axis_point, reentry_point, cps, config):
            """
            Transition safely between passes:
            1) complete lift to clearance,
            2) start 7th-axis motion without waiting,
            3) reposition tool while 7th axis is moving,
            4) ensure 7th-axis move is complete before next pass.

            :param lift_point: Clearance lift point before J7 transition.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param reentry_point: Robot point to approach after J7 transition.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """
            communicate(
                cps=cps,
                config=config,
                point=lift_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_axis_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=speed,
                velocity_profile="robot",
                wait=False,
            )
            communicate(
                cps=cps,
                config=config,
                point=reentry_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            if not _wait_for_j7_idle(cps):
                raise RuntimeError(
                    f"J7 did not become idle after async move to {seventh_axis_point}"
                )
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        run_single_movement(bottom5pre, conx2, toppoint5pre, cps, config)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)
    _run_smalldoor_side_by_ylen(
        3,
        force,
        cps,
        smalldoor3sidesmall,
        smalldoor3sidebig,
    )


def smalldoor4side(force,cps):
    def smalldoor4sidesmall(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(4, 0)
        print("b0:", b0)
        b1 = get_frame_point(4, 1)
        print("b1:", b1)
        b2= get_frame_point(4, 2)
        print("b2:", b2)
        b3 = get_frame_point(4, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor4sidebig(force,cps,speed,sanding_speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        b0 = get_frame_point(4, 0)
        print("b0:", b0)
        b1 = get_frame_point(4, 1)
        print("b1:", b1)
        b2= get_frame_point(4, 2)
        print("b2:", b2)
        b3 = get_frame_point(4, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],15,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],15,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],15,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,15,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,15,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)


        #Bottom Points Modified
        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,15,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1=x1
        print("conx1:", conx1)
        conxb=b0[0]
        print ("conxb:",conxb)
        conx2=conx1+conxb+(b2[0]-b1[0])/2
        print("conx2:", conx2)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)
        #Bottom Points
        bottomdoorpoints=[bottom4pre,bottom4,bottom4down,bottom0,bottom3,bottom5,bottom5pre]
        print("bottomdoorpoints:", bottomdoorpoints)
        #Upper Points
        upperdoorpoints=[toppoint5pre,toppoint5,toppoint5top,toppoint2,toppoint1,toppoint4,toppoint4pre]
        print("upperdoorpoints:", upperdoorpoints)
        
        def perform_process_bottom(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def perform_process_top(cps, config, points1, force):
            return _run_linear_sanding_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
            )
        def run_single_movement(lift_point, seventh_axis_point, reentry_point, cps, config):
            """
            Transition safely between passes:
            1) complete lift to clearance,
            2) start 7th-axis motion without waiting,
            3) reposition tool while 7th axis is moving,
            4) ensure 7th-axis move is complete before next pass.

            :param lift_point: Clearance lift point before J7 transition.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param reentry_point: Robot point to approach after J7 transition.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """
            communicate(
                cps=cps,
                config=config,
                point=lift_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_axis_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=speed,
                velocity_profile="robot",
                wait=False,
            )
            communicate(
                cps=cps,
                config=config,
                point=reentry_point,
                tcp=config['coords']['tcptool4plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=speed,
                velocity_profile="robot",
                wait=True,
            )
            if not _wait_for_j7_idle(cps):
                raise RuntimeError(
                    f"J7 did not become idle after async move to {seventh_axis_point}"
                )
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=speed,velocity_profile="robot",wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        run_single_movement(bottom5pre, conx2, toppoint5pre, cps, config)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,velocity_profile="robot",wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool4plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)
    _run_smalldoor_side_by_ylen(
        4,
        force,
        cps,
        smalldoor4sidesmall,
        smalldoor4sidebig,
    )


if __name__ == "__main__":
    smalldoor1side(force=5)
    smalldoor2side(force=5)
    smalldoor3side(force=5)
    #smalldoor4side(force=5)
