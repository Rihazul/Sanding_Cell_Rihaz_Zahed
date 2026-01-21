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
import threading
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

    timeout = float(time_factor)
    print(
        f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
    )
    return timeout


DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0
SANDING_Z_THRESHOLD = 5.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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
    max_points: Optional[int] = 80,
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
    max_points: Optional[int] = 80,
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
    tcp_name = tcp or config["coords"].get("tcptool1plane1")
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
        ret = cps.HRIF_InitMovePathL(
            box_id, robot_id, track_name, velocity, accel, jerk, ucs_name, tcp_name
        )
        print("[Spiral][Init] ret =", ret)
        if ret != 0:
            return False, None

    ret = cps.HRIF_PushMovePaths(box_id, robot_id, track_name, 1, count, all_points)
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
    completion_timeout=76.0,
) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False

    # Create a lightweight state helper for motion monitoring.
    robot_state = RobotState(config=load_config(), cps_client=cps)

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

    pass  # vibration handled in finalize

    force_active = False
    if force is not None and force_func is not None:
        if config is None:
            print("[Spiral] Missing config for force control; skipping force.")
        else:
            tcp_name = tcp or config["coords"].get("tcptool1plane1")
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
    print("[Spiral][Move] Starting MovePathL...")
    ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print("[Spiral][Move] ret =", ret)
    if ret != 0:
        return False
    vibration_active = False
    
    if vibration:
        turn_vibration_on(cps)
        vibration_active = True

    # Wait for completion (track path state + robot flags)
    ok = True
    start = time.time()
    motion_seen = False
    last_cart = None
    try:
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

            if not motion_seen:
                if robot_state.fetch_and_update_flags():
                    if robot_state.state["flags"].get("in_motion"):
                        motion_seen = True
                if not motion_seen and robot_state.fetch_and_update_pos():
                    curr_cart = list(robot_state.state.get("cartesian_position", []))
                    if last_cart and len(curr_cart) >= 6 and len(last_cart) >= 6:
                        if any(abs(a - b) > 1e-1 for a, b in zip(curr_cart[:6], last_cart[:6])):
                            motion_seen = True
                    last_cart = curr_cart

            # Only allow early exit after motion has started.
            if motion_seen and robot_state.wait_until_still(
                still_seconds=1.0, poll_seconds=0.05
            ):
                print("[Spiral] No motion detected for 1.0s; exiting wait loop.")
                break

            if time.time() - start > completion_timeout:
                print("Timeout waiting for idle. Path state:", pstate)
                ok = False
                break
            time.sleep(0.1)

    finally:
        if vibration_active:
            turn_vibration_off(cps)
        if force_active and config is not None:
            releaseForce(cps=cps, config=config)
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

def smalldoor1side(force,cps):
    def smalldoor1sidesmall(force,cps,speed):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        json_config = load_json_config()

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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2 - bottom_axis_offset,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0] - bottom_axis_offset,b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0] - bottom_axis_offset,b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab1"
            path_initialized = False
            push_failed = False
            total_count = 0
            enable_force = True

             # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return
            
            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )   

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]

                start_pose = [
                    start_pose[0],
                    start_pose[1],
                    start_pose[2],
                    start_pose[3],
                    start_pose[4],
                    start_pose[5],
                ]
                end_pose = [
                    end_pose[0],
                    end_pose[1],
                    end_pose[2],
                    end_pose[3],
                    end_pose[4],
                    end_pose[5],
                ]
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
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
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release Force Control
            releaseForce(cps=cps, config=config)

        communicate(cps=cps,config=config,seventh=x1 + bottom_axis_offset,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)

        # cps.HRIF_DisConnect(0)

    def smalldoor1sidebig(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2 ,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0] ,b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0] ,b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0] - bottom_axis_offset ,b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0] - bottom_axis_offset,b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0] ,b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0] - bottom_axis_offset,b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
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
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2 + 95.6,10,b2[3],b2[4],0]
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
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door1bottomtab1"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=[
                    sanding_points[0][0],
                    sanding_points[0][1],
                    sanding_points[0][2],
                    sanding_points[0][3],
                    sanding_points[0][4],
                    sanding_points[0][5],
                ],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False
            
            # Communicate to each point in sanding_points
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                start_pose = [
                    start_pose[0],
                    start_pose[1],
                    start_pose[2],
                    start_pose[3],
                    start_pose[4],
                    start_pose[5],
                ]
                end_pose = [
                    end_pose[0],
                    end_pose[1],
                    end_pose[2],
                    end_pose[3],
                    end_pose[4],
                    end_pose[5],
                ]
                
                if point == bottom4down:
                    enable_force = True
                if point == bottom0:
                    enable_vibration = True
                    
                # Ensure numeric 6‑DOF
                try:
                    start_pose = [float(v) for v in start_pose[:6]]
                    end_pose = [float(v) for v in end_pose[:6]]
                except (TypeError, ValueError):
                    continue


                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door1toptab1"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == toppoint5top:
                    enable_force = True
                if point == toppoint2:
                    enable_vibration = True
                    
                # Ensure numeric 6‑DOF
                try:
                    start_pose = [float(v) for v in start_pose[:6]]
                    end_pose = [float(v) for v in end_pose[:6]]
                except (TypeError, ValueError):
                    continue
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.2,
                    wait=True
                )

            def run_axis_movement():
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    speed=0.2,
                    wait=True
                )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()
        
        communicate(cps=cps,config=config,seventh=conx1 + bottom_axis_offset,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        #communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        run_single_movement(robot_point=toppoint5pre, seventh_axis_point=conx2, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=conx2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)

        # cps.HRIF_DisConnect(0)

    #Main Function Execution
    # ylen = get_y_values(1)['ylen']
    # print("ylen:",ylen)
    # smalldoor1sidebig(force)
    # smalldoor1sidesmall(force)
    # Get ylen value (with default handling)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    json_config = load_json_config()
    speed = float(json_config['robotSpeed'])

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor1sidebig(force,cps,speed)
        else:
            smalldoor1sidesmall(force,cps,speed)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor2side(force,cps):
    def smalldoor2sidesmall(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )   
            
            enable_force = False
            enable_vibration = False
            
            # Communicate to each point in sanding_points
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom03:
                    enable_force = True
                if point == bottom3:
                    enable_vibration = True
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                    
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor2sidebig(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
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
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
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
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door2bottomtab2"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            enable_force = False
            enable_vibration = False


            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom4down:
                    enable_force = True
                if point == bottom0:
                    enable_vibration = True
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize

                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door2toptab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            
            if len(sanding_points) < 2:
                return
            
            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            enable_force = False
            enable_vibration = False
            
            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == toppoint5top:
                    enable_force = True
                if point == toppoint2:
                    enable_vibration = True
                    
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
            
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.2,
                    wait=True
                )

            def run_axis_movement():
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    speed=0.2,
                    wait=True
                )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        #communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        run_single_movement(robot_point=toppoint5pre, seventh_axis_point=conx2, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=conx2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)

    #Main Function Execution
    # ylen = get_y_values(1)['ylen']
    # print("ylen:",ylen)
    # smalldoor1sidebig(force)
    # smalldoor1sidesmall(force)
    # Get ylen value (with default handling)
    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    json_config = load_json_config()
    speed = float(json_config['robotSpeed'])

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor2sidebig(force,cps,speed)
        else:
            smalldoor2sidesmall(force,cps,speed)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor3side(force,cps):
    def smalldoor3sidesmall(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab3"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass

            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )

            enable_force = False
            enable_vibration = False
            
            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom03:
                    enable_force = True
                if point == bottom3:
                    enable_vibration = True
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor3sidebig(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
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
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
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
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door3bottomtab3"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False
            
            # Communicate to each point in points1
            for i in range(len(sanding_points)-1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom4down:
                    enable_force = True
                if point == bottom0:
                    enable_vibration = True
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                    
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door3toptab3"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            if len(sanding_points) < 2:
                return

            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == toppoint5top:
                    enable_force = True
                if point == toppoint2:
                    enable_vibration = True
                    
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
            
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.2,
                    wait=True
                )

            def run_axis_movement():
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    speed=0.2,
                    wait=True
                )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        #communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        run_single_movement(robot_point=toppoint5pre, seventh_axis_point=conx2, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=conx2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)
    #Main Function Execution
    # ylen = get_y_values(1)['ylen']
    # print("ylen:",ylen)
    # smalldoor1sidebig(force)
    # smalldoor1sidesmall(force)
    # Get ylen value (with default handling)
    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    json_config = load_json_config()
    speed = float(json_config['robotSpeed'])

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor3sidebig(force,cps,speed)
        else:
            smalldoor3sidesmall(force,cps,speed)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor4side(force,cps):
    def smalldoor4sidesmall(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)

        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)

        def perform_process_bottom(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab4"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            # Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            if len(sanding_points) < 2:
                return
            
            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom03:
                    enable_force = True
                if point == bottom3:
                    enable_vibration = True
                
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        # cps.HRIF_DisConnect(0)

    def smalldoor4sidebig(force,cps,speed):
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
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
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
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
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
             # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door4bottomtab4"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            #Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            if len(sanding_points) < 2:
                return
            
            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            enable_force = False
            enable_vibration = False
            

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == bottom4down:
                    enable_force = True
                if point == bottom0:
                    enable_vibration = True
            
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1, force):
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "door4toptab4"
            path_initialized = False
            push_failed = False
            total_count = 0
            
            #Filter only sanding points (skip prepoints like Z=10)
            sanding_points = []
            for p in points1:
                try:
                    if float(p[2]) <= SANDING_Z_THRESHOLD:
                        sanding_points.append(p)
                except (TypeError, ValueError, IndexError):
                    pass
            if len(sanding_points) < 2:
                return
            # Move to first sanding point before pushing spiral
            communicate(
                cps=cps,
                config=config,
                point=sanding_points[0],
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                seventh=-1,
                speed=0.6,
                wait=True,
            )
            
            enable_force = False
            enable_vibration = False

            # Communicate to each point in points1
            for i in range(len(sanding_points) - 1):
                point = sanding_points[i]
                start_pose = sanding_points[i]
                end_pose   = sanding_points[i + 1]
                
                if point == toppoint5top:
                    enable_force = True
                if point == toppoint2:
                    enable_vibration = True
                    
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=start_pose,
                    end_pose=end_pose,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                )
                if not success:
                    push_failed = True
                    break
                path_initialized = True
                total_count += count
            
            if path_initialized and not push_failed:
                if enable_force:
                    pass  # force handled in finalize
                if enable_vibration:
                    pass  # vibration handled in finalize
                    
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                    force=force,
                    force_func=putForceZminus,
                    config=config,
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.2,
                    wait=True
                )

            def run_axis_movement():
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    speed=0.2,
                    wait=True
                )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()
        
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        #communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        run_single_movement(robot_point=toppoint5pre, seventh_axis_point=conx2, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=conx2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speed,wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        # cps.HRIF_DisConnect(0)
    #Main Function Execution
    # ylen = get_y_values(1)['ylen']
    # print("ylen:",ylen)
    # smalldoor1sidebig(force)
    # smalldoor1sidesmall(force)
    # Get ylen value (with default handling)
    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    json_config = load_json_config()
    speed = float(json_config['robotSpeed'])

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor4sidebig(force,cps,speed)
        else:
            smalldoor4sidesmall(force,cps,speed)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


if __name__ == "__main__":
    smalldoor1side(force=5)
    smalldoor2side(force=5)
    smalldoor3side(force=5)
    #smalldoor4side(force=5)
    


