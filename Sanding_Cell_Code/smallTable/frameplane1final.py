# testcommu.py

import sys
import os
import time
import json
import math
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
    base_name = track_name or "spiral_path"
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

   # turn_vibration_on(cps)

    print("[Spiral][Move] Starting MovePathL...")
    ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print("[Spiral][Move] ret =", ret)
    if ret != 0:
        return False

    # Wait for completion (track path state + robot flags)
    start = time.time()
    while True:
        pstate = []
        pret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
        if pret != 0:
            print("HRIF_ReadPathState failed:", pret)
            return False
        if len(pstate) > 3 and pstate[3] != "0":
            print(f"[Spiral] Path reported error: {pstate}")
            return False

        # Break early if the robot stays still for 0.5s.
        if robot_state.wait_until_still(still_seconds=0.2, poll_seconds=0.05):
            print("[Spiral] No motion detected for 0.5s; exiting wait loop.")
            break

        # motion_result = []
        # mret = cps.HRIF_IsMotionDone(box_id, robot_id, motion_result)
        # print(f"[Spiral] Path state: {pstate}, Motion done: {motion_result}")
        # if mret != 0:
        #     print("HRIF_IsMotionDone failed:", mret)
        #     return False

        # if motion_result[0]:
        #     break

        # if time.time() - start > completion_timeout:
        #     print("Timeout waiting for idle. Path state:", pstate)
        #     return False
        # time.sleep(0.1)
        # print(f"Waiting for spiral move to complete... path={pstate}\r", end="")

    return True

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

        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab1"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom03:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom3:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
                )

            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release Force Control
            releaseForce(cps=cps, config=config)

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=speed,wait=True)
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
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2 + 95.6,10,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1=x1
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
        
        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom4down:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom0:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
                )
            
            # Wait for blending and turn off vibration
            #waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab1"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==toppoint5top:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==toppoint2:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
                    
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

        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom03:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom3:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
        
        def perform_process_bottom(cps, config, points1,force):
            
             # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom4down:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom0:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
                )
            
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==toppoint5top:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==toppoint2:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
                    
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

        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab3"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom03:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom3:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
        
        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab3"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom4down:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom0:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
                )
            
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab3"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==toppoint5top:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==toppoint2:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
                    
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

        def perform_process_bottom(cps, config, points1,force):
            
    
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab4"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom03:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom3:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
        
        def perform_process_bottom(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab4"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==bottom4down:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom0:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
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
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def perform_process_top(cps, config, points1,force):
            
            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab4"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Communicate to each point in points1
            for i, point in enumerate(points1):
                start_pose = points1[i]
                end_pose   = points1[i + 1]
                if point==toppoint5top:
                    putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==toppoint2:
                    time.sleep(0.01)
                    turn_vibration_on(cps)
                    
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
    