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
    time_factor= float(total_points * distance_per_point / (velocity)) 

    timeout = float(time_factor)
    print(f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s")
    return timeout




def generate_spiral_between_points(
    start_pose,
    end_pose,
    turns: int = 22,
    radius: float = 12.0,
    angle_step_deg: float = 45.0,
    orientation: str = "horizontal",
):
    """
    Build a spiral path between two cartesian poses (X, Y, Z, Rx, Ry, Rz).
    Keeps orientation from start_pose, interpolates center XY between poses,
    and adds radial offsets for each step.
    """
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]
    if orientation == "horizontal":
        if x0 == x1:
            turns = 2
    else:
        if y0 == y1:
            turns = 2

    total_steps = int(turns * (360.0 / angle_step_deg))
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
    turns: int = 30,
    radius: float = 12.0,
    angle_step_deg: float = 10.0,
    orientation: str = "horizontal",
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
        radius=radius,
        angle_step_deg=angle_step_deg,
        orientation=orientation,
    )

    if len(all_points) % 6 != 0:
        print("[Spiral] Point list misaligned (not divisible by 6).")
        return False, None

    count = len(all_points) // 6
    print(f"[Spiral] Total points = {count}")

    if init_path:
        ret = cps.HRIF_InitMovePathL(
            box_id, robot_id,
            track_name,
            velocity,
            accel,
            jerk,
            ucs_name,
            tcp_name
        )
        print("[Spiral][Init] ret =", ret)
        if ret != 0:
            return False, None

    ret = cps.HRIF_PushMovePaths(
        box_id, robot_id,
        track_name,
        1,
        count,
        all_points
    )
    print("[Spiral][Push] ret =", ret)
    if ret != 0:
        return False, None

    return True, count


def run_spiral_horizontal(*args, **kwargs):
    """
    Convenience wrapper for run_spiral_between_points with horizontal orientation.
    """
    kwargs.setdefault("orientation", "horizontal")
    return run_spiral_between_points(*args, **kwargs)


def run_spiral_vertical(*args, **kwargs):
    """
    Convenience wrapper for run_spiral_between_points with vertical orientation.
    """
    kwargs.setdefault("orientation", "vertical")
    return run_spiral_between_points(*args, **kwargs)


def generate_rectangle_path(x_coords, y_coords, z_coords, inner_offset=5.0):
    """Create a simple rectangle path around the boundary with a fixed inner offset."""
    if not (x_coords and y_coords and z_coords):
        return []
    xs = [abs(x) for x in x_coords]
    ys = [abs(y) for y in y_coords]
    z_level = z_coords[0]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_x += inner_offset
    max_x -= inner_offset
    min_y += inner_offset
    max_y -= inner_offset
    rect = [
        [min_x, min_y, z_level, 0, 0, 0],
        [max_x, min_y, z_level, 0, 0, 0],
        [max_x, max_y, z_level, 0, 0, 0],
        [min_x, max_y, z_level, 0, 0, 0],
        [min_x, min_y, z_level, 0, 0, 0],
    ]
    return rect


def move_rectangle(cps, config, rect_points):
    """Drive a rectangle path using communicate."""
    if not rect_points:
        return
    rect_offset = 50.0
    shifted = [
        [pt[0] + rect_offset, pt[1] + rect_offset, pt[2], pt[3], pt[4], pt[5]]
        for pt in rect_points
    ]
    lift = 30.0
    first = shifted[0]
    # Move up, translate to start at safe height, then come down to workplane.
    lifted_start = [first[0], first[1], first[2] + lift, first[3], first[4], first[5]]
    communicate(
        cps=cps,
        config=config,
        point=lifted_start,
        tcp=config['coords']['tcptool1plane1'],
        ucs=config['coords']['ucsTable1'],
        seventh=-1,
        speed=0.5,
        wait=True,
    )
    start_down = first[:]
    communicate(
        cps=cps,
        config=config,
        point=start_down,
        tcp=config['coords']['tcptool1plane1'],
        ucs=config['coords']['ucsTable1'],
        seventh=-1,
        speed=0.5,
        wait=True,
    )
    for pt in shifted:
        communicate(
            cps=cps,
            config=config,
            point=pt,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            seventh=-1,
            speed=0.5,
            wait=True,
        )


def generate_zigzag_with_orientation(
    x_coords,
    y_coords,
    z_coords,
    *,
    tool3x,
    tool3y,
    innerSandingOffset,
    innerOffset,
    innerOffsetX,
    xframe_1=0,
    xframe_2=0,
    orientation="horizontal",
    max_points: int = 200,
):
    """
    Build zigzag coords and prepoint with support for horizontal/vertical orientation.

    Horizontal sweeps start on the right edge and snake downward row by row.
    Vertical sweeps preserve the previous behaviour (columns stepped across X).
    """
    orientation = (orientation or "horizontal").lower()

    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    zigzag_coords = []
    prepoint = None

    if x_coords and y_coords and z_coords:
        z_zigzag = boundary_coords[0][2]

        modified_Point2 = [
            (x_coords[1]) / 1 + tool3x + innerOffsetX,
            y_coords[1] - tool3y - (innerOffset * 0.5),
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

        xlen1 = abs(modified_Point3[0] - modified_Point1[0])
        xinner = xlen1 - xframe_1 - xframe_2

        xs = [abs(p[0]) for p in (modified_Point1, modified_Point2, modified_Point3, modified_Point4)]
        ys = [abs(p[1]) for p in (modified_Point1, modified_Point2, modified_Point3, modified_Point4)]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        if orientation == "horizontal":
            y_span = y_max - y_min
            if y_span > 0:
                num_steps = math.ceil(y_span / innerSandingOffset)
                step = y_span / num_steps
                for row_idx in range(num_steps + 1):
                    current_y = y_max - row_idx * step
                    if current_y < y_min:
                        current_y = y_min
                    if row_idx % 2 == 0:
                        row_points = [
                            [x_max, current_y, z_zigzag, 0, 0, 0],
                            [x_min, current_y, z_zigzag, 0, 0, 0],
                        ]
                    else:
                        row_points = [
                            [x_min, current_y, z_zigzag, 0, 0, 0],
                            [x_max, current_y, z_zigzag, 0, 0, 0],
                        ]
                    zigzag_coords.extend(row_points)

            prepoint = [x_max + 0.5, y_max, z_zigzag, 0, 0, 0]
        else:
            if xinner > 0:
                num_steps = math.ceil(xinner / innerSandingOffset)
                adjusted_step = xinner / num_steps

                offset = 0.0
                toggle = 0

                while offset <= xinner + 1e-9:
                    row_points = [
                        [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                        [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle

            prepoint = [abs(modified_Point1[0]) + 0.5, modified_Point1[1], z_zigzag, 0, 0, 0]

    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])

    if max_points and len(zigzag_coords) > max_points:
        original_last = zigzag_coords[-1]
        stride = math.ceil(len(zigzag_coords) / max_points)
        zigzag_coords = zigzag_coords[::stride]
        if zigzag_coords[-1] != original_last:
            zigzag_coords.append(original_last)

    for point in zigzag_coords:
        point[0] = abs(point[0])
        point[1] = abs(point[1])

    return zigzag_coords, prepoint


def finalize_spiral_path(cps: CPSClient, track_name: str, *, box_id: int = 0, robot_id: int = 0, completion_timeout = 76.0) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False
    # Start vibration only after the path push is closed


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

    print("[Spiral][Move] Starting MovePathL...")
    ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print("[Spiral][Move] ret =", ret)
    if ret != 0:
        return False
    turn_vibration_on(cps)
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
        time.sleep(0.1)
        if robot_state.wait_until_still(still_seconds=2, poll_seconds=0.05):
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

def smalldoor1zizag(force,z,cps, orientation: str = "horizontal"):

    def smalldoor1zizagsmall(force,z,cps, orientation: str = orientation):
        
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(1)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        

        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=80,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=10,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)

        def perform_process_top(cps, config, points1,force, orientation: str = "horizontal"):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=float(json_config['sandingSpeed']),
            #         wait=False
            #     )
            
            spiral_track_name = "small"
            path_initialized = False
            push_failed = False
            total_count = 0
            rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=6,
                    radius=12.0,
                    angle_step_deg=60.0,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0*6.0/60.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )
            move_rectangle(cps, config, rect_path)
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor1zizagbig(force,z,cps, orientation: str = orientation):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points TODO add padding
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(1)
        print("tcx0:", tcx0)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=200,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )

        #Single Pocket Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=1,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "door1big"
            path_initialized = False
            push_failed = False

            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=30,
                    radius=12.0,
                    angle_step_deg=10.0,
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
                # waitForBlending(cps=cps, config=config)

                total_count += count



            if path_initialized and not push_failed:
                print("[Spiral] Finalizing spiral path...")
                timeout = compute_timeout(total_points=total_count, velocity=300.0)
                finalize_spiral_path(cps, spiral_track_name, box_id=0, robot_id=0, completion_timeout=timeout)

            move_rectangle(cps, config, rect_path)
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        # Single vertical pass on one 7th-axis position
        current_tcx = tcx0
        current_prepoint = prepointp1
        current_zigzag = zigzag_pathp1

        communicate(
            cps=cps, config=config, 
            seventh=current_tcx, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.7, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=current_prepoint,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
        perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
    
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor1zizagbig(force,z,cps, orientation=orientation)
        else:
            smalldoor1zizagsmall(force,z,cps, orientation=orientation)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor2zizag(force,z,cps, orientation: str = "horizontal"):
    
    def smalldoor2zizagsmall(force,z,cps, orientation: str = "horizontal"):
        
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
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(2)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=70,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=22,      # add 5 mm margin from edges
            innerOffsetX=15,      # add 5 mm margin in X offset
            orientation=orientation
        )
        rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)
        max_points = 120
        if len(zigzag_pathp1) > max_points:
            original_last = zigzag_pathp1[-1]
            stride = math.ceil(len(zigzag_pathp1) / max_points)
            zigzag_pathp1 = zigzag_pathp1[::stride]
            if zigzag_pathp1[-1] != original_last:
                zigzag_pathp1.append(original_last)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force, orientation: str = "horizontal"):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0
            rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
            #     success, count = run_spiral_between_points(
            #         cps=cps,
            #         config=config,
            #         start_pose=point_A,
            #         end_pose=point_B,
            #         turns=9,  # significantly higher turns on Y moves
            #         radius=12.0,
            #         angle_step_deg=45.0,
            #         orientation=orientation,
            #         track_name=spiral_track_name,
            #         velocity=300.0,
            #         accel=500.0,
            #         jerk=10000.0,
            #         init_path=not path_initialized,
            #     )
            #     if not success:
            #         push_failed = True
            #         break

            #     path_initialized = True
            #     # waitForBlending(cps=cps, config=config)
            #     total_count += count


            # if path_initialized and not push_failed:
            #     timeout = compute_timeout(total_points=total_count, velocity=300.0*16.0/45.0)
            #     finalize_spiral_path(
            #         cps,
            #         spiral_track_name,
            #         box_id=0,
            #         robot_id=0,
            #         completion_timeout=timeout,
            #     )

            move_rectangle(cps, config, rect_path)
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables (horizontal default)
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force, orientation=orientation)  # Dynamic zigzag path
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor2zizagbig(force,z,cps, orientation: str = orientation):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(2)
        print("tcx0:", tcx0)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=120,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )

        #Single Pocket Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=1,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "door2_big"
            path_initialized = False
            push_failed = False
            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=30,
                    radius=12.0,
                    angle_step_deg=10.0,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            move_rectangle(cps, config, generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0))
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
            move_rectangle(cps, config, rect_path)
        
        # Single vertical pass on one 7th-axis position
        current_tcx = tcx0
        current_prepoint = prepointp1
        current_zigzag = zigzag_pathp1

        communicate(
            cps=cps, config=config, 
            seventh=current_tcx, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.7, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=current_prepoint,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
        perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
    
    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        # if ylen > 600:
        #     smalldoor2zizagbig(force,z,cps, orientation=orientation)
        # else:
        smalldoor2zizagsmall(force,z,cps, orientation=orientation)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

    
def smalldoor3zizag(force,z,cps, orientation: str = "horizontal"):

    def smalldoor3zizagsmall(force,z,cps, orientation: str = orientation):
        
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
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(3)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=50,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=10,
            orientation=orientation
        )
        # Down-sample if too many points to avoid controller push limit
        max_points = 240
        if len(zigzag_pathp1) > max_points:
            stride = math.ceil(len(zigzag_pathp1) / max_points)
            zigzag_pathp1 = zigzag_pathp1[::stride]
            # Ensure last point closes path
            if zigzag_pathp1 and zigzag_pathp1[-1] != zigzag_pathp1[-1:]:
                zigzag_pathp1.append(zigzag_pathp1[-1])
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        rect_path = generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=10,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0*10.0/45.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            move_rectangle(cps, config, generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0))
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor3zizagbig(force,z,cps, orientation: str = orientation):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(3)
        print("tcx0:", tcx0)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=80,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )

        #Single Pocket Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=1,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "door3_big"
            path_initialized = False
            push_failed = False
            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=30,
                    radius=12.0,
                    angle_step_deg=10.0,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )
            move_rectangle(cps, config, generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0))
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        # Single vertical pass on one 7th-axis position
        current_tcx = tcx0
        current_prepoint = prepointp1
        current_zigzag = zigzag_pathp1

        communicate(
            cps=cps, config=config, 
            seventh=current_tcx, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.7, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=current_prepoint,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
        perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
    
    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor3zizagbig(force,z,cps, orientation=orientation)
        else:
            smalldoor3zizagsmall(force,z,cps, orientation=orientation)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor4zizag(force,z,cps, orientation: str = "horizontal"):

    def smalldoor4zizagsmall(force,z,cps, orientation: str = orientation):
        
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
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(4)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=30,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=10,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=10,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0*10.0/45.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            move_rectangle(cps, config, generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0))
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor4zizagbig(force,z,cps, orientation: str = orientation):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(4)
        print("tcx0:", tcx0)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ",x_coords1)
        print("y_coords2 ",y_coords1)
        print("z_coords3",z_coords1)

        # Global variables
        #prepoint = None
        #zigzag_coords = []

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, orientation=orientation):
            return generate_zigzag_with_orientation(
                x_coords,
                y_coords,
                z_coords,
                tool3x=38.1,
                tool3y=50.8,
                innerSandingOffset=80,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                xframe_1=0,
                xframe_2=0,
                orientation=orientation,
            )

        #Single Pocket Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=1,
            orientation=orientation
        )
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            
            # Communicate to each point in points1
            # for point in points1:
            #     communicate(
            #         cps=cps,
            #         config=config,
            #         point=point,
            #         tcp=config['coords']['tcptool1plane1'],
            #         ucs=config['coords']['ucsTable1'],
            #         seventh=-1,
            #         speed=0.6,
            #         wait=False
            #     )
            
            spiral_track_name = "door4_big"
            path_initialized = False
            push_failed = False
            total_count = 0

            for index,_ in enumerate(points1):
                point_A = points1[index]
                if index + 1 >= len(points1):
                    break
                point_B = points1[index + 1]

                # communicate(
                #     cps=cps,    
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    turns=30,
                    radius=12.0,
                    angle_step_deg=10.0,
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
                # waitForBlending(cps=cps, config=config)
                total_count += count


            if path_initialized and not push_failed:
                timeout = compute_timeout(total_points=total_count, velocity=300.0)
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )
            move_rectangle(cps, config, generate_rectangle_path(x_coords1, y_coords1, z_coords1, inner_offset=5.0))
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        # Single vertical pass on one 7th-axis position
        current_tcx = tcx0
        current_prepoint = prepointp1
        current_zigzag = zigzag_pathp1

        communicate(
            cps=cps, config=config, 
            seventh=current_tcx, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.7, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=current_prepoint,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
        perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.7, wait=True
        )
    
    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor4zizagbig(force,z,cps, orientation=orientation)
        else:
            smalldoor4zizagsmall(force,z,cps, orientation=orientation)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")



if __name__ == "__main__":
    smalldoor1zizag(force=5,z=-6.5)
    smalldoor2zizag(force=5,z=-6.5)
    smalldoor3zizag(force=5,z=-6.5)
    #smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
