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
from smallTable.spiral_cache import (
    save_full_track_to_excel,
    load_full_track_from_excel,
    load_full_track_with_validation,
    cache_exists,
    _generate_cache_key,
    get_cache_path,
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




DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0


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
    radius_val = getattr(settings, "radius_mm", None) if not isinstance(settings, dict) else settings.get("radius_mm")
    linear_val = getattr(settings, "linear_speed_mm_s", None) if not isinstance(settings, dict) else settings.get("linear_speed_mm_s")
    DEFAULT_SPIRAL_LINEAR_SPEED = _clamp(float(linear_val if linear_val is not None else DEFAULT_SPIRAL_LINEAR_SPEED), 100.0, 300.0)
    DEFAULT_SPIRAL_RADIUS = _clamp(float(radius_val if radius_val is not None else DEFAULT_SPIRAL_RADIUS), 10.0, 15.0)


def generate_spiral_between_points(
    start_pose,
    end_pose,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = 80,
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
        turns = 3
    elif x0 == x1 and orientation == "horizontal":
        turns = 3
    else:
        dist = max(abs(x1 - x0), abs(y1 - y0))
        turns = (4/26) * dist 

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
    orientation: str = "horizontal",
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


def finalize_spiral_path(cps: CPSClient, track_name: str, *, box_id: int = 0, robot_id: int = 0, completion_timeout = 76.0) -> bool:
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

    turn_vibration_on(cps)

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


def run_spiral_with_cache(
    cps: CPSClient,
    config: dict,
    zigzag_points: list,
    *,
    door_id: int = 1,
    track_name: str = "spiral_cached",
    orientation: str = "horizontal",
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
    max_points: int = 80,
    velocity: float = 300.0,
    accel: float = 500.0,
    jerk: float = 10000.0,
    tcp: str = None,
    ucs: str = None,
    use_cache: bool = True,
    force_regenerate: bool = False,
    door_x_length: float = None,
    door_y_length: float = None,
) -> bool:
    """
    Run spiral path with Excel caching to avoid recalculation.
    Validates cached path against current door dimensions.
    
    Args:
        cps: Robot client
        config: Configuration dict
        zigzag_points: List of [x,y,z,rx,ry,rz] waypoints
        door_id: Door identifier for cache key
        track_name: Name for the robot track
        orientation: "horizontal" or "vertical"
        radius, turns, angle_step_deg: Spiral parameters
        velocity, accel, jerk: Motion parameters
        use_cache: If True, try to load from cache first
        force_regenerate: If True, regenerate even if cache exists
        door_x_length: Current door X dimension (for cache validation)
        door_y_length: Current door Y dimension (for cache validation)
    
    Returns:
        True if successful
    """
    tcp_name = tcp or config["coords"].get("tcptool1plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")
    
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    all_points = None
    total_count = 0
    
    # Try to load from cache with dimension validation
    if use_cache and not force_regenerate and door_x_length and door_y_length:
        cached_points, metadata, is_valid = load_full_track_with_validation(
            track_name, door_id, orientation,
            current_x_length=door_x_length,
            current_y_length=door_y_length,
            radius=radius, turns=turns, angle_step_deg=angle_step_deg
        )
        if cached_points and is_valid:
            all_points = cached_points
            total_count = len(all_points) // 6
            print(f"[SpiralCache] Loaded {total_count} points from cache (dimensions validated)")
    elif use_cache and not force_regenerate:
        # No dimensions provided, load without validation (legacy mode)
        cached_points, metadata = load_full_track_from_excel(
            track_name, door_id, orientation, radius, turns, angle_step_deg
        )
        if cached_points:
            all_points = cached_points
            total_count = len(all_points) // 6
            print(f"[SpiralCache] Loaded {total_count} points from cache (no dimension validation)")
    
    # Generate if not cached or validation failed
    if all_points is None:
        print("[SpiralCache] Generating spiral points (will cache for next time)...")
        all_segments = []
        all_points = []
        
        for index in range(len(zigzag_points) - 1):
            start_pose = zigzag_points[index]
            end_pose = zigzag_points[index + 1]
            
            segment_points = generate_spiral_between_points(
                start_pose=start_pose,
                end_pose=end_pose,
                turns=turns,
                radius=radius,
                angle_step_deg=angle_step_deg,
                max_points=max_points,
                orientation=orientation,
            )
            
            all_segments.append((start_pose, end_pose, segment_points))
            all_points.extend(segment_points)
        
        total_count = len(all_points) // 6
        
        # Save to cache with door dimensions
        if use_cache:
            save_full_track_to_excel(
                track_name=track_name,
                door_id=door_id,
                orientation=orientation,
                all_segments=all_segments,
                radius=radius,
                turns=turns,
                angle_step_deg=angle_step_deg,
                door_x_length=door_x_length,
                door_y_length=door_y_length,
            )
    
    # Now push and execute the path
    if len(all_points) % 6 != 0:
        print("[SpiralCache] Point list misaligned")
        return False
    
    # Initialize path
    ret = cps.HRIF_InitMovePathL(
        0, 0,
        track_name,
        velocity,
        accel,
        jerk,
        ucs_name,
        tcp_name
    )
    if ret != 0:
        print(f"[SpiralCache] InitMovePathL failed: {ret}")
        return False
    
    # Push all points at once
    ret = cps.HRIF_PushMovePaths(
        0, 0,
        track_name,
        1,
        total_count,
        all_points
    )
    if ret != 0:
        print(f"[SpiralCache] PushMovePaths failed: {ret}")
        return False
    
    print(f"[SpiralCache] Pushed {total_count} points to robot")
    
    # Finalize and execute
    timeout = compute_timeout(total_points=total_count, velocity=velocity)
    return finalize_spiral_path(cps, track_name, completion_timeout=timeout)


def generate_zigzag_path(
    x_coords, y_coords, z_coords, 
    innerOffset,innerOffsetX, 
    orientation="horizontal", movement="zigzag",
    innerSandingOffset=50):
    prepoint = None
    zigzag_coords = []
    
    # Parameters (adjust as needed)
    tool3y = 50.8   # Tool offset in Y
    tool3x = 38.1   # Tool offset in X
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
            (x_coords[1])/1 + tool3x + innerOffsetX,
            y_coords[1] - tool3y - innerOffset,
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            (x_coords[0])/1 + tool3x + innerOffsetX,
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
        x_min = min(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        x_max = max(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        y_min = min(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])
        y_max = max(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])

        # Calculate available horizontal dimension
        xlen1 = abs(modified_Point3[0] - modified_Point1[0])
        xinner = xlen1 - xframe_1 - xframe_2
        print("xinner=", xinner)

        orientation_mode = (orientation or "vertical").lower()
        movement_mode = (movement or "zigzag").lower()
        if movement_mode == "rect":
            zigzag_coords = [
                [modified_Point1[0], modified_Point1[1], z_zigzag, 0, 0, 0],
                [modified_Point2[0], modified_Point2[1], z_zigzag, 0, 0, 0],
                [modified_Point3[0], modified_Point3[1], z_zigzag, 0, 0, 0],
                [modified_Point4[0], modified_Point4[1], z_zigzag, 0, 0, 0],
                [modified_Point1[0], modified_Point1[1], z_zigzag, 0, 0, 0],
            ]
        elif orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.ceil(yinner / innerSandingOffset)
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag rows from top (max y) to bottom (min y)
                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [x_max, current_y, z_zigzag, 0, 0, 0],
                        [x_min, current_y, z_zigzag, 0, 0, 0],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
        else:
            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / innerSandingOffset)
                adjusted_step = xinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag path from left to right
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    row_points = [
                        [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                        [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
                    ]
                    # Reverse every other row to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle

        # Update only the y coordinate to its absolute value
        for point in zigzag_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])
    
        if movement_mode == "rect":
            prepoint = [abs(zigzag_coords[0][0])+0.5, zigzag_coords[0][1], z_zigzag, 0, 0, 0]
        elif orientation_mode == "horizontal":
            prepoint = [abs(x_max)+0.5, y_max, z_zigzag, 0, 0, 0]
        else:
            prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
    return zigzag_coords,prepoint

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

def smalldoor1zizag(force,z,cps, orientation="horizontal", movement="zigzag", spiral_settings=None):
    apply_spiral_settings(spiral_settings)
    
    def smalldoor1zizagsmall(force,z,cps):
        
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



        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, 
                                                       innerOffset=17,innerOffsetX=17, 
                                                       orientation=orientation, movement=movement,
                                                       innerSandingOffset=50)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1, force, door_x_len, door_y_len):
            """
            Execute spiral sanding with caching.
            
            Args:
                cps: Robot client
                config: Configuration dict
                points1: Zigzag waypoints
                force: Force control value
                door_x_len: Current door X dimension (for cache validation)
                door_y_len: Current door Y dimension (for cache validation)
            """
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            print("Force Control Activated")
            
            # Run spiral with caching (replaces the old loop)
            success = run_spiral_with_cache(
                cps=cps,
                config=config,
                zigzag_points=points1,
                door_id=1,                      # Door 1
                track_name="small_door1",       # Unique track name
                orientation=orientation,        # "horizontal" or "vertical"
                radius=12.0,
                turns=12,                       # Or calculate based on lane length
                angle_step_deg=45.0,
                velocity=300.0,
                accel=500.0,
                jerk=10000.0,
                use_cache=True,                 # Enable caching
                force_regenerate=False,         # Set True to force recalculation
                door_x_length=door_x_len,       # For cache validation
                door_y_length=door_y_len,       # For cache validation
            )
            
            if not success:
                print("[ERROR] Spiral execution failed!")
            
            # Turn off vibration and release force
            turn_vibration_off(cps)
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
        
        # Get current door dimensions for cache validation
        x_data = get_x_values(1, default_on_error=True)
        y_data = get_y_values(1, default_on_error=True)
        door_x_len = x_data.get('xlen', 390)  # Default if not available
        door_y_len = y_data.get('ylen', 700)  # Default if not available
        
        # Execute spiral with caching
        perform_process_top(
            cps, config, 
            points1=zigzag_pathp1,
            force=force,
            door_x_len=door_x_len,
            door_y_len=door_y_len
        )
        
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    

    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor1zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor2zizag(force,z,cps, orientation="horizontal", movement="zigzag", spiral_settings=None):
    apply_spiral_settings(spiral_settings)
    
    def smalldoor2zizagsmall(force,z,cps):
        
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

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, 
            innerOffset=17,innerOffsetX=10, 
            orientation=orientation, movement=movement, innerSandingOffset=50)
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
            turn_vibration_on(cps)
            
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
                    turns=6,
                    radius=12.0,
                    angle_step_deg=90.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
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
    

    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor2zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

    
def smalldoor3zizag(force,z,cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    apply_spiral_settings(spiral_settings)
    
    def smalldoor3zizagsmall(force,z,cps):
        
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



        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, 
            innerOffset=22,innerOffsetX=10, 
            orientation=orientation, movement=movement,
            innerSandingOffset=50)
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
            turn_vibration_on(cps)
            
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
                    turns=6,
                    radius=12.0,
                    angle_step_deg=90.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
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
    

    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor3zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor4zizag(force,z,cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    apply_spiral_settings(spiral_settings)
    
    def smalldoor4zizagsmall(force,z,cps):
        
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



        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, 
            innerOffset=17,innerOffsetX=10, 
            orientation=orientation, movement=movement,
            innerSandingOffset=50)
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
            turn_vibration_on(cps)
            
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
                    turns=6,
                    radius=12.0,
                    angle_step_deg=90.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
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
    

    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor4zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")



if __name__ == "__main__":
    smalldoor1zizag(force=5,z=-6.5)
    smalldoor2zizag(force=5,z=-6.5)
    smalldoor3zizag(force=5,z=-6.5)
    #smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
