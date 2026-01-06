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
# Fast cache using pickle + in-memory (100x faster than Excel)
from smallTable.spiral_cache_fast import (
    save_spiral_track,
    load_with_validation,
    load_from_cache,
    cache_exists,
    _generate_cache_key,
    get_pickle_path as get_cache_path,
    preload_all_caches,
)

# ============================================================
# FAST EXECUTION: Load pre-generated paths and execute directly
# No calculation delay - paths were generated after scanning
# ============================================================

# Default spiral parameters (must match path_pregenerator.py)
_PREGEN_PARAMS = {
    "radius": 12.0,
    "turns": 12,
    "angle_step_deg": 45.0,
    "velocity": 300.0,
    "accel": 500.0,
    "jerk": 10000.0,
}


def execute_precached_spiral(
    cps,
    config: dict,
    door_id: int,
    orientation: str = "horizontal",
    force: float = 3.0,
    params: dict = None,
) -> bool:
    """
    Execute a pre-generated spiral path from cache.
    NO CALCULATION - just load and execute immediately!
    
    Call this instead of run_spiral_with_cache() when paths are pre-generated.
    
    Args:
        cps: Robot CPS client
        config: Configuration dict
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        force: Force control value
        params: Override spiral parameters (must match what was used during pre-generation)
        
    Returns:
        True if successful
    """
    p = {**_PREGEN_PARAMS, **(params or {})}
    
    track_name = f"small_door{door_id}_{orientation}"
    cache_key = _generate_cache_key(
        track_name, door_id, orientation,
        p["radius"], p["turns"], p["angle_step_deg"]
    )
    
    # Load from cache - INSTANT from memory!
    all_points, metadata = load_from_cache(cache_key)
    
    if all_points is None:
        print(f"[FastExec] ERROR: No cached path for door{door_id}_{orientation}")
        print("[FastExec] Paths must be pre-generated after scanning!")
        print("[FastExec] Run: from smallTable.path_pregenerator import pregenerate_all_paths")
        return False
    
    total_count = len(all_points) // 6
    print(f"[FastExec] Loaded {total_count} points from cache - INSTANT!")
    
    # Get TCP/UCS from config
    tcp_name = config['coords'].get('tcptool1plane1')
    ucs_name = config['coords'].get('ucsTable1')
    
    # Initialize path on robot
    ret = cps.HRIF_InitMovePathL(
        0, 0,
        track_name,
        p["velocity"],
        p["accel"],
        p["jerk"],
        ucs_name,
        tcp_name
    )
    if ret != 0:
        print(f"[FastExec] InitMovePathL failed: {ret}")
        return False
    
    # Push all points at once to robot controller
    ret = cps.HRIF_PushMovePaths(
        0, 0,
        track_name,
        1,
        total_count,
        all_points
    )
    if ret != 0:
        print(f"[FastExec] PushMovePaths failed: {ret}")
        return False
    
    print(f"[FastExec] Pushed {total_count} points - waiting for PATH_READY...")
    
    # Activate force control
    putForceZminus(
        cps=cps,
        force=force,
        tcp=tcp_name,
        ucs=ucs_name,
        config=config
    )
    
    # Finalize and execute
    timeout = compute_timeout(total_points=total_count, velocity=p["velocity"])
    success = finalize_spiral_path(cps, track_name, completion_timeout=timeout)
    
    # Cleanup
    turn_vibration_off(cps)
    releaseForce(cps=cps, config=config)
    
    return success


def get_cached_prepoint(door_id: int, orientation: str = "horizontal", params: dict = None):
    """
    Get the prepoint (approach position) and door position from cached metadata.
    Use this for motion setup before executing the cached path.
    
    Returns:
        Tuple of (prepoint, door_position) or (None, None) if not cached
    """
    p = {**_PREGEN_PARAMS, **(params or {})}
    track_name = f"small_door{door_id}_{orientation}"
    cache_key = _generate_cache_key(
        track_name, door_id, orientation,
        p["radius"], p["turns"], p["angle_step_deg"]
    )
    
    _, metadata = load_from_cache(cache_key)
    if metadata:
        return metadata.get("prepoint"), metadata.get("door_position")
    return None, None


# ============================================================
# SIMPLE DOOR EXECUTION FUNCTIONS
# Just load cached track and execute MovePathL - no calculation!
# ============================================================

def run_door_spiral_fast(
    cps,
    door_id: int,
    orientation: str = "horizontal",
    force: float = 3.0,
    z: float = -6.5,
) -> bool:
    """
    Execute pre-cached spiral for a door. Simple wrapper that:
    1. Moves to door position (7th axis)
    2. Moves to prepoint (approach)
    3. Loads cached track
    4. Executes MovePathL
    
    Args:
        cps: Robot CPS client
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        force: Force control value
        z: Z depth (unused, tracks already have Z)
        
    Returns:
        True if successful
    """
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])
    
    # Get cached prepoint and door position
    prepoint, door_position = get_cached_prepoint(door_id, orientation)
    
    if prepoint is None:
        print(f"[RunDoor] ERROR: No cached path for door {door_id} {orientation}")
        print("[RunDoor] Run scanning first to generate paths!")
        return False
    
    prehoming = [0, 200, 50, 0, 0, 0]
    tcp = config['coords']['tcptool1plane1']
    ucs = config['coords']['ucsTable1']
    
    # Move to door position (7th axis)
    communicate(
        cps=cps, config=config,
        seventh=door_position,
        tcp=tcp, ucs=ucs,
        speed=0.2, wait=True
    )
    
    # Move to prehoming
    communicate(
        cps=cps, config=config,
        point=prehoming,
        tcp=tcp, ucs=ucs,
        seventh=-1,
        speed=0.2, wait=True
    )
    
    # Move to prepoint (approach position)
    communicate(
        cps=cps, config=config,
        point=prepoint,
        tcp=tcp, ucs=ucs,
        seventh=-1,
        speed=0.2, wait=True
    )
    
    # Execute the cached spiral path
    success = execute_precached_spiral(
        cps=cps,
        config=config,
        door_id=door_id,
        orientation=orientation,
        force=force,
    )
    
    # Return to prehoming
    communicate(
        cps=cps, config=config,
        point=prehoming,
        tcp=tcp, ucs=ucs,
        seventh=-1,
        speed=0.2, wait=True
    )
    
    return success


def smalldoor1_fast(force: float = 3.0, z: float = -6.5, cps=None, orientation: str = "horizontal"):
    """Fast execution for Door 1 - loads cached track and runs MovePathL."""
    return run_door_spiral_fast(cps, door_id=1, orientation=orientation, force=force, z=z)


def smalldoor2_fast(force: float = 3.0, z: float = -6.5, cps=None, orientation: str = "horizontal"):
    """Fast execution for Door 2 - loads cached track and runs MovePathL."""
    return run_door_spiral_fast(cps, door_id=2, orientation=orientation, force=force, z=z)


def smalldoor3_fast(force: float = 3.0, z: float = -6.5, cps=None, orientation: str = "horizontal"):
    """Fast execution for Door 3 - loads cached track and runs MovePathL."""
    return run_door_spiral_fast(cps, door_id=3, orientation=orientation, force=force, z=z)


def smalldoor4_fast(force: float = 3.0, z: float = -6.5, cps=None, orientation: str = "horizontal"):
    """Fast execution for Door 4 - loads cached track and runs MovePathL."""
    return run_door_spiral_fast(cps, door_id=4, orientation=orientation, force=force, z=z)


def run_all_doors_fast(cps, force: float = 3.0, orientations: list = None):
    """
    Execute all 4 doors with specified orientations.
    
    Args:
        cps: Robot CPS client
        force: Force control value
        orientations: List of orientations per door, e.g., ["horizontal", "horizontal", "vertical", "vertical"]
                     Defaults to all horizontal
    """
    orientations = orientations or ["horizontal"] * 4
    
    results = []
    for door_id in [1, 2, 3, 4]:
        orient = orientations[door_id - 1] if door_id <= len(orientations) else "horizontal"
        print(f"\n[RunAll] ===== Door {door_id} ({orient}) =====")
        success = run_door_spiral_fast(cps, door_id, orient, force)
        results.append({"door": door_id, "orientation": orient, "success": success})
    
    return results


def compute_timeout(
    *,
    total_points: Optional[int] = None,
    cap: float = 120.0,
    velocity: float = 300.0,
) -> float:
    """
    Estimate a completion timeout based on total points.
    This is just a safety cap - actual completion is detected by HRIF_IsMotionDone.
    """
    if total_points is None:
        return cap

    distance_per_point = 4.64 
    time_factor = float(total_points * distance_per_point / velocity)
    
    # Add 50% buffer and cap at max
    timeout = min(time_factor * 1.5, cap)
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
        turns = (4/180) * dist 

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


def finalize_spiral_path(cps: CPSClient, track_name: str, *, box_id: int = 0, robot_id: int = 0, completion_timeout: float = 120.0) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print(f"[Spiral][EndPush] ret = {ret}")
    if ret != 0:
        return False

    # Wait for PATH_READY (state[2] == "3" means ready, state[3] == "0" means no error)
    start = time.time()
    while True:
        st = []
        ret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, st)
        if ret == 0 and len(st) > 3:
            if st[2] == "3" and st[3] == "0":
                print(f"[Spiral] PATH_READY in {(time.time()-start)*1000:.0f}ms, state={st}")
                break
            if st[3] != "0":
                print(f"[Spiral] Path error during prep: state={st}")
                return False
        elapsed = time.time() - start
        if elapsed > 30.0:
            print(f"[Spiral] Timeout waiting for PATH_READY after {elapsed:.1f}s: state={st}")
            return False
        time.sleep(0.01)

    # Turn on vibration AFTER path is ready
    turn_vibration_on(cps)

    # Start motion
    print("[Spiral][Move] Starting MovePathL...")
    ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print(f"[Spiral][Move] ret = {ret}")
    if ret != 0:
        print(f"[Spiral] MovePathL failed with code {ret}")
        return False

    # PHASE 1: Wait for robot to START moving (up to 2 seconds)
    start = time.time()
    robot_started = False
    while time.time() - start < 2.0:
        robot_state = []
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, robot_state)
        if ret == 0 and len(robot_state) > 2:
            in_motion = robot_state[0] == "1"
            has_error = robot_state[2] == "1"
            
            if has_error:
                error_code = robot_state[3] if len(robot_state) > 3 else "unknown"
                print(f"[Spiral] Robot ERROR before starting motion! Error code: {error_code}")
                return False
            
            if in_motion:
                print(f"[Spiral] Robot started moving after {(time.time()-start)*1000:.0f}ms")
                robot_started = True
                break
        time.sleep(0.05)
    
    if not robot_started:
        # Check if there's an error
        robot_state = []
        cps.HRIF_ReadRobotState(box_id, robot_id, robot_state)
        if len(robot_state) > 2 and robot_state[2] == "1":
            print(f"[Spiral] Robot has error, state: {robot_state[:5]}")
            return False
        print(f"[Spiral] Warning: Robot didn't report in_motion within 2s, continuing anyway...")

    # PHASE 2: Wait for motion to complete
    start = time.time()
    last_print = 0
    while True:
        # Check robot state
        robot_state = []
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, robot_state)
        if ret == 0 and len(robot_state) > 12:
            in_motion = robot_state[0] == "1"
            has_error = robot_state[2] == "1"
            motion_done = robot_state[11] == "1"
            
            # Check for errors first
            if has_error:
                error_code = robot_state[3] if len(robot_state) > 3 else "unknown"
                print(f"[Spiral] Robot ERROR during motion! Error code: {error_code}")
                return False
            
            # Motion complete when: not moving AND motion_done flag is set
            if not in_motion and motion_done:
                print(f"[Spiral] Motion completed in {time.time()-start:.1f}s")
                break

        # Check path state for errors
        pstate = []
        pret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
        if pret == 0 and len(pstate) > 3 and pstate[3] != "0":
            print(f"[Spiral] Path reported error: {pstate}")
            return False

        elapsed = time.time() - start
        if elapsed > completion_timeout:
            print(f"[Spiral] Timeout after {elapsed:.1f}s. Robot state: {robot_state[:5]}")
            return False

        # Print progress every 2 seconds
        if elapsed - last_print >= 2.0:
            print(f"[Spiral] Moving... {elapsed:.1f}s, in_motion={robot_state[0] if robot_state else '?'}")
            last_print = elapsed

        time.sleep(0.1)

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
    
    # Try to load from FAST cache with dimension validation
    if use_cache and not force_regenerate and door_x_length and door_y_length:
        cached_points, metadata, is_valid = load_with_validation(
            track_name, door_id, orientation,
            current_x=door_x_length,
            current_y=door_y_length,
            radius=radius, turns=turns, angle_step_deg=angle_step_deg
        )
        if cached_points and is_valid:
            all_points = cached_points
            total_count = len(all_points) // 6
            print(f"[FastCache] Ready! {total_count} points loaded")
    elif use_cache and not force_regenerate:
        # No dimensions provided, load without validation
        cached_points, metadata = load_from_cache(cache_key)
        if cached_points:
            all_points = cached_points
            total_count = len(all_points) // 6
            print(f"[FastCache] Ready! {total_count} points (no validation)")
    
    # Generate if not cached or validation failed
    if all_points is None:
        print("[FastCache] Generating spiral points (will cache)...")
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
            
            all_points.extend(segment_points)
        
        total_count = len(all_points) // 6
        
        # Save to FAST cache with door dimensions
        if use_cache:
            save_spiral_track(
                track_name=track_name,
                door_id=door_id,
                orientation=orientation,
                all_points=all_points,
                radius=radius,
                turns=turns,
                angle_step_deg=angle_step_deg,
                door_x_length=door_x_length,
                door_y_length=door_y_length,
            )
    
    # Now push and execute the path
    if len(all_points) % 6 != 0:
        print("[FastCache] Point list misaligned")
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
        print(f"[FastCache] InitMovePathL failed: {ret}")
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
        print(f"[FastCache] PushMovePaths failed: {ret}")
        return False
    
    print(f"[FastCache] Pushed {total_count} points to robot")
    
    # Finalize and execute
    timeout = compute_timeout(total_points=total_count, velocity=velocity)
    return finalize_spiral_path(cps, track_name, completion_timeout=timeout)

# While robot executes path A, prepare path B in background
def prepare_next_path_async(cps, next_track_name, next_points):
    """Call this while robot is moving on current path"""
    cps.HRIF_InitMovePathL(0, 0, next_track_name, ...)
    cps.HRIF_PushMovePaths(0, 0, next_track_name, 1, len(next_points)//6, next_points)
    cps.HRIF_EndPushPathPoints(0, 0, next_track_name)
    # Now next_track_name is buffered and will reach PATH_READY faster

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
                num_steps = math.floor(yinner / innerSandingOffset)
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
                track_name=f"small_door1_{orientation}",  # Must match pre-generator format!
                orientation=orientation,        # "horizontal" or "vertical"
                radius=12.0,                       # Or calculate based on lane length
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
            
            # Get door dimensions for cache validation
            x_data = get_x_values(2, default_on_error=True)
            y_data = get_y_values(2, default_on_error=True)
            door_x_len = x_data.get('xlen', 390)
            door_y_len = y_data.get('ylen', 700)
            
            # Run spiral with caching - uses pre-generated path!
            success = run_spiral_with_cache(
                cps=cps,
                config=config,
                zigzag_points=points1,
                door_id=2,
                track_name=f"small_door2_{orientation}",  # Must match pre-generator format!
                orientation=orientation,
                radius=12.0,
                angle_step_deg=45.0,
                velocity=300.0,
                accel=500.0,
                jerk=10000.0,
                use_cache=True,
                force_regenerate=False,
                door_x_length=door_x_len,
                door_y_length=door_y_len,
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
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            print("Force Control Activated")
            
            # Get door dimensions for cache validation
            x_data = get_x_values(3, default_on_error=True)
            y_data = get_y_values(3, default_on_error=True)
            door_x_len = x_data.get('xlen', 390)
            door_y_len = y_data.get('ylen', 700)
            
            # Run spiral with caching - uses pre-generated path!
            success = run_spiral_with_cache(
                cps=cps,
                config=config,
                zigzag_points=points1,
                door_id=3,
                track_name=f"small_door3_{orientation}",  # Must match pre-generator format!
                orientation=orientation,
                radius=12.0,
                angle_step_deg=45.0,
                velocity=300.0,
                accel=500.0,
                jerk=10000.0,
                use_cache=True,
                force_regenerate=False,
                door_x_length=door_x_len,
                door_y_length=door_y_len,
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
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]

        # Collect boundary coordinates
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        #Generate zigzag path
        zigzag_pathp1,prepointp1= generate_zigzag_path(
            x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, 
            innerOffset=17,innerOffsetX=10, 
            orientation=orientation, movement=movement,
            innerSandingOffset=50)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            print("Force Control Activated")
            
            # Get door dimensions for cache validation
            x_data = get_x_values(4, default_on_error=True)
            y_data = get_y_values(4, default_on_error=True)
            door_x_len = x_data.get('xlen', 390)
            door_y_len = y_data.get('ylen', 700)
            
            # Run spiral with caching - uses pre-generated path!
            success = run_spiral_with_cache(
                cps=cps,
                config=config,
                zigzag_points=points1,
                door_id=4,
                track_name=f"small_door4_{orientation}",  # Must match pre-generator format!
                orientation=orientation,
                radius=12.0,
                angle_step_deg=45.0,
                velocity=300.0,
                accel=500.0,
                jerk=10000.0,
                use_cache=True,
                force_regenerate=False,
                door_x_length=door_x_len,
                door_y_length=door_y_len,
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
        communicate(
            cps=cps, config=config, 
            point=prepointp1,
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)
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
