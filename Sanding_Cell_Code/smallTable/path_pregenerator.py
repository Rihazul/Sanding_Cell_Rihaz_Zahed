# path_pregenerator.py
"""
Pre-generate all 8 spiral paths (horizontal + vertical for 4 doors) immediately after scanning.
This eliminates calculation delay during execution - paths are loaded and executed instantly.

Usage:
    1. After scanning: call pregenerate_all_paths()
    2. During execution: call execute_cached_path(cps, config, door_id, orientation)
"""

import os
import sys
import time
import yaml

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from smallTable.scancord import (
    get_inner_corner_point,
    get_door_position,
    get_x_values,
    get_y_values,
)
from smallTable.spiral_cache_fast import (
    save_to_cache,
    load_from_cache,
    _generate_cache_key,
    preload_all_caches,
    get_cache_stats,
    clear_cache,
)
from smallTable.zigzagplane1final import (
    generate_zigzag_path,
    generate_spiral_between_points,
    compute_timeout,
    finalize_spiral_path,
    turn_vibration_on,
    turn_vibration_off,
)
from Server_Better_V2 import putForceZminus, releaseForce


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


# Default spiral parameters (can be overridden)
DEFAULT_PARAMS = {
    "radius": 12.0,
    "turns": 12,
    "angle_step_deg": 45.0,
    "max_points": 80,
    "velocity": 300.0,
    "accel": 500.0,
    "jerk": 10000.0,
    "innerOffset": 17,
    "innerOffsetX": 17,
    "innerSandingOffset": 50,
}


def _calculate_door_points(door_id: int, z: float = -6.5):
    """
    Calculate the zigzag boundary points for a specific door.
    
    Args:
        door_id: Door number (1-4)
        z: Z depth for the pocket
        
    Returns:
        Tuple of (x_coords, y_coords, z_coords, distance, door_position)
    """
    # Get inner corner points for the door
    p8 = get_inner_corner_point(door_id, 0)
    p7 = get_inner_corner_point(door_id, 1)
    p6 = get_inner_corner_point(door_id, 2)
    p5 = get_inner_corner_point(door_id, 3)
    
    # Check if door data exists
    if p8[0] == "null" or p7[0] == "null":
        return None, None, None, None, None
    
    # 7th axis position
    door_position = p8[0] + get_door_position(door_id)
    
    # Calculate distance
    distance = p6[0] - p8[0]
    
    # Points calculation (negating X and using local coordinates)
    point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
    point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
    point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
    point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
    
    # Final points with distance offset
    point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
    point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
    point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
    point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
    
    x_coords = [point5u[0], point6u[0], point7u[0], point8u[0]]
    y_coords = [point5u[1], point6u[1], point7u[1], point8u[1]]
    z_coords = [point5u[2], point6u[2], point7u[2], point8u[2]]
    
    return x_coords, y_coords, z_coords, distance, door_position


def generate_door_spiral_path(
    door_id: int,
    orientation: str = "horizontal",
    z: float = -6.5,
    params: dict = None,
) -> tuple:
    """
    Generate complete spiral path for a door in specified orientation.
    
    Args:
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        z: Z depth
        params: Override default parameters
        
    Returns:
        Tuple of (all_points, prepoint, door_x_len, door_y_len, door_position)
    """
    p = {**DEFAULT_PARAMS, **(params or {})}
    
    # Calculate door boundary points
    x_coords, y_coords, z_coords, distance, door_position = _calculate_door_points(door_id, z)
    
    if x_coords is None:
        print(f"[PreGen] Door {door_id}: No scan data available")
        return None, None, None, None, None
    
    # Get door dimensions for cache validation
    x_data = get_x_values(door_id, default_on_error=True)
    y_data = get_y_values(door_id, default_on_error=True)
    door_x_len = x_data.get('xlen', 390)
    door_y_len = y_data.get('ylen', 700)
    
    # Generate zigzag waypoints
    zigzag_path, prepoint = generate_zigzag_path(
        x_coords=x_coords,
        y_coords=y_coords,
        z_coords=z_coords,
        innerOffset=p["innerOffset"],
        innerOffsetX=p["innerOffsetX"],
        orientation=orientation,
        movement="zigzag",
        innerSandingOffset=p["innerSandingOffset"],
    )
    
    if not zigzag_path:
        print(f"[PreGen] Door {door_id} {orientation}: No zigzag path generated")
        return None, None, None, None, None
    
    # Generate spiral points between each zigzag waypoint
    all_points = []
    for i in range(len(zigzag_path) - 1):
        start_pose = zigzag_path[i]
        end_pose = zigzag_path[i + 1]
        
        segment_points = generate_spiral_between_points(
            start_pose=start_pose,
            end_pose=end_pose,
            turns=p["turns"],
            radius=p["radius"],
            angle_step_deg=p["angle_step_deg"],
            max_points=p["max_points"],
            orientation=orientation,
        )
        all_points.extend(segment_points)
    
    total_count = len(all_points) // 6
    print(f"[PreGen] Door {door_id} {orientation}: Generated {total_count} points")
    
    return all_points, prepoint, door_x_len, door_y_len, door_position


def pregenerate_all_paths(
    z: float = -6.5,
    params: dict = None,
    doors: list = None,
) -> dict:
    """
    Pre-generate all 8 spiral paths (2 orientations × 4 doors) after scanning.
    Call this immediately after save_scan_results_to_json().
    
    Args:
        z: Z depth for pockets
        params: Override default spiral parameters
        doors: List of door IDs to generate (default: [1, 2, 3, 4])
        
    Returns:
        Dict with generation results and stats
    """
    p = {**DEFAULT_PARAMS, **(params or {})}
    doors = doors or [1, 2, 3, 4]
    orientations = ["horizontal", "vertical"]
    
    results = {
        "success": [],
        "failed": [],
        "total_points": 0,
        "generation_time_ms": 0,
    }
    
    start_time = time.time()
    
    for door_id in doors:
        for orientation in orientations:
            track_name = f"small_door{door_id}_{orientation}"
            cache_key = _generate_cache_key(
                track_name, door_id, orientation,
                p["radius"], p["turns"], p["angle_step_deg"]
            )
            
            # Generate the path
            all_points, prepoint, door_x_len, door_y_len, door_position = generate_door_spiral_path(
                door_id=door_id,
                orientation=orientation,
                z=z,
                params=p,
            )
            
            if all_points is None:
                results["failed"].append(f"door{door_id}_{orientation}")
                continue
            
            # Save to cache with metadata
            metadata = {
                "track_name": track_name,
                "door_id": door_id,
                "orientation": orientation,
                "radius": p["radius"],
                "turns": p["turns"],
                "angle_step_deg": p["angle_step_deg"],
                "total_points": len(all_points) // 6,
                "door_x_length": door_x_len,
                "door_y_length": door_y_len,
                "door_position": door_position,
                "prepoint": prepoint,  # Store prepoint for motion setup
            }
            
            save_to_cache(cache_key, all_points, metadata)
            
            results["success"].append(f"door{door_id}_{orientation}")
            results["total_points"] += len(all_points) // 6
    
    results["generation_time_ms"] = (time.time() - start_time) * 1000
    
    print(f"\n[PreGen] ========== SUMMARY ==========")
    print(f"[PreGen] Success: {len(results['success'])}/8 paths")
    print(f"[PreGen] Failed: {results['failed']}")
    print(f"[PreGen] Total points: {results['total_points']}")
    print(f"[PreGen] Time: {results['generation_time_ms']:.1f}ms")
    print(f"[PreGen] ================================\n")
    
    return results


def load_cached_path(
    door_id: int,
    orientation: str = "horizontal",
    params: dict = None,
) -> tuple:
    """
    Load a pre-generated path from cache.
    
    Args:
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        params: Spiral parameters (must match what was used for generation)
        
    Returns:
        Tuple of (all_points, metadata) or (None, None) if not found
    """
    p = {**DEFAULT_PARAMS, **(params or {})}
    track_name = f"small_door{door_id}_{orientation}"
    cache_key = _generate_cache_key(
        track_name, door_id, orientation,
        p["radius"], p["turns"], p["angle_step_deg"]
    )
    
    return load_from_cache(cache_key)


def execute_cached_path(
    cps,
    config: dict,
    door_id: int,
    orientation: str = "horizontal",
    force: float = 3.0,
    params: dict = None,
) -> bool:
    """
    Load and execute a pre-generated spiral path from cache.
    NO calculation needed - just load and run!
    
    Args:
        cps: Robot CPS client
        config: Configuration dict
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        force: Force control value
        params: Spiral parameters (must match generation)
        
    Returns:
        True if successful
    """
    p = {**DEFAULT_PARAMS, **(params or {})}
    
    # Load from cache
    all_points, metadata = load_cached_path(door_id, orientation, params)
    
    if all_points is None:
        print(f"[Execute] ERROR: No cached path for door{door_id}_{orientation}")
        print("[Execute] Run pregenerate_all_paths() first!")
        return False
    
    track_name = metadata.get("track_name", f"small_door{door_id}_{orientation}")
    total_count = len(all_points) // 6
    door_position = metadata.get("door_position")
    prepoint = metadata.get("prepoint")
    
    print(f"[Execute] Loaded {total_count} points from cache - INSTANT!")
    
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
        print(f"[Execute] InitMovePathL failed: {ret}")
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
        print(f"[Execute] PushMovePaths failed: {ret}")
        return False
    
    print(f"[Execute] Pushed {total_count} points to robot controller")
    
    # Activate force control
    putForceZminus(
        cps=cps,
        force=force,
        tcp=tcp_name,
        ucs=ucs_name,
        config=config
    )
    print("[Execute] Force control activated")
    
    # Finalize and execute
    timeout = compute_timeout(total_points=total_count, velocity=p["velocity"])
    success = finalize_spiral_path(cps, track_name, completion_timeout=timeout)
    
    # Cleanup
    turn_vibration_off(cps)
    releaseForce(cps=cps, config=config)
    
    return success


def get_prepoint_for_door(door_id: int, orientation: str = "horizontal", params: dict = None):
    """
    Get the prepoint (approach position) for a door from cached metadata.
    Use this for motion setup before executing the cached path.
    """
    _, metadata = load_cached_path(door_id, orientation, params)
    if metadata:
        return metadata.get("prepoint"), metadata.get("door_position")
    return None, None


# ============================================================
# Integration with scanning workflow
# ============================================================

def hook_after_scan(scan_data: dict, z: float = -6.5):
    """
    Call this hook immediately after save_scan_results_to_json().
    Automatically pre-generates all paths.
    
    Usage in scansmalltable.py:
        from smallTable.path_pregenerator import hook_after_scan
        
        json_file_path = save_scan_results_to_json(scan_data)
        hook_after_scan(scan_data)  # <-- Add this line!
    """
    print("\n[PreGen] Auto-generating all spiral paths after scan...")
    return pregenerate_all_paths(z=z)


if __name__ == "__main__":
    # Test: Pre-generate all paths (assuming scan data exists)
    print("Testing path pre-generation...")
    print(f"Cache stats before: {get_cache_stats()}")
    
    results = pregenerate_all_paths(z=-6.5)
    
    print(f"Cache stats after: {get_cache_stats()}")
    
    # Test loading
    for door in [1, 2, 3, 4]:
        for orient in ["horizontal", "vertical"]:
            points, meta = load_cached_path(door, orient)
            if points:
                print(f"  Door {door} {orient}: {len(points)//6} points ready")
            else:
                print(f"  Door {door} {orient}: NOT CACHED")
