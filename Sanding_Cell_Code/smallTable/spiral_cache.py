# spiral_cache.py
"""
Cache spiral paths to Excel to avoid recalculating during robot operation.
Includes door dimension validation to ensure cached paths match current door size.
"""

import os
import pandas as pd
import hashlib
import json
from typing import List, Optional, Tuple


CACHE_DIR = os.path.join(os.path.dirname(__file__), "spiral_cache")

# Tolerance for dimension comparison (mm)
DIMENSION_TOLERANCE = 2.0


def _ensure_cache_dir():
    """Create cache directory if it doesn't exist."""
    if not os.path.exists(CACHE_DIR):
        os.makedirs(CACHE_DIR)


def _generate_cache_key(
    track_name: str,
    door_id: int,
    orientation: str,
    radius: float,
    turns: int,
    angle_step_deg: float,
) -> str:
    """Generate a unique cache key based on parameters."""
    params = {
        "track": track_name,
        "door": door_id,
        "orientation": orientation,
        "radius": radius,
        "turns": turns,
        "angle_step": angle_step_deg,
    }
    param_str = json.dumps(params, sort_keys=True)
    hash_suffix = hashlib.md5(param_str.encode()).hexdigest()[:8]
    return f"{track_name}_door{door_id}_{orientation}_{hash_suffix}"


def get_cache_path(cache_key: str) -> str:
    """Get full path to cache file."""
    _ensure_cache_dir()
    return os.path.join(CACHE_DIR, f"{cache_key}.xlsx")


def save_spiral_path_to_excel(
    points: List[float],
    cache_key: str,
    metadata: Optional[dict] = None,
) -> str:
    """
    Save spiral path points to Excel file.
    
    Args:
        points: Flat list of [x, y, z, rx, ry, rz, x, y, z, rx, ry, rz, ...]
        cache_key: Unique identifier for this path
        metadata: Optional metadata to save in a separate sheet
    
    Returns:
        Path to saved Excel file
    """
    _ensure_cache_dir()
    
    # Convert flat list to rows of 6 values each
    num_points = len(points) // 6
    rows = []
    for i in range(num_points):
        idx = i * 6
        rows.append({
            "point_index": i,
            "X": points[idx],
            "Y": points[idx + 1],
            "Z": points[idx + 2],
            "Rx": points[idx + 3],
            "Ry": points[idx + 4],
            "Rz": points[idx + 5],
        })
    
    df = pd.DataFrame(rows)
    file_path = get_cache_path(cache_key)
    
    with pd.ExcelWriter(file_path, engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="SpiralPoints", index=False)
        
        # Save metadata if provided
        if metadata:
            meta_df = pd.DataFrame([metadata])
            meta_df.to_excel(writer, sheet_name="Metadata", index=False)
    
    print(f"[SpiralCache] Saved {num_points} points to: {file_path}")
    return file_path


def load_spiral_path_from_excel(cache_key: str) -> Tuple[Optional[List[float]], Optional[dict]]:
    """
    Load spiral path points from Excel file.
    
    Args:
        cache_key: Unique identifier for this path
    
    Returns:
        Tuple of (flat points list, metadata dict) or (None, None) if not found
    """
    file_path = get_cache_path(cache_key)
    
    if not os.path.exists(file_path):
        print(f"[SpiralCache] Cache miss: {cache_key}")
        return None, None
    
    try:
        df = pd.read_excel(file_path, sheet_name="SpiralPoints", engine="openpyxl")
        
        # Convert back to flat list
        points = []
        for _, row in df.iterrows():
            points.extend([
                row["X"], row["Y"], row["Z"],
                row["Rx"], row["Ry"], row["Rz"]
            ])
        
        # Load metadata if exists
        metadata = None
        try:
            meta_df = pd.read_excel(file_path, sheet_name="Metadata", engine="openpyxl")
            if not meta_df.empty:
                metadata = meta_df.iloc[0].to_dict()
        except Exception:
            pass
        
        print(f"[SpiralCache] Cache hit: {cache_key} ({len(points)//6} points)")
        return points, metadata
        
    except Exception as e:
        print(f"[SpiralCache] Error loading cache: {e}")
        return None, None


def cache_exists(cache_key: str) -> bool:
    """Check if cache file exists."""
    return os.path.exists(get_cache_path(cache_key))


def clear_cache(cache_key: Optional[str] = None):
    """Clear specific cache or all caches."""
    if cache_key:
        file_path = get_cache_path(cache_key)
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"[SpiralCache] Cleared: {cache_key}")
    else:
        # Clear all
        if os.path.exists(CACHE_DIR):
            for f in os.listdir(CACHE_DIR):
                if f.endswith(".xlsx"):
                    os.remove(os.path.join(CACHE_DIR, f))
            print("[SpiralCache] Cleared all caches")


def save_full_track_to_excel(
    track_name: str,
    door_id: int,
    orientation: str,
    all_segments: List[Tuple[List[float], List[float], List[float]]],
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
    door_x_length: Optional[float] = None,
    door_y_length: Optional[float] = None,
) -> str:
    """
    Save entire track (all segments concatenated) to a single Excel file.
    
    Args:
        track_name: Name of the track
        door_id: Door identifier
        orientation: "horizontal" or "vertical"
        all_segments: List of (start_pose, end_pose, spiral_points) tuples
        radius, turns, angle_step_deg: Spiral parameters for metadata
        door_x_length: Door X dimension (for validation on load)
        door_y_length: Door Y dimension (for validation on load)
    
    Returns:
        Path to saved Excel file
    """
    _ensure_cache_dir()
    
    rows = []
    segment_idx = 0
    
    for start_pose, end_pose, points in all_segments:
        num_points = len(points) // 6
        for i in range(num_points):
            idx = i * 6
            rows.append({
                "segment": segment_idx,
                "point_index": i,
                "X": points[idx],
                "Y": points[idx + 1],
                "Z": points[idx + 2],
                "Rx": points[idx + 3],
                "Ry": points[idx + 4],
                "Rz": points[idx + 5],
            })
        segment_idx += 1
    
    df = pd.DataFrame(rows)
    
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    file_path = get_cache_path(cache_key)
    
    metadata = {
        "track_name": track_name,
        "door_id": door_id,
        "orientation": orientation,
        "radius": radius,
        "turns": turns,
        "angle_step_deg": angle_step_deg,
        "total_segments": segment_idx,
        "total_points": len(rows),
        "door_x_length": door_x_length,
        "door_y_length": door_y_length,
    }
    
    with pd.ExcelWriter(file_path, engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="SpiralPoints", index=False)
        meta_df = pd.DataFrame([metadata])
        meta_df.to_excel(writer, sheet_name="Metadata", index=False)
    
    print(f"[SpiralCache] Saved full track: {track_name} ({len(rows)} points, {segment_idx} segments)")
    print(f"[SpiralCache] Door dimensions stored: X={door_x_length}mm, Y={door_y_length}mm")
    return file_path


def validate_cache_dimensions(
    metadata: dict,
    current_x_length: float,
    current_y_length: float,
) -> Tuple[bool, str]:
    """
    Validate that cached door dimensions match current door dimensions.
    
    Args:
        metadata: Cached metadata dict
        current_x_length: Current door X dimension
        current_y_length: Current door Y dimension
    
    Returns:
        Tuple of (is_valid, message)
    """
    cached_x = metadata.get("door_x_length")
    cached_y = metadata.get("door_y_length")
    
    # If no dimensions stored, can't validate
    if cached_x is None or cached_y is None:
        return False, "Cache missing door dimensions - regenerating"
    
    # Check X dimension
    x_diff = abs(float(cached_x) - current_x_length)
    if x_diff > DIMENSION_TOLERANCE:
        return False, f"X-length mismatch: cached={cached_x:.1f}mm, current={current_x_length:.1f}mm (diff={x_diff:.1f}mm)"
    
    # Check Y dimension
    y_diff = abs(float(cached_y) - current_y_length)
    if y_diff > DIMENSION_TOLERANCE:
        return False, f"Y-length mismatch: cached={cached_y:.1f}mm, current={current_y_length:.1f}mm (diff={y_diff:.1f}mm)"
    
    return True, f"Dimensions match (X={current_x_length:.1f}mm, Y={current_y_length:.1f}mm)"


def load_full_track_with_validation(
    track_name: str,
    door_id: int,
    orientation: str,
    current_x_length: float,
    current_y_length: float,
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
) -> Tuple[Optional[List[float]], Optional[dict], bool]:
    """
    Load entire track from Excel, validating door dimensions match.
    
    Args:
        track_name: Name of the track
        door_id: Door identifier
        orientation: "horizontal" or "vertical"
        current_x_length: Current door X dimension from scan
        current_y_length: Current door Y dimension from scan
        radius, turns, angle_step_deg: Spiral parameters
    
    Returns:
        Tuple of (points, metadata, is_valid)
        - If dimensions don't match, returns (None, None, False)
    """
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    points, metadata = load_spiral_path_from_excel(cache_key)
    
    if points is None or metadata is None:
        return None, None, False
    
    # Validate dimensions
    is_valid, message = validate_cache_dimensions(metadata, current_x_length, current_y_length)
    print(f"[SpiralCache] Validation: {message}")
    
    if not is_valid:
        print(f"[SpiralCache] Cache invalidated - will regenerate")
        return None, None, False
    
    return points, metadata, True


def load_full_track_from_excel(
    track_name: str,
    door_id: int,
    orientation: str,
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
) -> Tuple[Optional[List[float]], Optional[dict]]:
    """
    Load entire track from Excel as a flat list of points.
    
    Returns:
        Tuple of (flat points list, metadata) or (None, None) if not found
    """
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    return load_spiral_path_from_excel(cache_key)
