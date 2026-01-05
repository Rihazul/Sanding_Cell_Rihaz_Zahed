# spiral_cache_fast.py
"""
FAST cache for spiral paths using pickle + in-memory cache.
~100x faster than Excel-based caching.

Loading times comparison:
- Excel (openpyxl): 500-1000ms for 1000 points
- Pickle: 5-10ms for 1000 points  
- Memory cache: <0.1ms (instant!)
"""

import os
import pickle
import hashlib
import json
import time
from typing import List, Optional, Tuple, Dict, Any


CACHE_DIR = os.path.join(os.path.dirname(__file__), "spiral_cache")

# Tolerance for dimension comparison (mm)
DIMENSION_TOLERANCE = 2.0

# In-memory cache for INSTANT access after first load
_MEMORY_CACHE: Dict[str, Dict[str, Any]] = {}


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


def get_pickle_path(cache_key: str) -> str:
    """Get full path to pickle cache file."""
    _ensure_cache_dir()
    return os.path.join(CACHE_DIR, f"{cache_key}.pkl")


def save_to_cache(
    cache_key: str,
    points: List[float],
    metadata: dict,
) -> str:
    """
    Save spiral path to both memory and disk (pickle).
    
    Args:
        cache_key: Unique identifier
        points: Flat list of [x, y, z, rx, ry, rz, ...]
        metadata: Dict with door dimensions, params, etc.
    
    Returns:
        Path to saved file
    """
    _ensure_cache_dir()
    
    cache_data = {
        "points": points,
        "metadata": metadata,
        "timestamp": time.time(),
    }
    
    # Save to memory (instant access next time)
    _MEMORY_CACHE[cache_key] = cache_data
    
    # Save to disk (pickle is FAST)
    file_path = get_pickle_path(cache_key)
    with open(file_path, 'wb') as f:
        pickle.dump(cache_data, f, protocol=pickle.HIGHEST_PROTOCOL)
    
    num_points = len(points) // 6
    print(f"[FastCache] Saved {num_points} points to memory + disk: {cache_key}")
    return file_path


def load_from_cache(cache_key: str) -> Tuple[Optional[List[float]], Optional[dict]]:
    """
    Load from memory first, then disk. FAST!
    
    Returns:
        Tuple of (points, metadata) or (None, None) if not found
    """
    # 1. Check memory cache first (INSTANT)
    if cache_key in _MEMORY_CACHE:
        data = _MEMORY_CACHE[cache_key]
        num_points = len(data["points"]) // 6
        print(f"[FastCache] Memory hit: {cache_key} ({num_points} points) - INSTANT")
        return data["points"], data["metadata"]
    
    # 2. Check disk cache (pickle - still fast)
    file_path = get_pickle_path(cache_key)
    if os.path.exists(file_path):
        start = time.time()
        try:
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
            
            # Store in memory for next time
            _MEMORY_CACHE[cache_key] = data
            
            elapsed_ms = (time.time() - start) * 1000
            num_points = len(data["points"]) // 6
            print(f"[FastCache] Disk hit: {cache_key} ({num_points} points) - {elapsed_ms:.1f}ms")
            return data["points"], data["metadata"]
        except Exception as e:
            print(f"[FastCache] Error loading pickle: {e}")
            return None, None
    
    print(f"[FastCache] Cache miss: {cache_key}")
    return None, None


def validate_dimensions(
    metadata: dict,
    current_x: float,
    current_y: float,
) -> Tuple[bool, str]:
    """Check if cached dimensions match current door dimensions."""
    cached_x = metadata.get("door_x_length")
    cached_y = metadata.get("door_y_length")
    
    if cached_x is None or cached_y is None:
        return False, "No dimensions in cache"
    
    x_diff = abs(float(cached_x) - current_x)
    y_diff = abs(float(cached_y) - current_y)
    
    if x_diff > DIMENSION_TOLERANCE:
        return False, f"X mismatch: {cached_x:.1f} vs {current_x:.1f}mm"
    if y_diff > DIMENSION_TOLERANCE:
        return False, f"Y mismatch: {cached_y:.1f} vs {current_y:.1f}mm"
    
    return True, "OK"


def load_with_validation(
    track_name: str,
    door_id: int,
    orientation: str,
    current_x: float,
    current_y: float,
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
) -> Tuple[Optional[List[float]], Optional[dict], bool]:
    """
    Load cache with dimension validation.
    
    Returns:
        (points, metadata, is_valid)
    """
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    points, metadata = load_from_cache(cache_key)
    
    if points is None:
        return None, None, False
    
    is_valid, msg = validate_dimensions(metadata, current_x, current_y)
    if not is_valid:
        print(f"[FastCache] Invalidated: {msg}")
        # Remove from memory cache
        _MEMORY_CACHE.pop(cache_key, None)
        return None, None, False
    
    return points, metadata, True


def save_spiral_track(
    track_name: str,
    door_id: int,
    orientation: str,
    all_points: List[float],
    radius: float = 12.0,
    turns: int = 12,
    angle_step_deg: float = 45.0,
    door_x_length: float = None,
    door_y_length: float = None,
) -> str:
    """
    Save spiral track to cache.
    
    Args:
        track_name: Unique name for this track
        door_id: Door number (1-4)
        orientation: "horizontal" or "vertical"
        all_points: Flat list of all spiral points
        radius, turns, angle_step_deg: Spiral params
        door_x_length, door_y_length: Door dimensions for validation
    """
    cache_key = _generate_cache_key(track_name, door_id, orientation, radius, turns, angle_step_deg)
    
    metadata = {
        "track_name": track_name,
        "door_id": door_id,
        "orientation": orientation,
        "radius": radius,
        "turns": turns,
        "angle_step_deg": angle_step_deg,
        "total_points": len(all_points) // 6,
        "door_x_length": door_x_length,
        "door_y_length": door_y_length,
    }
    
    return save_to_cache(cache_key, all_points, metadata)


def preload_all_caches():
    """
    Load ALL cached paths into memory at startup.
    Call this once when the program starts for instant access.
    """
    if not os.path.exists(CACHE_DIR):
        print("[FastCache] No cache directory found")
        return 0
    
    count = 0
    start = time.time()
    
    for filename in os.listdir(CACHE_DIR):
        if filename.endswith(".pkl"):
            cache_key = filename[:-4]  # Remove .pkl
            file_path = os.path.join(CACHE_DIR, filename)
            try:
                with open(file_path, 'rb') as f:
                    data = pickle.load(f)
                _MEMORY_CACHE[cache_key] = data
                count += 1
            except Exception as e:
                print(f"[FastCache] Failed to preload {filename}: {e}")
    
    elapsed_ms = (time.time() - start) * 1000
    print(f"[FastCache] Preloaded {count} caches in {elapsed_ms:.1f}ms")
    return count


def clear_cache(cache_key: Optional[str] = None):
    """Clear specific cache or all caches (memory + disk)."""
    global _MEMORY_CACHE
    
    if cache_key:
        # Clear specific
        _MEMORY_CACHE.pop(cache_key, None)
        pkl_path = get_pickle_path(cache_key)
        if os.path.exists(pkl_path):
            os.remove(pkl_path)
        print(f"[FastCache] Cleared: {cache_key}")
    else:
        # Clear all
        _MEMORY_CACHE = {}
        if os.path.exists(CACHE_DIR):
            for f in os.listdir(CACHE_DIR):
                if f.endswith(".pkl"):
                    os.remove(os.path.join(CACHE_DIR, f))
        print("[FastCache] Cleared all caches")


def get_cache_stats() -> dict:
    """Get cache statistics."""
    memory_count = len(_MEMORY_CACHE)
    disk_count = 0
    if os.path.exists(CACHE_DIR):
        disk_count = len([f for f in os.listdir(CACHE_DIR) if f.endswith(".pkl")])
    
    return {
        "memory_cached": memory_count,
        "disk_cached": disk_count,
        "cache_dir": CACHE_DIR,
    }


# Backwards compatibility with old module
def _generate_cache_key_compat(*args, **kwargs):
    return _generate_cache_key(*args, **kwargs)

get_cache_path = get_pickle_path
cache_exists = lambda key: os.path.exists(get_pickle_path(key)) or key in _MEMORY_CACHE
