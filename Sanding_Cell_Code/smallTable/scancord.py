"""Module summary: load static/scan_results.json and expose door geometry.
Data schema: doors 1-4, corners 0-3; keys 7thx#, fr#doorn, in#doorn, out#doorn, pocket#doorn; xVals/yVals are per-door dicts.
read_scan_results(default_on_error=False): parse JSON, build dictionaries for positions/frame/inner/outer/pocket points, pad missing slots with "null"; return None or get_default_values() on errors.
get_default_values(): fabricate a fully-populated null dataset matching read_scan_results output.
print_door_info(): CLI printer for positions/frame/inner/outer/pocket/xVals/yVals with defaults if missing.
get_all_scan_data(default_on_error=False): wrapper returning the combined scan dictionary.
get_door_position(door_number,...): fetch 7thx{n} with optional default handling.
get_frame_point(door_number, corner_number,...): fetch fr{door}door{corner} or default list.
get_inner_corner_point(...): fetch in{door}door{corner} or default list.
get_outer_corner_point(...): fetch out{door}door{corner} or default list.
get_pocket_point(...): fetch pocket{door}door{corner} or default list.
get_x_values(door_number,...): fetch xVals entry (xlen, xframe_1, xframe_2) or default mapping.
get_y_values(door_number,...): fetch yVals entry (ylen, yframe_1, yframe_2) or default mapping.
Notes: file path resolved relative to this module; logger at INFO; helpers share read_scan_results behavior; default_on_error=True is safest for UI."""

import os
import json
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def read_scan_results(default_on_error=False):
    """
    Reads scan_results.json and extracts the door positions from robo7thPos,
    the coordinates from framePoints, innerCornerPoints, outerCornerPoints,
    and pocketPoints for all 4 doors.

    Args:
        default_on_error (bool): If True, returns default values when file is missing or invalid

    Returns:
        dict: Dictionary containing all the extracted data or default values if specified
    """
    # Define path to the scan_results.json file
    json_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "static", "scan_results.json"
    )

    try:
        # Read the JSON file
        with open(json_path, "r") as file:
            data = json.load(file)

        def _ensure_list(value):
            return value if isinstance(value, list) else []

        def _ensure_dict(value):
            return value if isinstance(value, dict) else {}

        # Extract 7th axis positions for each door
        robo7thPos = _ensure_list(data.get("robo7thPos"))
        door_positions = {
            "7thx1": robo7thPos[0] if len(robo7thPos) > 0 else "null",
            "7thx2": robo7thPos[1] if len(robo7thPos) > 1 else "null",
            "7thx3": robo7thPos[2] if len(robo7thPos) > 2 else "null",
            "7thx4": robo7thPos[3] if len(robo7thPos) > 3 else "null",
        }

        # Extract frame points for each door
        frame_points = {}
        framePoints_data = _ensure_list(data.get("framePoints"))

        # Door 1 (index 0)
        if len(framePoints_data) > 0:
            door1_data = _ensure_dict(framePoints_data[0])
            frame_points["fr1door0"] = door1_data.get("0", ["null", "null", "null"])
            frame_points["fr1door1"] = door1_data.get("1", ["null", "null", "null"])
            frame_points["fr1door2"] = door1_data.get("2", ["null", "null", "null"])
            frame_points["fr1door3"] = door1_data.get("3", ["null", "null", "null"])
        else:
            for corner in range(4):
                frame_points[f"fr1door{corner}"] = ["null", "null", "null"]

        # Door 2 (index 1)
        if len(framePoints_data) > 1:
            door2_data = _ensure_dict(framePoints_data[1])
            frame_points["fr2door0"] = door2_data.get("0", ["null", "null", "null"])
            frame_points["fr2door1"] = door2_data.get("1", ["null", "null", "null"])
            frame_points["fr2door2"] = door2_data.get("2", ["null", "null", "null"])
            frame_points["fr2door3"] = door2_data.get("3", ["null", "null", "null"])
        else:
            for corner in range(4):
                frame_points[f"fr2door{corner}"] = ["null", "null", "null"]

        # Door 3 (index 2)
        if len(framePoints_data) > 2:
            door3_data = _ensure_dict(framePoints_data[2])
            frame_points["fr3door0"] = door3_data.get("0", ["null", "null", "null"])
            frame_points["fr3door1"] = door3_data.get("1", ["null", "null", "null"])
            frame_points["fr3door2"] = door3_data.get("2", ["null", "null", "null"])
            frame_points["fr3door3"] = door3_data.get("3", ["null", "null", "null"])
        else:
            for corner in range(4):
                frame_points[f"fr3door{corner}"] = ["null", "null", "null"]

        # Door 4 (index 3)
        if len(framePoints_data) > 3:
            door4_data = _ensure_dict(framePoints_data[3])
            frame_points["fr4door0"] = door4_data.get("0", ["null", "null", "null"])
            frame_points["fr4door1"] = door4_data.get("1", ["null", "null", "null"])
            frame_points["fr4door2"] = door4_data.get("2", ["null", "null", "null"])
            frame_points["fr4door3"] = door4_data.get("3", ["null", "null", "null"])
        else:
            for corner in range(4):
                frame_points[f"fr4door{corner}"] = ["null", "null", "null"]

        # Extract inner corner points for each door
        inner_corner_points = {}
        innerCornerPoints_data = _ensure_list(data.get("innerCornerPoints"))

        # Door 1 (index 0)
        if len(innerCornerPoints_data) > 0:
            door1_inner_data = _ensure_dict(innerCornerPoints_data[0])
            inner_corner_points["in1door0"] = door1_inner_data.get(
                "0", ["null", "null", "null"]
            )
            inner_corner_points["in1door1"] = door1_inner_data.get(
                "1", ["null", "null", "null"]
            )
            inner_corner_points["in1door2"] = door1_inner_data.get(
                "2", ["null", "null", "null"]
            )
            inner_corner_points["in1door3"] = door1_inner_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                inner_corner_points[f"in1door{corner}"] = ["null", "null", "null"]

        # Door 2 (index 1)
        if len(innerCornerPoints_data) > 1:
            door2_inner_data = _ensure_dict(innerCornerPoints_data[1])
            inner_corner_points["in2door0"] = door2_inner_data.get(
                "0", ["null", "null", "null"]
            )
            inner_corner_points["in2door1"] = door2_inner_data.get(
                "1", ["null", "null", "null"]
            )
            inner_corner_points["in2door2"] = door2_inner_data.get(
                "2", ["null", "null", "null"]
            )
            inner_corner_points["in2door3"] = door2_inner_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                inner_corner_points[f"in2door{corner}"] = ["null", "null", "null"]

        # Door 3 (index 2)
        if len(innerCornerPoints_data) > 2:
            door3_inner_data = _ensure_dict(innerCornerPoints_data[2])
            inner_corner_points["in3door0"] = door3_inner_data.get(
                "0", ["null", "null", "null"]
            )
            inner_corner_points["in3door1"] = door3_inner_data.get(
                "1", ["null", "null", "null"]
            )
            inner_corner_points["in3door2"] = door3_inner_data.get(
                "2", ["null", "null", "null"]
            )
            inner_corner_points["in3door3"] = door3_inner_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                inner_corner_points[f"in3door{corner}"] = ["null", "null", "null"]

        # Door 4 (index 3)
        if len(innerCornerPoints_data) > 3:
            door4_inner_data = _ensure_dict(innerCornerPoints_data[3])
            inner_corner_points["in4door0"] = door4_inner_data.get(
                "0", ["null", "null", "null"]
            )
            inner_corner_points["in4door1"] = door4_inner_data.get(
                "1", ["null", "null", "null"]
            )
            inner_corner_points["in4door2"] = door4_inner_data.get(
                "2", ["null", "null", "null"]
            )
            inner_corner_points["in4door3"] = door4_inner_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                inner_corner_points[f"in4door{corner}"] = ["null", "null", "null"]

        # Extract outer corner points for each door
        outer_corner_points = {}
        outerCornerPoints_data = _ensure_list(data.get("outerCornerPoints"))

        # Door 1 (index 0)
        if len(outerCornerPoints_data) > 0:
            door1_outer_data = _ensure_dict(outerCornerPoints_data[0])
            outer_corner_points["out1door0"] = door1_outer_data.get(
                "0", ["null", "null", "null"]
            )
            outer_corner_points["out1door1"] = door1_outer_data.get(
                "1", ["null", "null", "null"]
            )
            outer_corner_points["out1door2"] = door1_outer_data.get(
                "2", ["null", "null", "null"]
            )
            outer_corner_points["out1door3"] = door1_outer_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                outer_corner_points[f"out1door{corner}"] = ["null", "null", "null"]

        # Door 2 (index 1)
        if len(outerCornerPoints_data) > 1:
            door2_outer_data = _ensure_dict(outerCornerPoints_data[1])
            outer_corner_points["out2door0"] = door2_outer_data.get(
                "0", ["null", "null", "null"]
            )
            outer_corner_points["out2door1"] = door2_outer_data.get(
                "1", ["null", "null", "null"]
            )
            outer_corner_points["out2door2"] = door2_outer_data.get(
                "2", ["null", "null", "null"]
            )
            outer_corner_points["out2door3"] = door2_outer_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                outer_corner_points[f"out2door{corner}"] = ["null", "null", "null"]

        # Door 3 (index 2)
        if len(outerCornerPoints_data) > 2:
            door3_outer_data = _ensure_dict(outerCornerPoints_data[2])
            outer_corner_points["out3door0"] = door3_outer_data.get(
                "0", ["null", "null", "null"]
            )
            outer_corner_points["out3door1"] = door3_outer_data.get(
                "1", ["null", "null", "null"]
            )
            outer_corner_points["out3door2"] = door3_outer_data.get(
                "2", ["null", "null", "null"]
            )
            outer_corner_points["out3door3"] = door3_outer_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                outer_corner_points[f"out3door{corner}"] = ["null", "null", "null"]

        # Door 4 (index 3)
        if len(outerCornerPoints_data) > 3:
            door4_outer_data = _ensure_dict(outerCornerPoints_data[3])
            outer_corner_points["out4door0"] = door4_outer_data.get(
                "0", ["null", "null", "null"]
            )
            outer_corner_points["out4door1"] = door4_outer_data.get(
                "1", ["null", "null", "null"]
            )
            outer_corner_points["out4door2"] = door4_outer_data.get(
                "2", ["null", "null", "null"]
            )
            outer_corner_points["out4door3"] = door4_outer_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                outer_corner_points[f"out4door{corner}"] = ["null", "null", "null"]

        # Extract pocket points for each door
        pocket_points = {}
        pocketPoints_data = _ensure_list(data.get("pocketPoints"))

        # Door 1 (index 0)
        if len(pocketPoints_data) > 0:
            door1_pocket_data = _ensure_dict(pocketPoints_data[0])
            pocket_points["pocket1door0"] = door1_pocket_data.get(
                "0", ["null", "null", "null"]
            )
            pocket_points["pocket1door1"] = door1_pocket_data.get(
                "1", ["null", "null", "null"]
            )
            pocket_points["pocket1door2"] = door1_pocket_data.get(
                "2", ["null", "null", "null"]
            )
            pocket_points["pocket1door3"] = door1_pocket_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                pocket_points[f"pocket1door{corner}"] = ["null", "null", "null"]

        # Door 2 (index 1)
        if len(pocketPoints_data) > 1:
            door2_pocket_data = _ensure_dict(pocketPoints_data[1])
            pocket_points["pocket2door0"] = door2_pocket_data.get(
                "0", ["null", "null", "null"]
            )
            pocket_points["pocket2door1"] = door2_pocket_data.get(
                "1", ["null", "null", "null"]
            )
            pocket_points["pocket2door2"] = door2_pocket_data.get(
                "2", ["null", "null", "null"]
            )
            pocket_points["pocket2door3"] = door2_pocket_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                pocket_points[f"pocket2door{corner}"] = ["null", "null", "null"]

        # Door 3 (index 2)
        if len(pocketPoints_data) > 2:
            door3_pocket_data = _ensure_dict(pocketPoints_data[2])
            pocket_points["pocket3door0"] = door3_pocket_data.get(
                "0", ["null", "null", "null"]
            )
            pocket_points["pocket3door1"] = door3_pocket_data.get(
                "1", ["null", "null", "null"]
            )
            pocket_points["pocket3door2"] = door3_pocket_data.get(
                "2", ["null", "null", "null"]
            )
            pocket_points["pocket3door3"] = door3_pocket_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                pocket_points[f"pocket3door{corner}"] = ["null", "null", "null"]

        # Door 4 (index 3)
        if len(pocketPoints_data) > 3:
            door4_pocket_data = _ensure_dict(pocketPoints_data[3])
            pocket_points["pocket4door0"] = door4_pocket_data.get(
                "0", ["null", "null", "null"]
            )
            pocket_points["pocket4door1"] = door4_pocket_data.get(
                "1", ["null", "null", "null"]
            )
            pocket_points["pocket4door2"] = door4_pocket_data.get(
                "2", ["null", "null", "null"]
            )
            pocket_points["pocket4door3"] = door4_pocket_data.get(
                "3", ["null", "null", "null"]
            )
        else:
            for corner in range(4):
                pocket_points[f"pocket4door{corner}"] = ["null", "null", "null"]

        # Process x and y values for each door
        x_vals = _ensure_list(data.get("xVals"))
        y_vals = _ensure_list(data.get("yVals"))
        door_profiles = _ensure_list(data.get("doorProfiles"))

        # Normalize xVals entries and ensure 4 entries
        for idx, entry in enumerate(x_vals):
            if not isinstance(entry, dict):
                x_vals[idx] = {"xlen": "null", "xframe_1": "null", "xframe_2": "null"}
        while len(x_vals) < 4:
            x_vals.append({"xlen": "null", "xframe_1": "null", "xframe_2": "null"})

        # Normalize yVals entries and ensure 4 entries
        for idx, entry in enumerate(y_vals):
            if not isinstance(entry, dict):
                y_vals[idx] = {"ylen": "null", "yframe_1": "null", "yframe_2": "null"}
        while len(y_vals) < 4:
            y_vals.append({"ylen": "null", "yframe_1": "null", "yframe_2": "null"})
        if not isinstance(door_profiles, list):
            door_profiles = []

        # Combine all data into one result dictionary
        result = {
            **door_positions,
            **frame_points,
            **inner_corner_points,
            **outer_corner_points,
            **pocket_points,
            "timestamp": data.get("timestamp", ""),
            "xVals": x_vals,
            "yVals": y_vals,
            "doorProfiles": door_profiles,
        }

        return result

    except FileNotFoundError:
        logger.error(f"Error: scan_results.json not found at {json_path}")
        if default_on_error:
            return get_default_values()
        return None
    except json.JSONDecodeError:
        logger.error(f"Error: Invalid JSON format in scan_results.json")
        if default_on_error:
            return get_default_values()
        return None
    except Exception as e:
        logger.error(f"Unexpected error reading scan_results.json: {str(e)}")
        if default_on_error:
            return get_default_values()
        return None


def get_default_values():
    """
    Returns default values for when the scan_results.json file is missing or invalid

    Returns:
        dict: Dictionary containing default values
    """
    # Default values for 7th axis positions
    door_positions = {
        "7thx1": "null",
        "7thx2": "null",
        "7thx3": "null",
        "7thx4": "null",
    }

    # Default values for frame points (empty lists)
    frame_points = {}
    for door in range(1, 5):
        for corner in range(4):
            frame_points[f"fr{door}door{corner}"] = ["null", "null", "null"]

    # Default values for inner corner points (empty lists)
    inner_corner_points = {}
    for door in range(1, 5):
        for corner in range(4):
            inner_corner_points[f"in{door}door{corner}"] = ["null", "null", "null"]

    # Default values for outer corner points (empty lists)
    outer_corner_points = {}
    for door in range(1, 5):
        for corner in range(4):
            outer_corner_points[f"out{door}door{corner}"] = ["null", "null", "null"]

    # Default values for pocket points (empty lists)
    pocket_points = {}
    for door in range(1, 5):
        for corner in range(4):
            pocket_points[f"pocket{door}door{corner}"] = ["null", "null", "null"]

    # Default x and y values
    x_vals = []
    y_vals = []
    for _ in range(4):
        x_vals.append({"xlen": "null", "xframe_1": "null", "xframe_2": "null"})
        y_vals.append({"ylen": "null", "yframe_1": "null", "yframe_2": "null"})
    door_profiles = []
    for door in range(1, 5):
        door_profiles.append(
            {
                "doorNumber": door,
                "profile": "unknown",
                "xProfileType": "unknown",
                "yProfileType": "unknown",
                "xPocketDetected": False,
                "yPocketDetected": False,
            }
        )

    # Combine all default values into one result dictionary
    result = {
        **door_positions,
        **frame_points,
        **inner_corner_points,
        **outer_corner_points,
        **pocket_points,
        "timestamp": "",
        "xVals": x_vals,
        "yVals": y_vals,
        "doorProfiles": door_profiles,
    }

    return result


def print_door_info():
    """
    Prints the door positions, frame points, inner corner points, and outer corner points for all doors
    """
    scan_data = read_scan_results()
    if not scan_data:
        print("No scan data available. Using default values for display.")
        scan_data = get_default_values()

    # Print door positions (7th axis)
    print("===== Door Positions (7th Axis) =====")
    for i in range(1, 5):
        key = f"7thx{i}"
        if key in scan_data:
            print(f"{key}: {scan_data[key]}")

    # Print frame points for each door
    print("\n===== Frame Points =====")
    for door in range(1, 5):
        print(f"\nDoor {door} Frame Points:")
        for corner in range(4):
            key = f"fr{door}door{corner}"
            if key in scan_data:
                print(f"{key}: {scan_data[key]}")

    # Print inner corner points for each door
    print("\n===== Inner Corner Points =====")
    for door in range(1, 5):
        print(f"\nDoor {door} Inner Corner Points:")
        for corner in range(4):
            key = f"in{door}door{corner}"
            if key in scan_data:
                print(f"{key}: {scan_data[key]}")

    # Print outer corner points for each door
    print("\n===== Outer Corner Points =====")
    for door in range(1, 5):
        print(f"\nDoor {door} Outer Corner Points:")
        for corner in range(4):
            key = f"out{door}door{corner}"
            if key in scan_data:
                print(f"{key}: {scan_data[key]}")

    # Print pocket points for each door
    print("\n===== Pocket Points =====")
    for door in range(1, 5):
        print(f"\nDoor {door} Pocket Points:")
        for corner in range(4):
            key = f"pocket{door}door{corner}"
            if key in scan_data:
                print(f"{key}: {scan_data[key]}")

    # Print xVals data for each door
    print("\n===== X Values =====")
    x_vals = scan_data.get("xVals", [])
    if x_vals:
        for i, door_data in enumerate(x_vals, 1):
            print(f"\nDoor {i} X Values:")
            print(f"door{i}xlen: {door_data.get('xlen', 'N/A')}")
            print(f"door{i}xframe_1: {door_data.get('xframe_1', 'N/A')}")
            print(f"door{i}xframe_2: {door_data.get('xframe_2', 'N/A')}")

    # Print yVals data for each door
    print("\n===== Y Values =====")
    y_vals = scan_data.get("yVals", [])
    if y_vals:
        for i, door_data in enumerate(y_vals, 1):
            print(f"\nDoor {i} Y Values:")
            print(f"door{i}ylen: {door_data.get('ylen', 'N/A')}")
            print(f"door{i}yframe_1: {door_data.get('yframe_1', 'N/A')}")
            print(f"door{i}yframe_2: {door_data.get('yframe_2', 'N/A')}")


# Helper functions to get specific data for use in other modules
def get_all_scan_data(default_on_error=False):
    """
    Returns the entire scan data dictionary for use in other modules

    Args:
        default_on_error (bool): If True, returns default values when file is missing or invalid

    Returns:
        dict: Dictionary containing all scan data or default values if specified
    """
    return read_scan_results(default_on_error)


def get_door_position(door_number, default_on_error=False, default_value="null"):
    """
    Get the 7th axis position for a specific door

    Args:
        door_number (int): Door number (1-4)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value: Default value to return if the position is not found

    Returns:
        Door position or default_value if not found
    """
    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    key = f"7thx{door_number}"
    return scan_data.get(key, default_value)


def get_frame_point(
    door_number, corner_number, default_on_error=False, default_value=None
):
    """
    Get the frame point for a specific door and corner

    Args:
        door_number (int): Door number (1-4)
        corner_number (int): Corner number (0-3)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (list): Default value to return if the point is not found

    Returns:
        list: Coordinates of the frame point or default_value if not found
    """
    if default_value is None:
        default_value = ["null", "null", "null"]

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    key = f"fr{door_number}door{corner_number}"
    return scan_data.get(key, default_value)


def get_inner_corner_point(
    door_number, corner_number, default_on_error=False, default_value=None
):
    """
    Get the inner corner point for a specific door and corner

    Args:
        door_number (int): Door number (1-4)
        corner_number (int): Corner number (0-3)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (list): Default value to return if the point is not found

    Returns:
        list: Coordinates of the inner corner point or default_value if not found
    """
    if default_value is None:
        default_value = ["null", "null", "null"]

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    key = f"in{door_number}door{corner_number}"
    return scan_data.get(key, default_value)


def get_outer_corner_point(
    door_number, corner_number, default_on_error=False, default_value=None
):
    """
    Get the outer corner point for a specific door and corner

    Args:
        door_number (int): Door number (1-4)
        corner_number (int): Corner number (0-3)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (list): Default value to return if the point is not found

    Returns:
        list: Coordinates of the outer corner point or default_value if not found
    """
    if default_value is None:
        default_value = ["null", "null", "null"]

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    key = f"out{door_number}door{corner_number}"
    return scan_data.get(key, default_value)


def get_pocket_point(
    door_number, corner_number, default_on_error=False, default_value=None
):
    """
    Get the pocket point for a specific door and corner

    Args:
        door_number (int): Door number (1-4)
        corner_number (int): Corner number (0-3)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (list): Default value to return if the point is not found

    Returns:
        list: Coordinates of the pocket point or default_value if not found
    """
    if default_value is None:
        default_value = ["null", "null", "null"]

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    key = f"pocket{door_number}door{corner_number}"
    return scan_data.get(key, default_value)


def get_x_values(door_number, default_on_error=False, default_value=None):
    """
    Get the x values (xlen, xframe_1, xframe_2) for a specific door

    Args:
        door_number (int): Door number (1-4)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (dict): Default value to return if the values are not found

    Returns:
        dict: Dictionary with x values or default_value if not found
    """
    if default_value is None:
        default_value = {"xlen": "null", "xframe_1": "null", "xframe_2": "null"}

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    x_vals = scan_data.get("xVals", [])
    if not x_vals or door_number < 1 or door_number > len(x_vals):
        return default_value

    return x_vals[door_number - 1]


def get_y_values(door_number, default_on_error=False, default_value=None):
    """
    Get the y values (ylen, yframe_1, yframe_2) for a specific door

    Args:
        door_number (int): Door number (1-4)
        default_on_error (bool): If True, returns default values when file is missing or invalid
        default_value (dict): Default value to return if the values are not found

    Returns:
        dict: Dictionary with y values or default_value if not found
    """
    if default_value is None:
        default_value = {"ylen": "null", "yframe_1": "null", "yframe_2": "null"}

    scan_data = read_scan_results(default_on_error)
    if not scan_data:
        return default_value

    y_vals = scan_data.get("yVals", [])
    if not y_vals or door_number < 1 or door_number > len(y_vals):
        return default_value

    return y_vals[door_number - 1]


if __name__ == "__main__":
    # Example usage
    print_door_info()
