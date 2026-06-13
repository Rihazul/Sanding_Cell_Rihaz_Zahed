# testcommu.py
import sys
import os
import json
import datetime
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import (
    setup_logger,
    waitForBlending,
    turn_vibration_on,
    turn_vibration_off,
    putForce,
    releaseForce,
    putForceZplus,
    handle_client,
    stop_requested,
)
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def save_scan_results_to_json(data, output_dir=None):
    print("Entered ....")
    """Save scan results to a fixed JSON file, overriding previous results."""
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Use fixed filename instead of timestamp-based name
    output_file = os.path.join(output_dir, 'scan_results.json')
    
    # Get current timestamp for the data
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    
    def _ensure_list(value):
        return value if isinstance(value, list) else []

    def _normalize_points_list(points_list):
        normalized = []
        for entry in _ensure_list(points_list):
            if isinstance(entry, dict):
                normalized.append({str(k): v for k, v in entry.items()})
            else:
                normalized.append({})
        return normalized

    def _safe_float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _map_detected_positions_to_slots(detected_positions, expected_slots):
        detected = []
        for idx, pos in enumerate(_ensure_list(detected_positions)):
            pos_val = _safe_float(pos)
            if pos_val is None:
                continue
            detected.append((idx, pos_val))

        slots = []
        for slot_idx, slot in enumerate(_ensure_list(expected_slots)):
            slot_val = _safe_float(slot)
            if slot_val is None:
                continue
            slots.append((slot_idx, slot_val))

        physical_numbers = [None] * len(_ensure_list(detected_positions))
        remaining_slots = slots[:]
        for det_idx, det_pos in detected:
            if not remaining_slots:
                break
            nearest_slot = min(
                remaining_slots,
                key=lambda slot_item: abs(float(slot_item[1]) - float(det_pos)),
            )
            physical_numbers[det_idx] = int(nearest_slot[0]) + 1
            remaining_slots = [
                slot_item
                for slot_item in remaining_slots
                if slot_item[0] != nearest_slot[0]
            ]

        for idx in range(len(physical_numbers)):
            if physical_numbers[idx] is None:
                physical_numbers[idx] = idx + 1

        return physical_numbers

    x_vals = _ensure_list(data.get('xVals'))
    y_vals = _ensure_list(data.get('yVals'))
    robo7th_pos = _ensure_list(data.get('robo7thPos'))
    expected_slots = _ensure_list(data.get('expectedDoorSlots'))
    physical_door_numbers = _map_detected_positions_to_slots(robo7th_pos, expected_slots)
    door_profiles = []
    total_doors = max(len(x_vals), len(y_vals))
    for idx in range(total_doors):
        x_entry = x_vals[idx] if idx < len(x_vals) and isinstance(x_vals[idx], dict) else {}
        y_entry = y_vals[idx] if idx < len(y_vals) and isinstance(y_vals[idx], dict) else {}
        if idx < len(physical_door_numbers):
            door_number = physical_door_numbers[idx]
        else:
            door_number = idx + 1
        door_profiles.append(
            {
                'doorNumber': door_number,
                'profile': y_entry.get('doorProfile')
                or x_entry.get('doorProfile')
                or 'unknown',
                'xProfileType': x_entry.get('xProfileType', 'unknown'),
                'yProfileType': y_entry.get('yProfileType', 'unknown'),
                'xPocketDetected': bool(x_entry.get('xPocketDetected', False)),
                'yPocketDetected': bool(y_entry.get('yPocketDetected', False)),
            }
        )

    # Convert data to a format suitable for JSON serialization
    json_data = {
        'timestamp': timestamp,
        'robo7thPos': robo7th_pos,
        'expectedDoorSlots': expected_slots,
        'physicalDoorNumbers': physical_door_numbers,
        'framePoints': _normalize_points_list(data.get('framePoints')),
        'pocketPoints': _normalize_points_list(data.get('pocketPoints')),
        'outerCornerPoints': _normalize_points_list(data.get('outerCornerPoints')),
        'innerCornerPoints': _normalize_points_list(data.get('innerCornerPoints')),
        'xVals': x_vals,
        'yVals': y_vals,
        'doorProfiles': door_profiles,
    }
    
    # Save to JSON file, overwriting any existing content
    with open(output_file, 'w') as f:
        json.dump(json_data, f, indent=4)
    
    print(f"Scan results saved to {output_file}")
    return output_file

def scanTableA(cps=None, config=None):
    # Load configuration from YAML if not provided
    if config is None:
        config = load_config()

    # Set up logger if needed
    if 'logger' not in config:
        config['logger'] = setup_logger(config['settings']['debug'])

    published_scan_path = None

    def publish_scan_results(scan_results):
        nonlocal published_scan_path
        (
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals,
            yVals,
        ) = scan_results
        scan_data = {
            'robo7thPos': own7thpos,
            'expectedDoorSlots': [
                config['table'][f'lengthx{i}']
                for i in range(int(config['table']['count']))
                if f'lengthx{i}' in config['table']
            ],
            'framePoints': framePoints,
            'pocketPoints': pocketPoints,
            'outerCornerPoints': outerCornerPoints,
            'innerCornerPoints': innerCornerPoints,
            'xVals': xVals,
            'yVals': yVals,
        }
        published_scan_path = save_scan_results_to_json(scan_data)
        return published_scan_path

    #Scan Table
    print("--- Attempting to perform table scanning via handle_client ---")
        # Call handle_client with parameters to only perform scanning.
        # This assumes handle_client is structured to:
        # 1. Internally call scan_table.
        # 2. Return the results from scan_table when startSanding is False.
        # 3. Manage its own CPS connection if needed for scanning.
    config["_scan_results_ready_callback"] = publish_scan_results
    try:
        own7thpos, framePoints, pocketPoints, outerCornerPoints, innerCornerPoints, xVals, yVals = handle_client(
                config=config,
                homingState=False,      # Set to True if homing is needed before scanning by handle_client
                startSanding=False ,     # Crucial to ensure only scanning occurs and results are returned
                scan=True,
                cps=cps
            )
    finally:
        config.pop("_scan_results_ready_callback", None)

    scan_failed = not any(
        [
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals,
            yVals,
        ]
    )
    if scan_failed:
        if stop_requested():
            raise InterruptedError("Scan cancelled by stop request.")
        detailed_reason = ""
        if isinstance(config, dict):
            detailed_reason = str(config.get("_scan_last_error") or "").strip()
        if detailed_reason:
            raise RuntimeError(detailed_reason)
        raise RuntimeError(
            "Scan did not collect any door data. Check J7 readiness and scan path, then retry."
        )

    print("\n--- Scan completed. Received data: ---")
    print(f"Robo 7th Pos: {own7thpos}")
    print(f"Frame Points: {framePoints}")
    print(f"Pocket Points: {pocketPoints}")
    print(f"Outer Corner Points: {outerCornerPoints}")
    print(f"Inner Corner Points: {innerCornerPoints}")
    print(f"X Vals: {xVals}")
    print(f"Y Vals: {yVals}")
    print("----------------------------------------")

    # Early publication occurs before the final safe-point return. Retry here
    # only if that publication failed or was unavailable.
    json_file_path = published_scan_path or publish_scan_results(
        (
            own7thpos,
            framePoints,
            pocketPoints,
            outerCornerPoints,
            innerCornerPoints,
            xVals,
            yVals,
        )
    )
    print(f"Scan results saved to JSON file: {json_file_path}")
    return json_file_path
    
    
    
    
    
    


if __name__ == "__main__":
    scanTableA()
