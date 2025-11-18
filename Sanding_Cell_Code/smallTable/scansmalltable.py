# testcommu.py
import sys
import os
import json
import datetime
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,handle_client
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
        # output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
        output_dir = './smallTable/static'
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Use fixed filename instead of timestamp-based name
    output_file = os.path.join(output_dir, 'scan_results.json')
    
    # Get current timestamp for the data
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # Convert data to a format suitable for JSON serialization
    json_data = {
        'timestamp': timestamp,
        'robo7thPos': data['robo7thPos'],
        'framePoints': [{str(k): v for k, v in points.items()} for points in data['framePoints']],
        'pocketPoints': [{str(k): v for k, v in points.items()} for points in data['pocketPoints']],
        'outerCornerPoints': [{str(k): v for k, v in points.items()} for points in data['outerCornerPoints']],
        'innerCornerPoints': [{str(k): v for k, v in points.items()} for points in data['innerCornerPoints']],
        'xVals': data['xVals'],
        'yVals': data['yVals']
    }
    
    # Save to JSON file, overwriting any existing content
    with open(output_file, 'w') as f:
        json.dump(json_data, f, indent=4)
    
    print(f"Scan results saved to {output_file}")
    return output_file

def scanTableA():
    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    # Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    

    #Scan Table
    print("--- Attempting to perform table scanning via handle_client ---")
        # Call handle_client with parameters to only perform scanning.
        # This assumes handle_client is structured to:
        # 1. Internally call scan_table.
        # 2. Return the results from scan_table when startSanding is False.
        # 3. Manage its own CPS connection if needed for scanning.
    own7thpos, framePoints, pocketPoints, outerCornerPoints, innerCornerPoints, xVals, yVals = handle_client(
            config=config,
            homingState=False,      # Set to True if homing is needed before scanning by handle_client
            startSanding=False ,     # Crucial to ensure only scanning occurs and results are returned
            scan=True
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

    # Save scan results to JSON file
    scan_data = {
        'robo7thPos': own7thpos,
        'framePoints': framePoints,
        'pocketPoints': pocketPoints,
        'outerCornerPoints': outerCornerPoints, 
        'innerCornerPoints': innerCornerPoints,
        'xVals': xVals,
        'yVals': yVals
    }
    json_file_path = save_scan_results_to_json(scan_data)
    print(f"Scan results saved to JSON file: {json_file_path}")
    
    
    
    
    
    


if __name__ == "__main__":
    scanTableA()