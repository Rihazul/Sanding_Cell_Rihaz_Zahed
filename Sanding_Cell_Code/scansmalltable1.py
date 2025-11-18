# testcommu.py
import sys
import os
# # Add parent directory to path so Python can find the modules
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,handle_client
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    # Ensure static directory exists for CSV files
    static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
    os.makedirs(static_dir, exist_ok=True)
    print(f"Ensuring static directory exists at: {static_dir}")
    
    # Add the static path to config if not already there
    config['paths'] = config.get('paths', {})
    config['paths']['static_dir'] = static_dir

    # Add model3D configuration if not exists
    if 'model3D' not in config:
        # Default values for model3D based on common configurations
        config['model3D'] = {
            1: 18,
            2: 20,
            3: 22,
            4: 25
        }
        print("Added default model3D configuration to config")

    # Make sure UI model is set properly
    if 'UI' not in config:
        config['UI'] = {}
    if 'model' not in config['UI']:
        config['UI']['model'] = 1
        print("Set default UI model to 1")

    # Make sure scanThreshold and scanThresholdMinD are defined
    if 'scanThreshold' not in config:
        config['scanThreshold'] = {'T1': 0.15, 'T2': 0.15, 'T3': 0.15, 'T4': 0.15}
        print("Added default scanThreshold configuration")
    if 'scanThresholdMinD' not in config:
        config['scanThresholdMinD'] = {'T1': 5, 'T2': 5, 'T3': 5, 'T4': 5}
        print("Added default scanThresholdMinD configuration")
    
    print(f"Model3D config: {config.get('model3D', 'Not present')}")
    print(f"Current UI model: {config.get('UI', {}).get('model', 'Not set')}")

    # Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    try:
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
                startSanding=False      # Crucial to ensure only scanning occurs and results are returned
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
        
    except Exception as e:
        print(f"Error during scanning: {str(e)}")
        import traceback
        print(traceback.format_exc())
    finally:
        # Disconnect from robot if we established our own connection
        if ret == 0:
            cps.HRIF_Disconnect(0)
            print("Robot connection closed.")

if __name__ == "__main__":
    main()