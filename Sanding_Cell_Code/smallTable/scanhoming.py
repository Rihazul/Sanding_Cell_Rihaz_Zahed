# testcommu.py

import sys
import os
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus,stop_requested
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


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


def scanhoming(cps=None, config=None):
    # Load configuration from YAML if not provided
    if config is None:
        config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    if stop_requested():
        return
    #Establish connection with robot unless provided
    if cps is None:
        cps = CPSClient()
        IP = config['server']['cpip']
        port = config['server']['cps']
        ret = cps.HRIF_Connect(0, IP, port)

    #Main Points
    b00 = get_outer_corner_point(1, 0)
    print("b00:", b00)
    b10 = get_outer_corner_point(1, 1)
    print("b10:", b10)
    b20= get_outer_corner_point(1, 2)
    print("b20:", b20)
    b30 = get_outer_corner_point(1, 3)
    print("b30:", b30)
    #7th axis postion
    x1=get_door_position(1)
    print("x1:", x1)
    #prehoming
    prehoming=[0,0,50,0,0,0]

    #communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.9,wait=True)
    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)


if __name__ == "__main__":
    scanhoming()
    # smalldoor2side(force=5)
    # smalldoor3side(force=5)
    #smalldoor4side(force=5)
    
