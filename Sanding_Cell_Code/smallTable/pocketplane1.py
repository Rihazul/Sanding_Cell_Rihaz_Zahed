# testcommu.py

import sys
import os
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from scancord import (
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
    with open('../configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


def smalldoor1pocketside(force,z):
    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Main Points
    b0 = get_pocket_point(1, 0)
    print("b0:", b0)
    b1 = get_pocket_point(1, 1)
    print("b1:", b1)
    b2= get_pocket_point(1, 2)
    print("b2:", b2)
    b3 = get_pocket_point(1, 3)
    print("b3", b3)
    #7th axis postion
    x1=get_door_position(1)
    print("x1:", x1)
    #prehoming
    prehoming=[0,0,50,0,0,0]
    #Depth of the poocket z
    # z=-6.5
    #offset
    offset=1.5

    #Bottom Points
    bottom0=[b0[0]+1.5*offset,b0[1]+offset,z,b0[3],b0[4],0]
    print("bottom0:", bottom0)
    bottom3=[b3[0],b3[1],z,b3[3],b3[4],0]
    print("bottom3:", bottom3)
    bottom0pre=[b0[0]+offset,b0[1]+offset,10,b0[3],b0[4],0]
    print("bottom0pre:", bottom0pre)
    bottom3pre=[b3[0]-offset,b3[1]+offset,10,b3[3],b3[4],0]
    print("bottom3pre:", bottom3pre)
    bottom03=[b0[0]+offset+2,b0[1]+offset,z,b0[3],b0[4],0]
    print("bottom03:", bottom03)
    bottom2= [b2[0],b2[1]-2*offset,z,b2[3],b2[4],0]
    print("bottom2:", bottom2)
    bottom1= [b1[0]+1.5*offset,b1[1]-2*offset,z,b1[3],b1[4],0]
    print("bottom1:", bottom1)
    bottom1pre= [b1[0]+offset,b1[1]-offset,10,b1[3],b1[4],0]
    print("bottom1pre:", bottom1pre)

    bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
    print("bottompoints:", bottompoints)



    def perform_process_bottom(cps, config, points1,force):
        
        # Communicate to each point in points1
        for point in points1:
            if point==bottom03:putForceZminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==bottom3:
                time.sleep(0.01)
                turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=0.6,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)

    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
    perform_process_bottom(cps, config, points1=bottompoints,force=force)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)

def smalldoor2pocketside(force,z):
    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Main Points
    twob0 = get_pocket_point(2, 0)
    print("twob0:", twob0)
    twob1 = get_pocket_point(2, 1)
    print("twob1:", twob1)
    twob2= get_pocket_point(2, 2)
    print("twob2:", twob2)
    twob3 = get_pocket_point(2, 3)
    print("twob3", twob3)
    #7th axis postion
    x2=get_door_position(2)
    print("x2:", x2)
    #prehoming
    prehoming=[0,0,50,0,0,0]
    #Depth of the poocket z
    z=-6.5
    #offset
    offset=1.5

    #Bottom Points
    twobottom0=[twob0[0]+1.5*offset,twob0[1]+offset,z,twob0[3],twob0[4],0]
    print("twobottom0:",twobottom0)
    twobottom3=[twob3[0],twob3[1],z,twob3[3],twob3[4],0]
    print("twobottom3:",twobottom3)
    twobottom0pre=[twob0[0]+offset,twob0[1]+offset,10,twob0[3],twob0[4],0]
    print("twobottom0pre:",twobottom0pre)
    twobottom3pre=[twob3[0],twob3[1],10,twob3[3],twob3[4],0]
    print("twobottom3pre:",twobottom3pre)
    twobottom03=[twob0[0]+offset+2,twob0[1]+offset,z,twob0[3],twob0[4],0]
    print("twobottom03:",twobottom03)
    twobottom2= [twob2[0],twob2[1]-2*offset,z,twob2[3],twob2[4],0]
    print("twobottom2:",twobottom2)
    twobottom1= [twob1[0]+1.5*offset,twob1[1]-2*offset,z,twob1[3],twob1[4],0]
    print("twobottom1:",twobottom1)
    twobottom1pre= [twob1[0]+offset,twob1[1]-offset,10,twob1[3],twob1[4],0]
    print("twobottom1pre:",twobottom1pre)

    twobottompoints=[twobottom0pre,twobottom0,twobottom03,twobottom3,twobottom2,twobottom1,twobottom0,twobottom0pre]
    print("twobottompoints:",twobottompoints)



    def perform_process_bottom(cps, config, points1,force):
        
        # Communicate to each point in points1
        for point in points1:
            if point==twobottom03:putForceZminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==twobottom3:
                time.sleep(0.01)
                turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=0.6,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)

    communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
    perform_process_bottom(cps, config, points1=twobottompoints,force=force)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)

def smalldoor3pocketside(force,z):
    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Main Points
    threeb0 = get_pocket_point(3, 0)
    print("threeb0:", threeb0)
    threeb1 = get_pocket_point(3, 1)
    print("threeb1:", threeb1)
    threeb2= get_pocket_point(3, 2)
    print("threeb2:", threeb2)
    threeb3 = get_pocket_point(3, 3)
    print("threeb3", threeb3)
    #7th axis postion
    x3=get_door_position(3)
    print("x3:", x3)
    #prehoming
    prehoming=[0,0,50,0,0,0]
    #Depth of the poocket z
    z=-6.5
    #offset
    offset=1.5

    #Bottom Points
    threebottom0=[threeb0[0]+1.5*offset,threeb0[1]+offset,z,threeb0[3],threeb0[4],0]
    print("threebottom0:",threebottom0)
    threebottom3=[threeb3[0],threeb3[1],z,threeb3[3],threeb3[4],0]
    print("threebottom3:",threebottom3)
    threebottom0pre=[threeb0[0]+offset,threeb0[1]+offset,10,threeb0[3],threeb0[4],0]
    print("threebottom0pre:",threebottom0pre)
    threebottom3pre=[threeb3[0],threeb3[1],10,threeb3[3],threeb3[4],0]
    print("threebottom3pre:",threebottom3pre)
    threebottom03=[threeb0[0]+2,threeb0[1]+offset,z,threeb0[3],threeb0[4],0]
    print("threebottom03:",threebottom03)
    threebottom2= [threeb2[0],threeb2[1]-2*offset,z,threeb2[3],threeb2[4],0]
    print("threebottom2:",threebottom2)
    threebottom1= [threeb1[0]+1.5*offset,threeb1[1]-2*offset,z,threeb1[3],threeb1[4],0]
    print("threebottom1:",threebottom1)
    threebottom1pre= [threeb1[0]+offset,threeb1[1]-offset,10,threeb1[3],threeb1[4],0]
    print("threebottom1pre:",threebottom1pre)

    threebottompoints=[threebottom0pre,threebottom0,threebottom03,threebottom3,threebottom2,threebottom1,threebottom0,threebottom0pre]
    print("threebottompoints:",threebottompoints)



    def perform_process_bottom(cps, config, points1,force):
        
        # Communicate to each point in points1
        for point in points1:
            if point==threebottom03:putForceZminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==threebottom3:
                time.sleep(0.01)
                turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=0.6,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)

    communicate(cps=cps,config=config,seventh=x3,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
    perform_process_bottom(cps, config, points1=threebottompoints,force=force)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)

def smalldoor4pocketside(force,z):
    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Main Points
    fourb0 = get_pocket_point(4, 0)
    print("fourb0:", fourb0)
    fourb1 = get_pocket_point(4, 1)
    print("fourb1:", fourb1)
    fourb2= get_pocket_point(4, 2)
    print("fourb2:", fourb2)
    fourb3 = get_pocket_point(4, 3)
    print("fourb3", fourb3)
    #7th axis postion
    x4=get_door_position(4)
    print("x4:", x4)
    #prehoming
    prehoming=[0,0,50,0,0,0]
    #Depth of the poocket z
    z=-6.5
    #offset
    offset=1.5

    #Bottom Points
    fourbottom0=[fourb0[0]+1.5*offset,fourb0[1]+offset,z,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom0:",fourbottom0)
    fourbottom3=[fourb3[0],fourb3[1],z,fourb3[3],fourb3[4],fourb3[5]]
    print("fourbottom3:",fourbottom3)
    fourbottom0pre=[fourb0[0]+offset,fourb0[1]+offset,10,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom0pre:",fourbottom0pre)
    fourbottom3pre=[fourb3[0],fourb3[1],10,fourb3[3],fourb3[4],fourb3[5]]
    print("fourbottom3pre:",fourbottom3pre)
    fourbottom03=[fourb0[0]+offset+2,fourb0[1]+offset,z,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom03:",fourbottom03)
    fourbottom2= [fourb2[0],fourb2[1]-2*offset,z,fourb2[3],fourb2[4],fourb2[5]]
    print("fourbottom2:",fourbottom2)
    fourbottom1= [fourb1[0]+1.5*offset,fourb1[1]-2*offset,z,fourb1[3],fourb1[4],fourb1[5]]
    print("fourbottom1:",fourbottom1)
    fourbottom1pre= [fourb1[0]+offset,fourb1[1]-offset,10,fourb1[3],fourb1[4],fourb1[5]]
    print("fourbottom1pre:",fourbottom1pre)

    fourbottompoints=[fourbottom0pre,fourbottom0,fourbottom03,fourbottom3,fourbottom2,fourbottom1,fourbottom0,fourbottom0pre]
    print("fourbottompoints:",fourbottompoints)



    def perform_process_bottom(cps, config, points1,force):
        
        # Communicate to each point in points1
        for point in points1:
            if point==fourbottom03:putForceZminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==fourbottom3:
                time.sleep(0.01)
                turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                seventh=-1,
                speed=0.6,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)

    communicate(cps=cps,config=config,seventh=x4,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
    perform_process_bottom(cps, config, points1=fourbottompoints,force=force)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)

if __name__ == "__main__":
    smalldoor1pocketside(force=5,z=-6.5)
    smalldoor2pocketside(force=5,z=-6.5)
    smalldoor3pocketside(force=5,z=-6.5)
    #smalldoor4pocketside(force=5,z=-6.5)
    

    