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

def smalldoor1side(force):
    def smalldoor1sidesmall(force):
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
        b0 = get_frame_point(1, 0)
        print("b0:", b0)
        b1 = get_frame_point(1, 1)
        print("b1:", b1)
        b2= get_frame_point(1, 2)
        print("b2:", b2)
        b3 = get_frame_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
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

    def smalldoor1sidebig(force):
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
        b0 = get_frame_point(1, 0)
        print("b0:", b0)
        b1 = get_frame_point(1, 1)
        print("b1:", b1)
        b2= get_frame_point(1, 2)
        print("b2:", b2)
        b3 = get_frame_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,50,0,0,0]

        #Bottom Points
        bottom0=[b0[0],b0[1],1,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],1,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2,b0[1],1,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1],1,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0],b1[1],1,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0],b1[1],10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0],b1[1]/2,1,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0],b1[1]/2-2,1,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0],b1[1]/2+2,1,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0],b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,1,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)


        #Bottom Points Modified


        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2,b2[1],1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2,b2[1]/2,10,b2[3],b2[4],0]
        print("toppoint4pre:", toppoint4pre)

        #COnveyer
        conx1=x1
        print("conx1:", conx1)
        conxb=b0[0]
        print ("conxb:",conxb)
        conx2=conxb+(b2[0]-b1[0])/2
        print("conx2:", conx2)

        


        bottompoints=[bottom0pre,bottom0,bottom03,bottom3,bottom2,bottom1,bottom0,bottom0pre]
        print("bottompoints:", bottompoints)
        #Bottom Points
        bottomdoorpoints=[bottom4pre,bottom4,bottom4down,bottom0,bottom3,bottom5,bottom5pre]
        print("bottomdoorpoints:", bottomdoorpoints)
        #Upper Points
        upperdoorpoints=[toppoint5pre,toppoint5,toppoint5top,toppoint2,toppoint1,toppoint4,toppoint4pre]
        print("upperdoorpoints:", upperdoorpoints)
        



        def perform_process_bottom(cps, config, points1,force):
            
            # Communicate to each point in points1
            for point in points1:
                if point==bottom4down:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==bottom0:
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

        def perform_process_top(cps, config, points1,force):
            
            # Communicate to each point in points1
            for point in points1:
                if point==toppoint5top:putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==toppoint2:
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
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.2,
                    wait=True
                )

            def run_axis_movement():
                communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    speed=0.2,
                    wait=True
                )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()
        communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        perform_process_bottom(cps, config, points1=bottomdoorpoints,force=force)
        #communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        run_single_movement(robot_point=toppoint5pre, seventh_axis_point=conx2, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=conx2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        perform_process_top(cps, config, points1=upperdoorpoints,force=force)
        communicate(cps=cps,config=config,point=midhoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
        #communicate(cps=cps,config=config,seventh=conx1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
    
    #Main Function Execution
    # ylen = get_y_values(1)['ylen']
    # print("ylen:",ylen)
    # smalldoor1sidebig(force)
    # smalldoor1sidesmall(force)
    # Get ylen value (with default handling)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor1sidebig(force)
        else:
            smalldoor1sidesmall(force)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor2side(force):
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
    twob0 = get_frame_point(2, 0)
    print("twob0:", twob0)
    twob1 = get_frame_point(2, 1)
    print("twob1:", twob1)
    twob2 = get_frame_point(2, 2)
    print("twob2:", twob2)
    twob3 = get_frame_point(2, 3)
    print("twob3", twob3)
    #7th axis postion
    x2=get_door_position(2)
    print("x2:", x2)
    #prehoming
    prehoming=[0,0,50,0,0,0]

    #Bottom Points
    twobottom0=[twob0[0],twob0[1],1,twob0[3],twob0[4],0]
    print("twobottom0:", twobottom0)
    twobottom3=[twob3[0],twob3[1],1,twob3[3],twob3[4],0]
    print("twobottom3:", twobottom3)
    twobottom0pre=[twob0[0],twob0[1],10,twob0[3],twob0[4],0]
    print("twobottom0pre:", twobottom0pre)
    twobottom3pre=[twob3[0],twob3[1],10,twob3[3],twob3[4],0]
    print("twobottom3pre:", twobottom3pre)
    twobottom03=[twob0[0]+2,twob0[1],1,twob0[3],twob0[4],0]
    print("twobottom03:", twobottom03)
    twobottom2= [twob2[0],twob2[1],1,twob2[3],twob2[4],0]
    print("twobottom2:",twobottom2)
    twobottom1= [twob1[0],twob1[1],1,twob1[3],twob1[4],0]
    print("twobottom1:", twobottom1)
    twobottom1pre= [twob1[0],twob1[1],10,twob1[3],twob1[4],0]
    print("twobottom1pre:", twobottom1pre)

    twobottompoints=[twobottom0pre,twobottom0,twobottom03,twobottom3,twobottom2,twobottom1,twobottom0,twobottom0pre]
    print("twobottompoints:", twobottompoints)




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

def smalldoor3side(force):
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
    threeb0 = get_frame_point(3, 0)
    print("threeb0:", threeb0)
    threeb1 = get_frame_point(3, 1)
    print("threeb1:", threeb1)
    threeb2 = get_frame_point(3, 2)
    print("threeb2:", threeb2)
    threeb3 = get_frame_point(3, 3)
    print("threeb3", threeb3)
    #7th axis postion
    x3=get_door_position(3)
    print("x3:", x3)
    #prehoming
    prehoming=[0,0,50,0,0,0]

    #Bottom Points
    threebottom0=[threeb0[0],threeb0[1],1,threeb0[3],threeb0[4],0]
    print("threebottom0:", threebottom0)
    threebottom3=[threeb3[0],threeb3[1],1,threeb3[3],threeb3[4],0]
    print("threebottom3:", threebottom3)
    threebottom0pre=[threeb0[0],threeb0[1],10,threeb0[3],threeb0[4],0]
    print("threebottom0pre:", threebottom0pre)
    threebottom3pre=[threeb3[0],threeb3[1],10,threeb3[3],threeb3[4],0]
    print("threebottom3pre:", threebottom3pre)
    threebottom03=[threeb0[0]+2,threeb0[1],1,threeb0[3],threeb0[4],0]
    print("threebottom03:", threebottom03)
    threebottom2= [threeb2[0],threeb2[1],1,threeb2[3],threeb2[4],0]
    print("threebottom2:",threebottom2)
    threebottom1= [threeb1[0],threeb1[1],1,threeb1[3],threeb1[4],0]
    print("threebottom1:", threebottom1)
    threebottom1pre= [threeb1[0],threeb1[1],10,threeb1[3],threeb1[4],0]
    print("threebottom1pre:", threebottom1pre)

    threebottompoints=[threebottom0pre,threebottom0,threebottom03,threebottom3,threebottom2,threebottom1,threebottom0,threebottom0pre]
    print("threebottompoints:", threebottompoints)




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

def smalldoor4side(force):
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
    fourb0 = get_frame_point(4, 0)
    print("fourb0:", fourb0)
    fourb1 = get_frame_point(4, 1)
    print("fourb1:", fourb1)
    fourb2 = get_frame_point(4, 2)
    print("fourb2:", fourb2)
    fourb3 = get_frame_point(4, 3)
    print("fourb3", fourb3)
    #7th axis postion
    x4=get_door_position(4)
    print("x4:", x4)
    #prehoming
    prehoming=[0,0,50,0,0,0]

    #Bottom Points
    fourbottom0=[fourb0[0],fourb0[1],1,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom0:", fourbottom0)
    fourbottom3=[fourb3[0],fourb3[1],1,fourb3[3],fourb3[4],fourb3[5]]
    print("fourbottom3:", fourbottom3)
    fourbottom0pre=[fourb0[0],fourb0[1],10,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom0pre:", fourbottom0pre)
    fourbottom3pre=[fourb3[0],fourb3[1],10,fourb3[3],fourb3[4],fourb3[5]]
    print("fourbottom3pre:", fourbottom3pre)
    fourbottom03=[fourb0[0]+2,fourb0[1],1,fourb0[3],fourb0[4],fourb0[5]]
    print("fourbottom03:", fourbottom03)
    fourbottom2= [fourb2[0],fourb2[1],1,fourb2[3],fourb2[4],fourb2[5]]
    print("fourbottom2:",fourbottom2)
    fourbottom1= [fourb1[0],fourb1[1],1,fourb1[3],fourb1[4],fourb1[5]]
    print("fourbottom1:", fourbottom1)
    fourbottom1pre= [fourb1[0],fourb1[1],10,fourb1[3],fourb1[4],fourb1[5]]
    print("fourbottom1pre:", fourbottom1pre)

    fourbottompoints=[fourbottom0pre,fourbottom0,fourbottom03,fourbottom3,fourbottom2,fourbottom1,fourbottom0,fourbottom0pre]
    print("fourbottompoints:", fourbottompoints)




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
    #smalldoor1side(force=5)
    smalldoor2side(force=5)
    smalldoor3side(force=5)
    #smalldoor4side(force=5)
    