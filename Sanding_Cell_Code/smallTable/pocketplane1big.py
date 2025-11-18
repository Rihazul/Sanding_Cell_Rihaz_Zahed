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




def smalldoor1sidebig(force,z):
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
        #z=-6.5
        #prehoming
        prehoming=[0,0,50,0,0,0]
        offset=1.5

        #Bottom Points
        bottom0=[b0[0]+1.5*offset,b0[1],z,b0[3],b0[4],0]
        print("bottom0:", bottom0)
        bottom3=[b3[0],b3[1],z,b3[3],b3[4],0]
        print("bottom3:", bottom3)
        bottom0pre=[b0[0],b0[1],10,b0[3],b0[4],0]
        print("bottom0pre:", bottom0pre)
        bottom3pre=[b3[0],b3[1],10,b3[3],b3[4],0]
        print("bottom3pre:", bottom3pre)
        bottom03=[b0[0]+2+1.5*offset,b0[1],z,b0[3],b0[4],0]
        print("bottom03:", bottom03)
        bottom2= [b2[0],b2[1]-2*offset,z,b2[3],b2[4],0]
        print("bottom2:", bottom2)
        bottom1= [b1[0]+1.5*offset,b1[1]-2*offset,z,b1[3],b1[4],0]
        print("bottom1:", bottom1)
        bottom1pre= [b1[0]+1.5*offset,b1[1]-2*offset,10,b1[3],b1[4],0]
        print("bottom1pre:", bottom1pre)
        bottom4=[b1[0]+1.5*offset,b1[1]/2,z,b1[3],b1[4],0]
        print("bottom4:", bottom4)
        bottom4down= [b1[0]+1.5*offset,b1[1]/2-2,z,b1[3],b1[4],0]
        print("bottom4down:", bottom4down)
        bottom4up= [b1[0]+1.5*offset,b1[1]/2+2,z,b1[3],b1[4],0]
        print("bottom4up:", bottom4up)
        bottom4pre= [b1[0]+1.5*offset,b1[1]/2,10,b1[3],b1[4],0]
        print("bottom4pre:", bottom4pre)
        bottom5=[b2[0],b2[1]/2,z,b2[3],b2[4],0]
        print("bottom5:", bottom5)
        bottom5pre=[b2[0],b2[1]/2,10,b2[3],b2[4],0]
        print("bottom5pre:", bottom5pre)

        #Mid Homing
        midhoming=[b3[0]/2,300,80,0,0,0]
        print("midhoming:",midhoming)


        #Bottom Points Modified


        #Modified Points
        toppoint2=[(b2[0]-b1[0])/2,b2[1]-2*offset,1,b2[3],b2[4],0]
        print("toppoint2:", toppoint2)
        toppoint5=[(b2[0]-b1[0])/2,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint5:", toppoint5)
        toppoint5top=[(b2[0]-b1[0])/2,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint5top:", toppoint5top)
        toppoint5pre=[(b2[0]-b1[0])/2,b2[1]/2,20,b2[3],b2[4],0]
        print("toppoint5pre:", toppoint5pre)
        toppoint1=[-(b2[0]-b1[0])/2+1.5*offset,b2[1]-2*offset,1,b2[3],b2[4],0]
        print("toppoint1:", toppoint1)
        toppoint4=[-(b2[0]-b1[0])/2+1.5*offset,b2[1]/2,1,b2[3],b2[4],0]
        print("toppoint4:", toppoint4)
        toppoint4top=[-(b2[0]-b1[0])/2+1.5*offset,b2[1]/2+2,1,b2[3],b2[4],0]
        print("toppoint4top:", toppoint4top)
        toppoint4pre=[-(b2[0]-b1[0])/2+1.5*offset,b2[1]/2,10,b2[3],b2[4],0]
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

if __name__ == "__main__":
    smalldoor1sidebig(force=2,z=-6.5)
    