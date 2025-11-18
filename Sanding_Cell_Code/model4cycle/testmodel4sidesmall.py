# testcommu.py

import sys
import os
import json
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from Table4Model.exportpointsmodule import exported_points
from model4cycle.testmodel4side import testmodel4sidesmallfunctionbig


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config






def testmodel4sidesmallfunction(force,cps):
    def testmodel4sidesmallfunctionsmall(force,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        p1 = exported_points["p1"]
        p2 = exported_points["p2"]
        p3 = exported_points["p3"]
        p4 = exported_points["p4"]
        p5 = exported_points["p5"]
        p6 = exported_points["p6"]
        p7 = exported_points["p7"]
        p8 = exported_points["p8"]
        p9 = exported_points["p9"]
        p10 = exported_points["p10"]
        p11 = exported_points["p11"]
        p12 = exported_points["p12"]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        print(speeed)

        #speeed=0.7

        print("p1=",p1)
        print("p2=",p2)
        print("p3=",p3)
        print("p4=",p4)
        print("p5=",p5)
        print("p6=",p6)
        print("p7=",p7)
        print("p8=",p8)
        print("p9=",p9)
        print("p10=",p10)
        print("p11=",p11)
        print("p12=",p12)
        
        #Home Position
        phome=[0,0,-100,180,0,0]
        #Width of the Door
        width=p1[0]-p3[0]
        print("width=",width)
        #Width of the Frame
        framewidth=p7[0]-p3[0]
        print("framewidth=",framewidth)
        
        #Conveyer Movement for the Bottom Portion
        x0=0
        print("x0=",x0)
        x1=width/2
        print("x1=",x1)
        

        #Bootom Points
        pointbottom1=[p4[0]+framewidth/2,p4[1]+framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointbottom1=",pointbottom1)
        pointbottom2=[width/2-framewidth/2,p4[1]+framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointbottom2=",pointbottom2)
        pointbottom1pre=[p4[0]+framewidth/2,p4[1]+framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("pointbottom1pre=",pointbottom1pre)
        pointbottom2pre=[width/2-framewidth/2,p4[1]+framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("pointbottom2pre=",pointbottom2pre)
        pointbottom12=[p4[0]+framewidth/2+2,p4[1]+framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointbottom12=",pointbottom12)
        pointbottomair=[p4[0]+framewidth/2,p4[1]+framewidth/2,p4[2]-20,p4[3],p4[4],p4[5]]
        print("pointbottomair=",pointbottomair)

        #Left Side Points
        pointleft1=[width/2-framewidth/2,p4[1]+framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointleft1=",pointleft1)
        pointleft2=[width/2-framewidth/2,p3[1]-framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointleft2=",pointleft2)
        pointleft12=[width/2-framewidth/2,p4[1]+framewidth/2+2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointleft12=",pointleft12)
        pointleft2pre=[width/2-framewidth/2,p3[1]-framewidth/2,p4[2]-5-10,p4[3],p4[4],p4[5]]
        print("pointleft2air=",pointleft2pre)

        #Pointstop
        #Top Points
        pointtop1=[width/2-framewidth/2,p3[1]-framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointtop1=",pointtop1)
        pointtop12=[width/2-framewidth/2-2,p3[1]-framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointtop12=",pointtop12)
        pointtop2=[0,p3[1]-framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("pointtop2=",pointtop2)
        pointtop2pre=[0,p3[1]-framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("pointtop2pre=",pointtop2pre)
        pointtopair=[width/2-framewidth/2,p3[1]-framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("pointtopair=",pointtopair)


        #Right Points
        point1right=[p4[0]+framewidth/2,p4[1]+framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("point1right=",point1right)
        point2right=[p4[0]+framewidth/2,p3[1]-framewidth/2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("point2right=",point2right)
        point21right=[p4[0]+framewidth/2,p3[1]-framewidth/2-2,p4[2]-6,p4[3],p4[4],p4[5]]
        print("point21right=",point21right)
        point1rightpre=[p4[0]+framewidth/2,p4[1]+framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("point1rightpre=",point1rightpre)
        pointrightair=[p4[0]+framewidth/2,p3[1]-framewidth/2,p4[2]-15,p4[3],p4[4],p4[5]]
        print("pointrightair=",pointrightair)


        pointsbottom=[pointbottom1pre,pointbottom1,pointbottom12,pointbottom2,pointbottom2pre]
        print("pointsbottom=",pointsbottom)
        conveyerbottoms=[x0,x1]
        print("conveyerbottoms=",conveyerbottoms)
        pointslefts=[pointleft1,pointleft12,pointleft2,pointleft2pre]
        print("pointslefts=",pointslefts)
        pointstops=[pointtop1,pointtop12,pointtop2,pointtop2pre]
        print("pointstops=",pointstops)
        pointsrights=[point2right,point21right,point1right,point1rightpre]
        print("pointsrights=",pointsrights)


        def perform_process_bottom(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointbottom12:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointbottom2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_left(cps, config, points1,force):
            # Vibration on
            #turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointleft12:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointleft2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_topreal(cps, config, points1,force):
            # Vibration on
            #turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointtop12:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointtop2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_right(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==point21right:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==point1right:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        # def perform_process_middle(cps, config, points1):
        #     # Vibration on
        #     #turn_vibration_on(cps)
            
        #     # Force Control Activated
        #     #putForceZplus(
        #         #cps=cps,
        #         #force=15,
        #         #tcp=config['coords']['tcpReal'],
        #         #ucs=config['coords']['ucsTable2'],
        #         #config=config
        #     #)
            
        #     # Communicate to each point in points1
        #     for point in points1:
        #         if point==point12middle:putForceZplus(
        #         cps=cps,
        #         force=10,
        #         tcp=config['coords']['tcpReal'],
        #         ucs=config['coords']['ucsTable2'],
        #         config=config
        #         )
        #         if point==point2middle:turn_vibration_on(cps)
        #         communicate(
        #             cps=cps,
        #             config=config,
        #             point=point,
        #             tcp=config['coords']['tcpReal'],
        #             ucs=config['coords']['ucsTable2'],
        #             seventh=-1,
        #             speed=0.6,
        #             wait=False
        #         )
            
        #     # Wait for blending and turn off vibration
        #     waitForBlending(cps=cps, config=config)
        #     turn_vibration_off(cps)
            
        #     # Release Force Control
        #     releaseForce(cps=cps, config=config)
        
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            lock = threading.Lock()
            #time.sleep(0.2)
            """
            Moves the robot and seventh axis using one robot point and one seventh-axis point.

            :param robot_point: A single set of coordinates for the robot to move to.
            :param seventh_axis_point: A single point or value for the seventh axis.
            :param cps: The CPS object or any required instance used inside communicate().
            :param config: A configuration dictionary that contains coords, etc.
            """

            def run_robot_movement():
                with lock:
                    communicate(
                    cps=cps,
                    config=config,
                    point=robot_point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,   # or whatever parameter is needed for robot movement
                    speed=0.8,
                    wait=True
                )

            #time.sleep(0.1)

            def run_axis_movement():
                with lock:
                    communicate(
                    cps=cps,
                    config=config,
                    seventh=seventh_axis_point,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    speed=0.5,
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
        #Bottom Cycle
        communicate(cps=cps,config=config,seventh=x0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=pointbottomair,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
        #Bottom Cycle 1
        perform_process_bottom(cps, config, points1=pointsbottom,force=force)
        
        # Run only 1 cycle at position x1
        current_x = x1
        print(f"Processing bottom cycle at position x1 = {current_x}")
        run_single_movement(robot_point=pointbottomair, seventh_axis_point=current_x, cps=cps, config=config)
        perform_process_bottom(cps, config, points1=pointsbottom,force=force)
        
        #Left Cycle Movement
        perform_process_left(cps, config, points1=pointslefts,force=force)
        #Top Cycle Movement
        #perform_process_topreal(cps, config, points1=pointstops)


        # current_x = x1
        # print(f"Processing bottom cycle at position x1 = {current_x}")
        # run_single_movement(robot_point=pointtopair, seventh_axis_point=current_x, cps=cps, config=config)
        # perform_process_topreal(cps, config, points1=pointstops)

        for i in [1, 0]:  # Process x3 first, then x1 (change to [1, 3] for original order)
            current_x = eval(f"x{i}")
            print(f"Processing bottom cycle at position x{i} = {current_x}")
            run_single_movement(robot_point=pointtopair, seventh_axis_point=current_x, cps=cps, config=config)
            perform_process_topreal(cps, config, points1=pointstops,force=force)
        
        #Right Movement
        run_single_movement(robot_point=pointrightair, seventh_axis_point=x0, cps=cps, config=config)
        #communicate(cps=cps,config=config,seventh=x0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.7,wait=True)
        perform_process_right(cps, config, points1=pointsrights,force=force)
        communicate(cps=cps,config=config,point=phome,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
 #Main Cycle
    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 1000:
            testmodel4sidesmallfunctionbig(force,cps)
        else:
            testmodel4sidesmallfunctionsmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(xlen)} - expected number or 'null'")
if __name__ == "__main__":
    testmodel4sidesmallfunction(force=5)
    