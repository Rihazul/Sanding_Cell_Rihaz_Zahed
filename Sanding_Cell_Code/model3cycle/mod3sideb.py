# testcommu.py


import sys
import os
import json
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading
from Table3Model.exportpointsmodule import exported_points
from model3cycle.mode3sides import mod3sidesmall

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






def mod3sidebig(force,cps):
    def mod3sidebigbig(force,cps):
        # Load configuration from YAML
        config = load_config()

        # Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        # Establish connection with robot
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

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        robot_speed = speeed
        sanding_speed = float(json_config.get('sandingSpeed', robot_speed))
        if sanding_speed <= 0:
            sanding_speed = robot_speed
        print(robot_speed)

        
        # p1= [2012.95000001, 0.0, 0, 180, 0, 0]
        # p2= [2012.95000001, 381.000000002, 0, 180, 0, 0]
        # p3= [0.0, 381.000000002, 0, 180, 0, 0]
        # p4= [0.0, 0.0, 0, 180, 0, 0]

        # p9= [704.8500000032, 57.1500000002, -9.525000000039999, 180, 0, 0]
        # p10= [704.8500000032, 323.8500000012, -9.525000000039999, 180, 0, 0]
        # p11= [57.1500000002, 323.8500000012, -9.525000000039999, 180, 0, 0]
        # p12= [57.1500000002, 57.1500000002, -9.525000000039999, 180, 0, 0]

        #Conveyer
        x=p4[0]
        print("x=",x)
        increment=p1[0]/4
        x1=increment
        print("x1=",x1)
        x2=increment*2
        print("x2=",x2)
        x3=increment*3
        print("x3=",x3)
        x4=increment*4
        print("x4=",x4)

        #Middle Conveyer movement
        xmiddle=p9[0]
        print("xmiddle=",xmiddle)

        #Offset and Tool Measurement
        tool3y = 50.8  # Tool offset in Y
        tool3x = 38.1  # Tool offset in X
        Offset = 5  # Inner boundary offset

        #Bottom Section
        point4=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2]-4,p4[3],p4[4],p4[5]]
        print("point4=",point4)
        point1=[p1[0]/4,p4[1]+tool3y-19.243,p4[2]-4,p4[3],p4[4],p4[5]]
        print("point1=",point1)
        point41=[p4[0]+tool3x-9.749+2,p4[1]+tool3y-19.243,p4[2]-4,p4[3],p4[4],p4[5]]
        print("point41=",point41)
        point4pre=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2]-1-3-10,p4[3],p4[4],p4[5]]
        print("point4pre=",point4pre)
        point1pre=[p1[0]/4,p4[1]+tool3y-19.243,p4[2]-1-3-10,p4[3],p4[4],p4[5]]
        print("point1pre=",point1pre)
        point1air=[p1[0]/4,p4[1]+tool3y-19.243,p4[2]-9,p4[3],p4[4],p4[5]]
        print("point1air=",point1air)
        point4air=[p4[0]+tool3x-9.749,p4[1]+tool3y-19.243,p4[2]-9,p4[3],p4[4],p4[5]]
        print("point4air=",point4air)

        #Left Side
        pointl2=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243,p4[2]-4,p4[3],p4[4],p4[5]]
        print("pointl2=",pointl2)
        pointl3=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-4,p4[3],p4[4],p4[5]]
        print("pointl3=",pointl3)
        pointl2pre=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243,p4[2]-1-3-10,p4[3],p4[4],p4[5]]
        print("pointl2pre=",pointl2pre)
        pointl3pre=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-1-3-10,p4[3],p4[4],p4[5]]
        print("pointl3pre=",pointl3pre)
        pointl3air=[p4[0]-tool3x+11.138,p3[1]-tool3y+23.769,p4[2]-9,p4[3],p4[4],p4[5]]
        print("pointl3air=",pointl3air)
        pointl23=[p4[0]-tool3x+11.138,p4[1]+tool3y-19.243+1+1,p4[2]-4,p4[3],p4[4],p4[5]]
        print("pointl23=",pointl23)

        #Top1
        pointtop2=[p2[0]/4-32.4,p2[1]-27.04,p2[2]-4,p2[3],p2[4],p2[5]]
        print("pointtop2=",pointtop2)
        pointtop21=[p2[0]/4-32.4-1-1,p2[1]-27.04,p2[2]-4,p2[3],p2[4],p2[5]]
        print("pointtop21=",pointtop21)
        pointtop2pre=[p2[0]/4-32.4,p2[1]-27.04,p2[2]-1-3-10,p2[3],p2[4],p2[5]]
        print("pointtop2pre=",pointtop2pre)
        pointtop1=[0,p2[1]-27.04,p2[2]-4,p2[3],p2[4],p2[5]]
        print("pointtop1=",pointtop1)
        pointtop1pre=[0,p2[1]-27.04,p2[2]-1-3-10,p2[3],p2[4],p2[5]]
        print("pointtop1pre=",pointtop1pre)
        pointtop1air=[0,p2[1]-27.04,p2[2]-9,p2[3],p2[4],p2[5]]
        print("pointtop1air=",pointtop1air)
        pointtop2air=[p2[0]/4-32.4,p2[1]-27.04,p2[2]-9,p2[3],p2[4],p2[5]]
        print("pointtop2air=",pointtop2air)

        #Right Points
        #Right Cycle
        point1right=[0+tool3x-9.749,p2[1]-tool3y+23.76,p2[2]-4,p2[3],p2[4],p2[5]]
        print("point1right=",point1right)
        point14right=[0+tool3x-9.749,p2[1]-tool3y+23.76-1-1,p2[2]-4,p2[3],p2[4],p2[5]]
        print("point14right=",point14right)
        point1rightpre=[0+tool3x-9.749,p2[1]-tool3y+23.76,p2[2]-1-3-10,p2[3],p2[4],p2[5]]
        print("point1rightpre=",point1rightpre)
        point4right=[0+tool3x-9.749,p4[1],p2[2]-4,p2[3],p2[4],p2[5]]
        print("point4right=",point4right)
        point4rightpre=[0+tool3x-9.749,p4[1],p2[2]-1-3-10,p2[3],p2[4],p2[5]]
        print("point4rightpre=",point4rightpre)
        point4rightair=[0+tool3x-9.749,p4[1],p2[2]-9,p2[3],p2[4],p2[5]]
        print("point4rightair=",point4rightair)

        #Middle Cycle
        point1middle=[p12[0]/2,0+tool3y-20.8,-4,180,0,0]
        print("point1middle=",point1middle)
        point1middlepre=[p12[0]/2,0+tool3y-20.8,-1-3-10,180,0,0]
        print("point1middlepre=",point1middlepre)
        point12middle=[p12[0]/2,0+tool3y-20.8+1+1,-4,180,0,0]
        print("point12middle=",point12middle)
        point2middle=[p12[0]/2,p2[1]-tool3y+23.76,-4,180,0,0]
        print("point2middle=",point2middle)
        point3middle=[((p12[0]/2)*3)+4.28,p2[1]-tool3y+23.76,-4,180,0,0]
        print("point3middle=",point3middle)
        point4middle=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-4,180,0,0]
        print("point4middle=",point4middle)
        point4middlepre=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-1-3-10,180,0,0]
        print("point4middlepre=",point4middlepre)
        point4middleair=[((p12[0]/2)*3)+4.28,0+tool3y-20.8,-9,180,0,0]
        print("point4middleair=",point4middleair)

        #fOR Bottom points
        points1=[point4pre,point4,point41,point1,point1pre]
        pointsleft=[pointl2pre,pointl2,pointl23,pointl3,pointl3pre]
        pointstop=[pointtop2pre,pointtop2,pointtop21,pointtop1,pointtop1pre]
        pointright=[point1rightpre,point1right,point14right,point4right,point4rightpre]
        pointmiddle=[point1middlepre,point1middle,point12middle,point2middle,point3middle,point4middle,point4middlepre]

        def perform_process_bottom_edge(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool4plane2'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==point41:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==point1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool4plane2'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
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
                #tcp=config['coords']['tcptool4plane2'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointl23:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointl3:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool4plane2'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_top_edge(cps, config, points1,force):
            # Vibration on
            #turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool4plane2'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointtop21:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointtop1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool4plane2'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
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
                #tcp=config['coords']['tcptool4plane2'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==point14right:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==point4right:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool4plane2'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_middle(cps, config, points1,force):
            # Vibration on
            #turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool4plane2'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==point12middle:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==point2middle:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool4plane2'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            lock = threading.Lock()
            # time.sleep(0.5)
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
                        tcp=config['coords']['tcptool4plane2'],
                        ucs=config['coords']['ucsTable2'],
                        seventh=-1,   # or whatever parameter is needed for robot movement
                        speed=robot_speed,
                        velocity_profile="robotspeed",
                        wait=True
                    )

            def run_axis_movement():
                with lock:
                    communicate(
                        cps=cps,
                        config=config,
                        seventh=seventh_axis_point,
                        tcp=config['coords']['tcptool4plane2'],
                        ucs=config['coords']['ucsTable2'],
                        speed=robot_speed,
                        velocity_profile="robotspeed",
                        wait=False
                    )

            # Start each movement in its own thread
            robot_thread = threading.Thread(target=run_robot_movement)
            axis_thread = threading.Thread(target=run_axis_movement)

            robot_thread.start()
            axis_thread.start()

            # Wait for both movements to finish before returning
            robot_thread.join()
            axis_thread.join()

        communicate(cps=cps,config=config,point=point4air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=robot_speed, velocity_profile="robotspeed", wait=False)
        #turn_vibration_on(cps)
        #communicate(cps=cps,config=config,point=point1pre,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
        #Bottom1
        perform_process_bottom_edge(cps, config, points1=points1,force=force)
        communicate(cps=cps,config=config,point=point1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        run_single_movement(robot_point=point4air, seventh_axis_point=x1, cps=cps, config=config)
        #Bottom2
        perform_process_bottom_edge(cps, config, points1=points1,force=force)
        communicate(cps=cps,config=config,point=point1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        run_single_movement(robot_point=point4air, seventh_axis_point=x2, cps=cps, config=config)
        #Bottom3
        perform_process_bottom_edge(cps, config, points1=points1,force=force)
        communicate(cps=cps,config=config,point=point1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        run_single_movement(robot_point=point4air, seventh_axis_point=x3, cps=cps, config=config)
        #Bottom4
        perform_process_bottom_edge(cps, config, points1=points1,force=force)
        communicate(cps=cps,config=config,point=point1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        run_single_movement(robot_point=point4air, seventh_axis_point=x4, cps=cps, config=config)
        #Left Side Cycle
        perform_process_left(cps, config, points1=pointsleft,force=force)
        communicate(cps=cps,config=config,point=pointl3air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        #Top Cycle
        #Top Cycle 1
        run_single_movement(robot_point=pointtop2air, seventh_axis_point=x3, cps=cps, config=config)
        perform_process_top_edge(cps, config, points1=pointstop,force=force)
        communicate(cps=cps,config=config,point=pointtop1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)

        #Top Cycle 2
        run_single_movement(robot_point=pointtop2air, seventh_axis_point=x2, cps=cps, config=config)
        perform_process_top_edge(cps, config, points1=pointstop,force=force)
        communicate(cps=cps,config=config,point=pointtop1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)

        #Top Cycle 3
        run_single_movement(robot_point=pointtop2air, seventh_axis_point=x1, cps=cps, config=config)
        perform_process_top_edge(cps, config, points1=pointstop,force=force)
        communicate(cps=cps,config=config,point=pointtop1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)

        #Top Cycle 4
        run_single_movement(robot_point=pointtop2air, seventh_axis_point=x, cps=cps, config=config)
        perform_process_top_edge(cps, config, points1=pointstop,force=force)
        communicate(cps=cps,config=config,point=pointtop1air,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)

        #Right Cycle
        perform_process_right(cps, config, points1=pointright,force=force)
        communicate(cps=cps,config=config,point=point4rightair,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)

        #Middle Cycle
        communicate(cps=cps,config=config,seventh=xmiddle,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=robot_speed, velocity_profile="robotspeed", wait=False)
        perform_process_middle(cps, config, points1=pointmiddle,force=force)
        communicate(cps=cps,config=config,point=point4middleair,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=robot_speed, velocity_profile="robotspeed", wait=True)
        communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=robot_speed, velocity_profile="robotspeed", wait=False)
    #Main Cycle
    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 1000:
            mod3sidebigbig(force,cps)
        else:
            mod3sidesmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(xlen)} - expected number or 'null'")


if __name__ == "__main__":
    mod3sidebig()
    




