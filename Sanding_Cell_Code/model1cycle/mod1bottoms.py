# testcommu.py




import sys
import os
import time 
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))




import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_tool_spin_on,turn_vibration_off,turn_tool_spin_off,putForce,releaseForce,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from Table1Model.exportpointsmodule import exported_points








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





def model1bottomsmall(force,cps):
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
    p13 = exported_points["p13"]
    p14 = exported_points["p14"]
    p15 = exported_points["p15"]
    p16 = exported_points["p16"]




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
    print("p13=",p13)
    print("p14=",p14)
    print("p15=",p15)
    print("p16=",p16)
    json_config = load_json_config()
    robot_speed = float(json_config['robotSpeed'])
    sanding_speed = float(json_config['sandingSpeed'])
    speed_profile = {
        'travel': robot_speed,
        'approach': robot_speed,
        'return': robot_speed,
        'tool_change': robot_speed,
        'contact': sanding_speed,
    }
    if speed_profile['contact'] <= 0:
        speed_profile['contact'] = speed_profile['travel']
    z=-4


    #Boottom Points
    botlenght=p16[1]-p4[1]
    print("botlenght=",botlenght)
    framesize=p16[0]
    print("framesize=",framesize)
    totalbotomlength=botlenght-framesize
    print("totalbotomlength=",totalbotomlength)
    totalbotomlengthhalf =totalbotomlength/2
    print("totalbotomlengthhalf=",totalbotomlengthhalf)
    pointybottomb=totalbotomlengthhalf/2
    print("pointybottomb=",pointybottomb)
    pointybottoma=pointybottomb+totalbotomlengthhalf
    print("pointybottoma=",pointybottoma)
    #Final Bottom Points 1
    pointybottomb1=[p4[0],pointybottomb,z,180,0,0]
    print("pointybottomb1=",pointybottomb1)
    pointybottomb2=[p1[0],pointybottomb,z,180,0,0]
    print("pointybottomb2=",pointybottomb2)


    #Final Bottom Points 2
    pointybottoma1=[p4[0],pointybottoma,z,180,0,0]
    print("pointybottoma1=",pointybottoma1)
    pointybottoma2=[p1[0],pointybottoma,z,180,0,0]
    print("pointybottoma2=",pointybottoma2)


    #Bootom Points Movement
    #7th axis position
    length = p1[0]-p4[0]
    print("length=",length)
    
    bigger = False
    
    if length > 1350:
        length_chunk=length/3
        print("length_chunk=",length_chunk)
        bigger = True
    else:
        length_chunk=length/2
        print("length_chunk=",length_chunk)   
        bigger = False 
    
    x1=p3[0]
    print("x1=",x1)
    x2=length_chunk*1
    print("x2=",x2)
    x3=length_chunk*2
    print("x3=",x3)
    x4=length_chunk*3
    print("x4=",x4)
    x5=length_chunk*4
    print("x5=",x5)
    x6=length_chunk*5
    print("x6=",x6)

    prehoming=[0,0,-20,180,0,0]


    #Movement to bottom points for b
    pointybottomb1=[p4[0],pointybottomb,z,180,0,0]
    print("pointybottomb1=",pointybottomb1)
    prepointybottomb1=[p4[0],pointybottomb,-10,180,0,0]
    print("prepointybottomb1=",prepointybottomb1)
    pointybottomb12m=[p4[0]+2,pointybottomb,z,180,0,0]
    print("pointybottomb12m=",pointybottomb12m)
    pointybottomb2m=[length_chunk/1,pointybottomb,z,180,0,0]
    print("pointybottomb2m=",pointybottomb2m)
    prepointybottomb2m=[length_chunk/1,pointybottomb,-10,180,0,0]
    print("prepointybottomb2m=",prepointybottomb2m)


    #Movement to bottom points for a
    pointybottoma1=[p4[0],pointybottoma,z,180,0,0]
    print("pointybottoma1=",pointybottoma1)
    prepointybottoma1=[p4[0],pointybottoma,-10,180,0,0]
    print("prepointybottoma1=",prepointybottoma1)
    pointybottoma12m=[length_chunk/1+2,pointybottoma,z,180,0,0]
    print("pointybottoma12m=",pointybottoma12m)
    pointybottoma2m=[length_chunk/1,pointybottoma,z,180,0,0]
    print("pointybottoma2m=",pointybottoma2m)
    prepointybottoma2m=[length_chunk/1,pointybottoma,-10,180,0,0]
    print("prepointybottoma2m=",prepointybottoma2m)


    bottompointsb=[prepointybottomb1,pointybottomb1,pointybottomb12m,pointybottomb2m,prepointybottomb2m]
    print("bottompointsb=",bottompointsb)
    bottompointsa=[prepointybottoma2m,pointybottoma2m,pointybottoma12m,pointybottoma1,prepointybottoma1]
    print("bottompointsa=",bottompointsa)


    def perform_process_bottom(cps, config, points1,force):
        if not points1:
            return
        # Reach pre-contact first.
        communicate(
            cps=cps,
            config=config,
            point=points1[0],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['approach'],
            velocity_profile="robot",
            wait=True
        )
        # Engage force at first sanding point, then enable vibration at sweep point.
        communicate(
            cps=cps,
            config=config,
            point=points1[1],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['contact'],
            velocity_profile="sanding",
            wait=True
        )
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        for idx, point in enumerate(points1[2:], start=2):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        releaseForce(cps=cps, config=config, wait_for_blending=False)
    
    def perform_process_bottoma(cps, config, points1,force):
        if not points1:
            return
        # Reach pre-contact first.
        communicate(
            cps=cps,
            config=config,
            point=points1[0],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['approach'],
            velocity_profile="robot",
            wait=True
        )
        # Engage force at first sanding point, then enable vibration at sweep point.
        communicate(
            cps=cps,
            config=config,
            point=points1[1],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['contact'],
            velocity_profile="sanding",
            wait=True
        )
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        for idx, point in enumerate(points1[2:], start=2):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        releaseForce(cps=cps, config=config, wait_for_blending=False)
    
    def run_single_movement(robot_point, seventh_axis_point, cps, config):
        """Lift/position first, then start J7 non-blocking, then sync J7."""
        communicate(
            cps=cps,
            config=config,
            point=robot_point,
            seventh=-1,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True,
        )
    def perform_bottom_u_pass(cps, config, force):
        """Sand B then A as one continuous contact pass at the current J7 station."""
        if not bottompointsb or not bottompointsa:
            return

        communicate(
            cps=cps,
            config=config,
            point=bottompointsb[0],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['approach'],
            velocity_profile="robot",
            wait=True
        )
        communicate(
            cps=cps,
            config=config,
            point=bottompointsb[1],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['contact'],
            velocity_profile="sanding",
            wait=True
        )
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)

        continuous_points = bottompointsb[2:-1] + bottompointsa[1:-1]
        for idx, point in enumerate(continuous_points):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == len(continuous_points) - 1)
            )

        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        releaseForce(cps=cps, config=config, wait_for_blending=False)
        communicate(
            cps=cps,
            config=config,
            point=bottompointsa[-1],
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['return'],
            velocity_profile="robot",
            wait=True
        )

    if not bigger:
        #Bottom Cycle at x2 (U pattern: bottom -> top)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=speed_profile['travel'],velocity_profile="robot",wait=False)
        perform_bottom_u_pass(cps, config, force)

        #Shift to x1 and repeat U pattern
        run_single_movement(robot_point=prehoming, seventh_axis_point=x1, cps=cps, config=config)
        perform_bottom_u_pass(cps, config, force)

        #Last cycle
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)

    else:
        #Bottom Cycle at x3 (U pattern: bottom -> top)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
        communicate(cps=cps,config=config,seventh=x3,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=speed_profile['travel'],velocity_profile="robot",wait=False)
        perform_bottom_u_pass(cps, config, force)
        
        #Shift to x2 and repeat U pattern
        run_single_movement(robot_point=prehoming, seventh_axis_point=x2, cps=cps, config=config)
        perform_bottom_u_pass(cps, config, force)
        
        #Shift to x1 and repeat U pattern
        run_single_movement(robot_point=prehoming, seventh_axis_point=x1, cps=cps, config=config)
        perform_bottom_u_pass(cps, config, force)
        
        #Last cycle
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)  
    
    # #Bottom Cycle a
    # communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],speed=0.2,wait=True)
    # perform_process_bottoma(cps, config, points1=bottompointsa,force=force)
    # run_single_movement(robot_point=prehoming, seventh_axis_point=x1, cps=cps, config=config)
    # perform_process_bottoma(cps, config, points1=bottompointsa,force=force)

    # #last cycle
    # communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)


if __name__ == "__main__":
    model1bottomsmall(force=5)
