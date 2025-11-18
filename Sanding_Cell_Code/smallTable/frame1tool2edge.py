import sys
import os

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import math
import time
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus,putForceXplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading
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
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def door1frametool2edge(force):


    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)
    
    #Main Points
    p1 = get_outer_corner_point(1, 0)
    print("p1:", p1)
    p2 = get_outer_corner_point(1, 1)
    print("p2:", p2)
    p3= get_outer_corner_point(1, 2)
    print("p3:", p3)
    p4 = get_outer_corner_point(1, 3)
    print("p4:", p4)

    #7th axis postion
    x1=get_door_position(1)
    print("x1:", x1)
    #prehoming
    prehoming=[0,0,100,180,0,-180]
    print("prehoming:",prehoming)

    #Right Points
    rightpoint1=[p1[0]+40,p1[1],10,180,-22,-180]
    print("rightpoint1:", rightpoint1)
    prerightpoint1=[p1[0]-10,p1[1],10,180,-22,-180]
    print("prerightpoint1:", prerightpoint1)
    rightpoint2=[p2[0]+40,p2[1],10,180,-22,-180]
    print("rightpoint2:", rightpoint2)
    prerightpoint2=[p2[0]-10,p2[1],10,180,-22,-180]
    print("prerightpoint2:", prerightpoint2)
    rightpoint12=[p1[0]+40,p1[1]+2,10,180,-22,-180]
    print("rightpoint12:", rightpoint12)

    rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
    print("rightpoints:", rightpoints)

    #Extraright
    extraright=[p2[0]-10,p2[1]+10,10,180,0,90]

    #top points
    toppoint1=[p2[0],p2[1]-40,10,180,-22,90]
    print("toppoint1:", toppoint1)
    pretopoint1=[p2[0],p2[1]+10,10,180,-22,90]
    print("pretopoint1:", pretopoint1)
    toppoint2=[p3[0],p3[1]-40,10,180,-22,90]
    print("toppoint2:", toppoint2)
    pretopoint2=[p3[0],p3[1]+10,10,180,-22,90]
    print("pretopoint2:", pretopoint2)
    toppoint12=[p2[0]+2,p2[1]-40,10,180,-22,90]
    print("toppoint12:", toppoint12)

    toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
    print("toppoints:", toppoints)

    #Extratop
    extratop=[p3[0]+10,p3[1]+10,10,180,0,0]
    print("extratop:", extratop)

    #LeftPoints
    leftpoint1=[p3[0]+2-40,p3[1],10,180,-22,0]
    print("leftpoint1:", leftpoint1)
    preleftpoint1=[p3[0]+1+10,p3[1],10,180,-22,0]
    print("preleftpoint1:", preleftpoint1)
    leftpoint2=[p4[0]+2-40,p4[1],10,180,-22,0]
    print("leftpoint2:", leftpoint2)
    preleftpoint2=[p4[0]+1+10,p4[1],10,180,-22,0]
    print("preleftpoint2:", preleftpoint2)
    leftpoint12=[p3[0]+2-40,p3[1]-2,10,180,-22,0]
    print("leftpoint12:", leftpoint12)

    leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
    print("leftpoints:", leftpoints)

    #Extra left
    extraleft=[p4[0]+10,p4[1]-10,10,180,0,-90]

    #Bottom Points
    bottompoint1=[p4[0],p4[1]-2+40,10,180,-22,-90]
    print("bottompoint1:", bottompoint1)
    prebottompoint1=[p4[0],p4[1]-10,10,180,-22,-90]
    print("prebottompoint1:", prebottompoint1)
    bottompoint2=[p1[0],p1[1]-2+40,10,180,-22,-90]
    print("bottompoint2:", bottompoint2)
    prebottompoint2=[p1[0],p1[1]-10,10,180,-22,-90]
    print("prebottompoint2:", prebottompoint2)
    bottompoint12=[p4[0]-2,p4[1]-2+40,10,180,-22,-90]
    print("bottompoint12:", bottompoint12)
    bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
    print("bottompoints:", bottompoints)

    #Posthoming
    posthoming=[p1[0],p1[1]-10,100,180,0,-90]




    def perform_process_right(cps, config, points1,force):
        # Vibration on
        # turn_vibration_on(cps)
        
        # Force Control Activated
        #putForceZplus(
            #cps=cps,
            #force=15,
            #tcp=config['coords']['tcpReal'],
            #ucs=config['coords']['ucsTable1'],
            #config=config
        #)
        
        # Communicate to each point in points1
        for point in points1:
            if point==rightpoint12:putForceXplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==rightpoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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
        # Vibration on
        # turn_vibration_on(cps)
        
        # Force Control Activated
        #putForceZplus(
            #cps=cps,
            #force=15,
            #tcp=config['coords']['tcpReal'],
            #ucs=config['coords']['ucsTable1'],
            #config=config
        #)
        
        # Communicate to each point in points1
        for point in points1:
            if point==toppoint12:putForceYminus1(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==toppoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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

    def perform_process_left(cps, config, points1,force):
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
            if point==leftpoint12:putForceXminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==leftpoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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
            if point==bottompoint12:putForceYplus1(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==bottompoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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

    # #Right Cycle
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
    perform_process_right(cps, config, points1=rightpoints,force=force)
    communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Top Cycle
    perform_process_top(cps, config, points1=toppoints,force=force)
    communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Right Cycle
    perform_process_left(cps, config, points1=leftpoints,force=force)
    communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Bottom Cycle
    perform_process_bottom(cps, config, points1=bottompoints,force=force)
    communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

     # #Joint 6 movement 
    moveOnlyJ6r(cps, -326, config)

def door2frametool2edge(force):


    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)
    
    #Main Points
    p1 = get_outer_corner_point(2, 0)
    print("p1:", p1)
    p2 = get_outer_corner_point(2, 1)
    print("p2:", p2)
    p3= get_outer_corner_point(2, 2)
    print("p3:", p3)
    p4 = get_outer_corner_point(2, 3)
    print("p4:", p4)

    #7th axis postion
    x1=get_door_position(2)
    print("x1:", x1)
    #prehoming
    prehoming=[0,0,100,180,0,-180]
    print("prehoming:",prehoming)

    #Right Points
    rightpoint1=[p1[0]+40,p1[1],10,180,-22,-180]
    print("rightpoint1:", rightpoint1)
    prerightpoint1=[p1[0]-10,p1[1],10,180,-22,-180]
    print("prerightpoint1:", prerightpoint1)
    rightpoint2=[p2[0]+40,p2[1],10,180,-22,-180]
    print("rightpoint2:", rightpoint2)
    prerightpoint2=[p2[0]-10,p2[1],10,180,-22,-180]
    print("prerightpoint2:", prerightpoint2)
    rightpoint12=[p1[0]+40,p1[1]+2,10,180,-22,-180]
    print("rightpoint12:", rightpoint12)

    rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
    print("rightpoints:", rightpoints)

    #Extraright
    extraright=[p2[0]-10,p2[1]+10,10,180,0,90]

    #top points
    toppoint1=[p2[0],p2[1]-40,10,180,-22,90]
    print("toppoint1:", toppoint1)
    pretopoint1=[p2[0],p2[1]+10,10,180,-22,90]
    print("pretopoint1:", pretopoint1)
    toppoint2=[p3[0],p3[1]-40,10,180,-22,90]
    print("toppoint2:", toppoint2)
    pretopoint2=[p3[0],p3[1]+10,10,180,-22,90]
    print("pretopoint2:", pretopoint2)
    toppoint12=[p2[0]+2,p2[1]-40,10,180,-22,90]
    print("toppoint12:", toppoint12)

    toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
    print("toppoints:", toppoints)

    #Extratop
    extratop=[p3[0]+10,p3[1]+10,10,180,0,0]
    print("extratop:", extratop)

    #LeftPoints
    leftpoint1=[p3[0]+2-40,p3[1],10,180,-22,0]
    print("leftpoint1:", leftpoint1)
    preleftpoint1=[p3[0]+1+10,p3[1],10,180,-22,0]
    print("preleftpoint1:", preleftpoint1)
    leftpoint2=[p4[0]+2-40,p4[1],10,180,-22,0]
    print("leftpoint2:", leftpoint2)
    preleftpoint2=[p4[0]+1+10,p4[1],10,180,-22,0]
    print("preleftpoint2:", preleftpoint2)
    leftpoint12=[p3[0]+2-40,p3[1]-2,10,180,-22,0]
    print("leftpoint12:", leftpoint12)

    leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
    print("leftpoints:", leftpoints)

    #Extra left
    extraleft=[p4[0]+10,p4[1]-10,10,180,0,-90]

    #Bottom Points
    bottompoint1=[p4[0],p4[1]-2+40,10,180,-22,-90]
    print("bottompoint1:", bottompoint1)
    prebottompoint1=[p4[0],p4[1]-10,10,180,-22,-90]
    print("prebottompoint1:", prebottompoint1)
    bottompoint2=[p1[0],p1[1]-2+40,10,180,-22,-90]
    print("bottompoint2:", bottompoint2)
    prebottompoint2=[p1[0],p1[1]-10,10,180,-22,-90]
    print("prebottompoint2:", prebottompoint2)
    bottompoint12=[p4[0]-2,p4[1]-2+40,10,180,-22,-90]
    print("bottompoint12:", bottompoint12)
    bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
    print("bottompoints:", bottompoints)

    #Posthoming
    posthoming=[p1[0],p1[1]-10,100,180,0,-90]




    def perform_process_right(cps, config, points1,force):
        # Vibration on
        # turn_vibration_on(cps)
        
        # Force Control Activated
        #putForceZplus(
            #cps=cps,
            #force=15,
            #tcp=config['coords']['tcpReal'],
            #ucs=config['coords']['ucsTable1'],
            #config=config
        #)
        
        # Communicate to each point in points1
        for point in points1:
            if point==rightpoint12:putForceXplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==rightpoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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
        # Vibration on
        # turn_vibration_on(cps)
        
        # Force Control Activated
        #putForceZplus(
            #cps=cps,
            #force=15,
            #tcp=config['coords']['tcpReal'],
            #ucs=config['coords']['ucsTable1'],
            #config=config
        #)
        
        # Communicate to each point in points1
        for point in points1:
            if point==toppoint12:putForceYminus1(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==toppoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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

    def perform_process_left(cps, config, points1,force):
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
            if point==leftpoint12:putForceXminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==leftpoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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
            if point==bottompoint12:putForceYplus1(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool2plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config
            )
            if point==bottompoint2:turn_vibration_on(cps)
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool2plane1'],
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

    # #Right Cycle
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)
    perform_process_right(cps, config, points1=rightpoints,force=force)
    communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Top Cycle
    perform_process_top(cps, config, points1=toppoints,force=force)
    communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Right Cycle
    perform_process_left(cps, config, points1=leftpoints,force=force)
    communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

    #Bottom Cycle
    perform_process_bottom(cps, config, points1=bottompoints,force=force)
    communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.2,wait=True)

     # #Joint 6 movement 
    moveOnlyJ6r(cps, -326, config)



if __name__ == "__main__":
    
    #door1frametool2edge(force=5)
    door2frametool2edge(force=5)
    