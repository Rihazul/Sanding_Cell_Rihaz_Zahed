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

def door1frametool2side(force,cps):
    def door1frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
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

        #7th axis position for the top
        x2=x1+(p4[0]-p1[0])
        print("x2:", x2)

        #prehoming
        prehoming=[(p4[0]-p1[0]),0,100,180,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-4,p1[1],10,180,0,180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-4,p2[1],10,180,0,180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-4,p1[1]+2,10,180,0,180]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0]-4,p2[1]/2,10,180,0,180]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,10,180,0,180]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0]-4,p2[1]/2-2,10,180,0,180]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0]-4,p2[1]/2+2,10,180,0,180]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0]-4,p1[1]+2,10,180,0,180]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,180,0,180]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-4,p2[1]/2,10,180,0,180]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,10,180,0,180]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-4,p2[1]/2+2,10,180,0,180]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-4,p2[1],10,180,0,180]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],10,180,0,180]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,100,180,0,180]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,180,0,90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1]+4,10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,10,180,0,90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,10,180,0,90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,10,180,0,90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[0,p3[1]+4,10,180,0,90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[0,p3[1]+10,10,180,0,90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[0+10,p3[1]+20,80,180,0,0]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+5,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+5,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+5,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+5,p3[1]/2,10,180,0,0]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+5,p3[1]/2-2,10,180,0,0]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+5,p3[1]/2+2,10,180,0,0]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,10,180,0,0]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[0+5,p3[1],10,180,0,0]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[0+10,p3[1],10,180,0,0]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[0+5,p3[1]-2,10,180,0,0]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[0+5,p3[1]/2,10,180,0,0]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[0+10,p3[1]/2,10,180,0,0]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[0+10,p3[1]/2,100,180,0,0]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+20,p4[1]-30,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-4,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-4,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-4,10,180,0,-90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,180,0,180]

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
                if point==rightpoint14:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==rightpoint4:turn_vibration_on(cps)
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

        def perform_process_upright(cps, config, points1,force):
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
                if point==uprightpoint4up:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uprightpoint2:turn_vibration_on(cps)
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

        def perform_process_topup(cps, config, points1,force):
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
                if point==uptoppoint12:putForceYminus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uptoppoint2:turn_vibration_on(cps)
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

        
        def perform_process_upleft(cps, config, points1,force):
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
                if point==upleftpoint3down:putForceXminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==upleftpoint5:turn_vibration_on(cps)
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
                if point==leftpoint5down:putForceXminus(
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


        #Cycles for Big Door
        # #Bottom Cycles
        moveOnlyJ6r(cps, 90, config)
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -270, config)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -90, config)

        # cps.HRIF_DisConnect(0)



    def door1frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
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
        rightpoint1=[p1[0]-3,p1[1],10,180,0,-180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,-180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],10,180,0,-180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,-180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,10,180,0,-180]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1]+4,10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+10,p4[1]-30,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-4,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-4,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-4,10,180,0,-90]
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
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.6,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        # #Joint 6 movement 
        moveOnlyJ6r(cps, -326, config)

        # cps.HRIF_DisConnect(0)



    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            door1frametool2sidebig(force,cps)
        else:
            door1frametool2sidesmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def door2frametool2side(force,cps):
    def door2frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
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

        #7th axis position for the top
        x2=x1+(p4[0]-p1[0])
        print("x2:", x2)

        #prehoming
        prehoming=[(p4[0]-p1[0]),0,100,180,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],10,180,0,180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],10,180,0,180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,10,180,0,180]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,10,180,0,180]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0],p2[1]/2-2,10,180,0,180]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,10,180,0,180]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,180,0,180]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,10,180,0,180]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,10,180,0,180]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,10,180,0,180]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],10,180,0,180]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],10,180,0,180]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,100,180,0,180]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,180,0,90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1],10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,10,180,0,90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,10,180,0,90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,10,180,0,90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[0,p3[1]+4,10,180,0,90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[0,p3[1]+10,10,180,0,90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[0+10,p3[1],80,180,0,0]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+2,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+3,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,10,180,0,0]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,10,180,0,0]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+3,p3[1]/2+2,10,180,0,0]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,10,180,0,0]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[0+3,p3[1],10,180,0,0]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[0+10,p3[1],10,180,0,0]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[0+3,p3[1]-2,10,180,0,0]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[0+3,p3[1]/2,10,180,0,0]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[0+10,p3[1]/2,10,180,0,0]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[0+10,p3[1]/2,100,180,0,0]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+20,p4[1]-40,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-2,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-2,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-2,10,180,0,-90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,180,0,180]

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
                if point==rightpoint14:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==rightpoint4:turn_vibration_on(cps)
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

        def perform_process_upright(cps, config, points1,force):
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
                if point==uprightpoint4up:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uprightpoint2:turn_vibration_on(cps)
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

        def perform_process_topup(cps, config, points1,force):
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
                if point==uptoppoint12:putForceYminus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uptoppoint2:turn_vibration_on(cps)
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

        
        def perform_process_upleft(cps, config, points1,force):
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
                if point==upleftpoint3down:putForceXminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==upleftpoint5:turn_vibration_on(cps)
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
                if point==leftpoint5down:putForceXminus(
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


        #Cycles for Big Door
        # #Bottom Cycles
        moveOnlyJ6r(cps, 90, config)
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -270, config)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -90, config)

        # cps.HRIF_DisConnect(0)

    def door2frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
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
        rightpoint1=[p1[0]-3,p1[1],10,180,0,-180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,-180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],10,180,0,-180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,-180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,10,180,0,-180]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1]+4,10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+5,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+5,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+10,p4[1]-40,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-4,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-4,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-4,10,180,0,-90]
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
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.6,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        # #Joint 6 movement 
        moveOnlyJ6r(cps, -326, config)

        # cps.HRIF_DisConnect(0)

    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            door2frametool2sidebig(force,cps)
        else:
            door2frametool2sidesmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")



def door3frametool2side(force,cps):
    def door3frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p1 = get_outer_corner_point(3, 0)
        print("p1:", p1)
        p2 = get_outer_corner_point(3, 1)
        print("p2:", p2)
        p3= get_outer_corner_point(3, 2)
        print("p3:", p3)
        p4 = get_outer_corner_point(3, 3)
        print("p4:", p4)

        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)

        #7th axis position for the top
        x2=x1+(p4[0]-p1[0])
        print("x2:", x2)

        #prehoming
        prehoming=[(p4[0]-p1[0]),0,100,180,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],10,180,0,180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],10,180,0,180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,10,180,0,180]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,10,180,0,180]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0],p2[1]/2-2,10,180,0,180]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,10,180,0,180]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,180,0,180]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,10,180,0,180]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,10,180,0,180]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,10,180,0,180]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],10,180,0,180]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],10,180,0,180]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,100,180,0,180]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,180,0,90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+10,10,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1],10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,10,180,0,90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,10,180,0,90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,10,180,0,90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[0,p3[1]+4,10,180,0,90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[0,p3[1]+10,10,180,0,90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[0+10,p3[1],80,180,0,0]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+2,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+2,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,10,180,0,0]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,10,180,0,0]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+3,p3[1]/2+2,10,180,0,0]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,10,180,0,0]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[0+3,p3[1],10,180,0,0]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[0+10,p3[1],10,180,0,0]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[0+3,p3[1]-2,10,180,0,0]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[0+3,p3[1]/2,10,180,0,0]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[0+10,p3[1]/2,10,180,0,0]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[0+10,p3[1]/2,100,180,0,0]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+20,p4[1]-40,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-2,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-2,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-2,10,180,0,-90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,180,0,180]

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
                if point==rightpoint14:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==rightpoint4:turn_vibration_on(cps)
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

        def perform_process_upright(cps, config, points1,force):
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
                if point==uprightpoint4up:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uprightpoint2:turn_vibration_on(cps)
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

        def perform_process_topup(cps, config, points1,force):
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
                if point==uptoppoint12:putForceYminus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uptoppoint2:turn_vibration_on(cps)
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

        
        def perform_process_upleft(cps, config, points1,force):
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
                if point==upleftpoint3down:putForceXminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==upleftpoint5:turn_vibration_on(cps)
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
                if point==leftpoint5down:putForceXminus(
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


        #Cycles for Big Door
        # #Bottom Cycles
        moveOnlyJ6r(cps, 90, config)
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -270, config)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -90, config)

        # cps.HRIF_DisConnect(0)

    def door3frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p1 = get_outer_corner_point(3, 0)
        print("p1:", p1)
        p2 = get_outer_corner_point(3, 1)
        print("p2:", p2)
        p3= get_outer_corner_point(3, 2)
        print("p3:", p3)
        p4 = get_outer_corner_point(3, 3)
        print("p4:", p4)

        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,100,180,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],10,180,0,-180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,-180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],10,180,0,-180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,-180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,10,180,0,-180]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-10,p2[1]+30,80,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1]+4,10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+10,p4[1]-40,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-4,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-4,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-4,10,180,0,-90]
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
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.6,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        # #Joint 6 movement 
        moveOnlyJ6r(cps, -326, config)

        # cps.HRIF_DisConnect(0)

    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            door3frametool2sidebig(force,cps)
        else:
            door3frametool2sidesmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def door4frametool2side(force,cps):
    def door4frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p1 = get_outer_corner_point(4, 0)
        print("p1:", p1)
        p2 = get_outer_corner_point(4, 1)
        print("p2:", p2)
        p3= get_outer_corner_point(4, 2)
        print("p3:", p3)
        p4 = get_outer_corner_point(4, 3)
        print("p4:", p4)

        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)

        #7th axis position for the top
        x2=x1+(p4[0]-p1[0])
        print("x2:", x2)

        #prehoming
        prehoming=[(p4[0]-p1[0]),0,100,180,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],10,180,0,180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],10,180,0,180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,10,180,0,180]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,10,180,0,180]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0]+2,p2[1]/2-2,10,180,0,180]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,10,180,0,180]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,10,180,0,180]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,180,0,180]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,10,180,0,180]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,10,180,0,180]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,10,180,0,180]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],10,180,0,180]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],10,180,0,180]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,100,180,0,180]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,180,0,90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+10,10,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1],10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,10,180,0,90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,10,180,0,90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,10,180,0,90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[0,p3[1]+4,10,180,0,90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[0,p3[1]+10,10,180,0,90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[0+10,p3[1],80,180,0,0]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+2,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+2,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,10,180,0,0]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,10,180,0,0]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+2,p3[1]/2+2,10,180,0,0]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,10,180,0,0]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[0+3,p3[1],10,180,0,0]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[0+10,p3[1],10,180,0,0]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[0+3,p3[1]-2,10,180,0,0]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[0+3,p3[1]/2,10,180,0,0]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[0+10,p3[1]/2,10,180,0,0]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[0+10,p3[1]/2,100,180,0,0]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+20,p4[1]-40,80,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-2,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-2,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-2,10,180,0,-90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-10,10,180,0,180]

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
                if point==rightpoint14:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==rightpoint4:turn_vibration_on(cps)
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

        def perform_process_upright(cps, config, points1,force):
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
                if point==uprightpoint4up:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uprightpoint2:turn_vibration_on(cps)
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

        def perform_process_topup(cps, config, points1,force):
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
                if point==uptoppoint12:putForceYminus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==uptoppoint2:turn_vibration_on(cps)
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

        
        def perform_process_upleft(cps, config, points1,force):
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
                if point==upleftpoint3down:putForceXminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool2plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==upleftpoint5:turn_vibration_on(cps)
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
                if point==leftpoint5down:putForceXminus(
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


        #Cycles for Big Door
        # #Bottom Cycles
        moveOnlyJ6r(cps, 90, config)
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -270, config)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.7,wait=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.7,wait=True)
        moveOnlyJ6r(cps, -90, config)

        # cps.HRIF_DisConnect(0)

    def door4frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p1 = get_outer_corner_point(4, 0)
        print("p1:", p1)
        p2 = get_outer_corner_point(4, 1)
        print("p2:", p2)
        p3= get_outer_corner_point(4, 2)
        print("p3:", p3)
        p4 = get_outer_corner_point(4, 3)
        print("p4:", p4)

        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        #prehoming
        prehoming=[0,0,100,180,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],10,180,0,-180]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],10,180,0,-180]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],10,180,0,-180]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],10,180,0,-180]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,10,180,0,-180]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-10,p2[1]+10,10,180,0,90]

        #top points
        toppoint1=[p2[0],p2[1]+4,10,180,0,90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,10,180,0,90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,10,180,0,90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,10,180,0,90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,10,180,0,90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+10,10,180,0,0]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],10,180,0,0]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],10,180,0,0]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],10,180,0,0]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],10,180,0,0]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,10,180,0,0]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft=[p4[0]+10,p4[1]-20,10,180,0,-90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-4,10,180,0,-90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-10,10,180,0,-90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-4,10,180,0,-90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-10,10,180,0,-90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-4,10,180,0,-90]
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
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=0.6,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=0.6,wait=True)

        # #Joint 6 movement 
        moveOnlyJ6r(cps, -326, config)

        # cps.HRIF_DisConnect(0)

    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            door4frametool2sidebig(force,cps)
        else:
            door4frametool2sidesmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

    
if __name__ == "__main__":
    
    door1frametool2side(force=5)
    door2frametool2side(force=5)
    door3frametool2side(force=5)
    door4frametool2side(force=5)
   
    