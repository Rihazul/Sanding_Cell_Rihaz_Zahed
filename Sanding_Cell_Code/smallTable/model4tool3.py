# testcommu.py

import sys
import os
import time
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus,putForceXminus
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

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config


             
def smalldoor1tool3(z,cps):
    def smalldoortool3big(z,cps):
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

        prehoming=[0,0,100,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(1, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(1, 1)
        print("b10:", b10)
        b20 = get_outer_corner_point(1, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(1, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)
        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
        #tpoints=[pretpoint5,tpoint5,tpoint3,tpoint0,tpoint4,pretpoint4]

        #Top half cycles

        
        xlength=(point3[0]-point1[0])/2
        print("xlength:",xlength)
        conxb=point1[0]
        print("conxb",conxb)
        x2=conxb+x1+xlength
        print("x2:",x2)

        ttpoint2=[xlength-Tooloffset+5, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint2:", ttpoint2)
        ttpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint5:", ttpoint5)
        ttpoint52=[xlength-Tooloffset+5, point2[1]/2-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint52:", ttpoint52)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        prehomingtop=[xlength/2-Tooloffset+5, point2[1]/2-Tooloffset, 20, 0, 0, 0]
        print("prehomingtop:",prehomingtop)

        toppoints=[prettpoint5,ttpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        

        
        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint40:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint0:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==ttpoint52:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==ttpoint2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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





        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)


        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        communicate(cps=cps,config=config,point=prehomingtop,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_top(cps, config, points1=toppoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    def smalldoortool3small(z,cps):
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

        prehoming=[0,0,80,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
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
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+2, point0[2], 0, 0, 0]
        print("tpoint01:", tpoint01)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint01:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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



        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor2tool3(z,cps):
    def smalldoortool3big(z,cps):
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

        prehoming=[0,0,100,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(2, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(2, 1)
        print("b10:", b10)
        b20 = get_outer_corner_point(2, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(2, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)
        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
        #tpoints=[pretpoint5,tpoint5,tpoint3,tpoint0,tpoint4,pretpoint4]

        #Top half cycles

        
        xlength=(point3[0]-point1[0])/2
        print("xlength:",xlength)
        conxb=point1[0]
        print("conxb",conxb)
        x2=conxb+x1+xlength
        print("x2:",x2)

        ttpoint2=[xlength-Tooloffset+5, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint2:", ttpoint2)
        ttpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint5:", ttpoint5)
        ttpoint52=[xlength-Tooloffset+5, point2[1]/2-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint52:", ttpoint52)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        prehomingtop=[xlength/2-Tooloffset+5, point2[1]/2-Tooloffset, 20, 0, 0, 0]
        print("prehomingtop:",prehomingtop)

        toppoints=[prettpoint5,ttpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        

        
        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint40:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint0:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==ttpoint52:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==ttpoint2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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





        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)


        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        communicate(cps=cps,config=config,point=prehomingtop,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_top(cps, config, points1=toppoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    def smalldoortool3small(z,cps):
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

        prehoming=[0,0,80,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(2, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(2, 1)
        print("b10:", b10)
        b20= get_outer_corner_point(2, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(2, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+2, point0[2], 0, 0, 0]
        print("tpoint01:", tpoint01)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint01:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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



        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor3tool3(z,cps):
    def smalldoortool3big(z,cps):
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

        prehoming=[0,0,100,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(3, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(3, 1)
        print("b10:", b10)
        b20 = get_outer_corner_point(3, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(3, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)
        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
        #tpoints=[pretpoint5,tpoint5,tpoint3,tpoint0,tpoint4,pretpoint4]

        #Top half cycles

        
        xlength=(point3[0]-point1[0])/2
        print("xlength:",xlength)
        conxb=point1[0]
        print("conxb",conxb)
        x2=conxb+x1+xlength
        print("x2:",x2)

        ttpoint2=[xlength-Tooloffset+5, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint2:", ttpoint2)
        ttpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint5:", ttpoint5)
        ttpoint52=[xlength-Tooloffset+5, point2[1]/2-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint52:", ttpoint52)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        prehomingtop=[xlength/2-Tooloffset+5, point2[1]/2-Tooloffset, 20, 0, 0, 0]
        print("prehomingtop:",prehomingtop)

        toppoints=[prettpoint5,ttpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        

        
        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint40:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint0:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==ttpoint52:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==ttpoint2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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





        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)


        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        communicate(cps=cps,config=config,point=prehomingtop,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_top(cps, config, points1=toppoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    def smalldoortool3small(z,cps):
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

        prehoming=[0,0,80,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(3, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(3, 1)
        print("b10:", b10)
        b20= get_outer_corner_point(3, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(3, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+2, point0[2], 0, 0, 0]
        print("tpoint01:", tpoint01)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint01:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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



        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor4tool3(z,cps):
    def smalldoortool3big(z,cps):
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

        prehoming=[0,0,100,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(4, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(4, 1)
        print("b10:", b10)
        b20 = get_outer_corner_point(4, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(4, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)
        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, 10, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
        #tpoints=[pretpoint5,tpoint5,tpoint3,tpoint0,tpoint4,pretpoint4]

        #Top half cycles

        
        xlength=(point3[0]-point1[0])/2
        print("xlength:",xlength)
        conxb=point1[0]
        print("conxb",conxb)
        x2=conxb+x1+xlength
        print("x2:",x2)

        ttpoint2=[xlength-Tooloffset+5, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint2:", ttpoint2)
        ttpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint5:", ttpoint5)
        ttpoint52=[xlength-Tooloffset+5, point2[1]/2-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint52:", ttpoint52)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, 10, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        prehomingtop=[xlength/2-Tooloffset+5, point2[1]/2-Tooloffset, 20, 0, 0, 0]
        print("prehomingtop:",prehomingtop)

        toppoints=[prettpoint5,ttpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        

        
        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint40:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint0:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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

        def perform_process_top(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==ttpoint52:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==ttpoint2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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





        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)


        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=0.2,wait=True)
        communicate(cps=cps,config=config,point=prehomingtop,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_top(cps, config, points1=toppoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    def smalldoortool3small(z,cps):
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

        prehoming=[0,0,80,0,0,0]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b00 = get_outer_corner_point(4, 0)
        print("b00:", b00)
        b10 = get_outer_corner_point(4, 1)
        print("b10:", b10)
        b20= get_outer_corner_point(4, 2)
        print("b20:", b20)
        b30 = get_outer_corner_point(4, 3)
        print("b30:", b30)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        
        #Offsets
        #Processing Points
        offset3d = 48
        print("offset3d:", offset3d)
        b0 = [b00[0]+offset3d, b00[1]+offset3d+10, b00[2], b00[3], b00[4], 0]
        print("b0:", b0)
        b1 = [b10[0]+offset3d, b10[1]-offset3d, b10[2], b10[3], b10[4], 0]
        print("b1:", b1)
        b2 = [b20[0]-offset3d, b20[1]-offset3d, b20[2], b20[3], b20[4], 0]
        print("b2:", b2)
        b3 = [b30[0]-offset3d, b30[1]+offset3d+10, b30[2], b30[3], b30[4], 0]
        print("b3", b3)
    
    
        Tooloffset=28


        point0=[ b0[0], b0[1], z+1, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], z+1, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], z+1, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], z+1, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+2, point0[2], 0, 0, 0]
        print("tpoint01:", tpoint01)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, 10, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

        def perform_process_bottom(cps, config, points1):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==tpoint01:putForceZminus(
                cps=cps,
                force=5,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
                )
                if point==tpoint1:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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



        def perform_process_left(cps, config, points1):
            # Vibration on
            turn_vibration_on(cps)
            
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
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.4,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=tpoints)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=speeed,wait=True)
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


if __name__ == "__main__":
    smalldoor1tool3(z=0)
    # smalldoor2tool3(z=-10)
    # smalldoor3tool3(z=-10)
    # smalldoor4tool3(z=-10)

   
    