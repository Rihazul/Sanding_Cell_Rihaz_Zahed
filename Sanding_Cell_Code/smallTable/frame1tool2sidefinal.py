import sys
import os

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import math
import json
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus,putForceXplus,stop_requested
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
    """Loads runtime UI cycle data from cycleData.json."""
    with open('./configs/cycleData.json', 'r') as file:
        return json.load(file)


def _run_tool2_side_process(
    cps,
    config,
    points,
    force,
    sanding_speed,
    *,
    tcp,
    ucs,
    force_point=None,
    force_func=None,
    vibration_point=None,
):
    def abort_if_stopped():
        if not stop_requested():
            return
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        try:
            cps.HRIF_SetForceControlState(0, 0, 0)
        except Exception:
            pass
        config["logger"].warning(
            "[tool2-side] Emergency stop requested; aborting remaining Tool 2 side sequence."
        )
        raise RuntimeError("Tool 2 side operation stopped by emergency stop")

    robot_speed = float(load_json_config().get("robotSpeed", 0.9))
    abort_if_stopped()
    try:
        for point_index, point in enumerate(points):
            abort_if_stopped()
            if force_func is not None and force_point is not None and point is force_point:
                force_func(
                    cps=cps,
                    force=force,
                    tcp=tcp,
                    ucs=ucs,
                    config=config,
                )
                abort_if_stopped()
            if vibration_point is not None and point is vibration_point:
                abort_if_stopped()
                turn_vibration_on(cps)
                abort_if_stopped()
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed if point_index == 0 else sanding_speed,
                velocity_profile="robotspeed" if point_index == 0 else "sandingspeed",
                wait=True,
            )
            abort_if_stopped()

        waitForBlending(cps=cps, config=config)
        abort_if_stopped()
    finally:
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        if stop_requested():
            try:
                cps.HRIF_SetForceControlState(0, 0, 0)
            except Exception:
                pass
        else:
            releaseForce(cps=cps, config=config)


def _run_tool2_side_by_ylen(door_num, force, cps, small_runner, big_runner):
    ylen_data = get_y_values(door_num, default_on_error=True)
    ylen = ylen_data['ylen']
    print("ylen:", ylen)

    if ylen == "null":
        raise RuntimeError(
            f"Tool 2 side cannot run Door {door_num}: no scanned Y-length data is available."
        )

    if not isinstance(ylen, (int, float)):
        raise RuntimeError(
            f"Tool 2 side cannot run Door {door_num}: invalid ylen value {ylen!r}."
        )

    if ylen > 600:
        return big_runner(force, cps)
    return small_runner(force, cps)


def door1frametool2side(force,cps):
    def door1frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[(p4[0]-p1[0]),0,120,0,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-4,p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-4,p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-4,p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0]-4,p2[1]/2,5,0,0,0]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,5,0,0,0]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0]-4,p2[1]/2-2,5,0,0,0]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0]-4,p2[1]/2+2,5,0,0,0]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0]-4,p1[1]+2,5,0,0,0]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,0,0,0]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-4,p2[1]/2,5,0,0,0]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,5,0,0,0]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-4,p2[1]/2+2,5,0,0,0]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-4,p2[1],5,0,0,0]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],5,0,0,0]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,120,0,0,0]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,0,0,-90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,0,0,0]

        #top points
        toppoint1=[p2[0],p2[1]+4,5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,5,0,0,-90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,5,0,0,-90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,5,0,0,-90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[p1[0],p3[1]+4,5,0,0,-90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[p1[0],p3[1]+10,5,0,0,-90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[p1[0]+10,p3[1]+20,80,0,0,-180]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,0,0,-180]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+5,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+5,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+5,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+5,p3[1]/2,5,0,0,-180]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+5,p3[1]/2-2,5,0,0,-180]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+5,p3[1]/2+2,5,0,0,-180]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,5,0,0,-180]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[p1[0]+3,p3[1],5,0,0,-180]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[p1[0]+10,p3[1],5,0,0,-180]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[p1[0]+3,p3[1]-2,5,0,0,-180]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[p1[0]+3,p3[1]/2,5,0,0,-180]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[p1[0]+10,p3[1]/2,5,0,0,-180]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[p1[0]+10,p3[1]/2,100,0,0,-180]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]+30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1],80,0,0,0]
        extraleft=[p4[0]+20,p4[1],80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,0,0,0]

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint14,
                force_func=putForceXplus,
                vibration_point=rightpoint4,
            )
        def perform_process_upright(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uprightpoint4up,
                force_func=putForceXplus,
                vibration_point=uprightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_topup(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uptoppoint12,
                force_func=putForceYminus1,
                vibration_point=uptoppoint2,
            )
        def perform_process_upleft(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=upleftpoint3down,
                force_func=putForceXminus,
                vibration_point=upleftpoint5,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint5down,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        #Cycles for Big Door
        # #Bottom Cycles
        # moveOnlyJ6r(cps, 90, config)
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        # moveOnlyJ6r(cps, -270, config)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True,require_seventh_run_transition=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        # moveOnlyJ6r(cps, -90, config)

        # cps.HRIF_DisConnect(0)



    def door1frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[0,0,100,0,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,0,0,0]

        #top points
        toppoint1=[p2[0],p2[1]+4,5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,0,0,-90]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint12,
                force_func=putForceXplus,
                vibration_point=rightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint12,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        # #Right Cycle
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)        
        
        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # # #Joint 6 movement 
        # moveOnlyJ6r(cps, -326, config)

        # cps.HRIF_DisConnect(0)



    return _run_tool2_side_by_ylen(
        1,
        force,
        cps,
        door1frametool2sidesmall,
        door1frametool2sidebig,
    )


def door2frametool2side(force,cps):
    def door2frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[(p4[0]-p1[0]),0,120,0,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,5,0,0,0]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,5,0,0,0]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0],p2[1]/2-2,5,0,0,0]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,5,0,0,0]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,0,0,0]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,5,0,0,0]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,5,0,0,0]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,5,0,0,0]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],5,0,0,0]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],5,0,0,0]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,120,0,0,0]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,0,0,-90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,0,0,-90]

        #top points
        toppoint1=[p2[0],p2[1],5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,5,0,0,-90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,5,0,0,-90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,5,0,0,-90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[p1[0],p3[1]+4,5,0,0,-90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[p1[0],p3[1]+10,5,0,0,-90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[p1[0]+10,p3[1],80,0,0,-180]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,0,0,-180]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+2,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+3,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,5,0,0,-180]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,5,0,0,-180]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+3,p3[1]/2+2,5,0,0,-180]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,5,0,0,-180]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[p1[0]+3,p3[1],5,0,0,-180]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[p1[0]+10,p3[1],5,0,0,-180]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[p1[0]+3,p3[1]-2,5,0,0,-180]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[p1[0]+3,p3[1]/2,5,0,0,-180]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[p1[0]+10,p3[1]/2,5,0,0,-180]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[p1[0]+10,p3[1]/2,100,0,0,-180]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,0,0,0]

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint14,
                force_func=putForceXplus,
                vibration_point=rightpoint4,
            )
        def perform_process_upright(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uprightpoint4up,
                force_func=putForceXplus,
                vibration_point=uprightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_topup(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uptoppoint12,
                force_func=putForceYminus1,
                vibration_point=uptoppoint2,
            )
        def perform_process_upleft(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=upleftpoint3down,
                force_func=putForceXminus,
                vibration_point=upleftpoint5,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint5down,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        #Cycles for Big Door
        # #Bottom Cycles
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True,require_seventh_run_transition=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # cps.HRIF_DisConnect(0)

    def door2frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[0,0,100,0,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-20,p2[1]+30,80,0,0,0]

        #top points
        toppoint1=[p2[0],p2[1]+4,5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,0,0,-90]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+5,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+5,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint12,
                force_func=putForceXplus,
                vibration_point=rightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint12,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        # #Right Cycle
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # #Joint 6 movement 

        # cps.HRIF_DisConnect(0)

    return _run_tool2_side_by_ylen(
        2,
        force,
        cps,
        door2frametool2sidesmall,
        door2frametool2sidebig,
    )


def door3frametool2side(force,cps):
    def door3frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[(p4[0]-p1[0]),0,120,0,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,5,0,0,0]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,5,0,0,0]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0],p2[1]/2-2,5,0,0,0]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,5,0,0,0]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,0,0,0]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,5,0,0,0]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,5,0,0,0]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,5,0,0,0]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],5,0,0,0]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],5,0,0,0]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,120,0,0,0]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,0,0,-90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+10,5,0,0,-90]

        #top points
        toppoint1=[p2[0],p2[1],5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,5,0,0,-90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,5,0,0,-90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,5,0,0,-90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[p1[0],p3[1]+4,5,0,0,-90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[p1[0],p3[1]+10,5,0,0,-90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[p1[0]+10,p3[1],80,0,0,-180]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,0,0,-180]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+2,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+2,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,5,0,0,-180]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,5,0,0,-180]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+3,p3[1]/2+2,5,0,0,-180]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,5,0,0,-180]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[p1[0]+3,p3[1],5,0,0,-180]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[p1[0]+10,p3[1],5,0,0,-180]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[p1[0]+3,p3[1]-2,5,0,0,-180]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[p1[0]+3,p3[1]/2,5,0,0,-180]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[p1[0]+10,p3[1]/2,5,0,0,-180]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[p1[0]+10,p3[1]/2,100,0,0,-180]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,0,0,0]

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint14,
                force_func=putForceXplus,
                vibration_point=rightpoint4,
            )
        def perform_process_upright(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uprightpoint4up,
                force_func=putForceXplus,
                vibration_point=uprightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_topup(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uptoppoint12,
                force_func=putForceYminus1,
                vibration_point=uptoppoint2,
            )
        def perform_process_upleft(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=upleftpoint3down,
                force_func=putForceXminus,
                vibration_point=upleftpoint5,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint5down,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        #Cycles for Big Door
        # #Bottom Cycles
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True,require_seventh_run_transition=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # cps.HRIF_DisConnect(0)

    def door3frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[0,0,100,0,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-10,p2[1]+30,80,0,0,0]

        #top points
        toppoint1=[p2[0],p2[1]+4,5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+30,80,0,0,-90]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint12,
                force_func=putForceXplus,
                vibration_point=rightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint12,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        # #Right Cycle
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # #Joint 6 movement 

        # cps.HRIF_DisConnect(0)

    return _run_tool2_side_by_ylen(
        3,
        force,
        cps,
        door3frametool2sidesmall,
        door3frametool2sidebig,
    )


def door4frametool2side(force,cps):
    def door4frametool2sidebig(force,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[(p4[0]-p1[0]),0,120,0,0,-180]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0],p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0],p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        #RightPoints for Big Door
        rightpoint4=[p2[0],p2[1]/2,5,0,0,0]
        print("rightpoint4:", rightpoint4)
        rightpoint4pre=[p2[0]-10,p2[1]/2,5,0,0,0]
        print("rightpoint4pre:", rightpoint4pre)
        rightpoint4down=[p2[0]+2,p2[1]/2-2,5,0,0,0]
        print("rightpoint4down:", rightpoint4down)
        rightpoint4up=[p2[0],p2[1]/2+2,5,0,0,0]
        print("rightpoint4up:", rightpoint4up)
        rightpoint14=[p1[0],p1[1]+2,5,0,0,0]
        print("rightpoint14:",rightpoint14)

        #Right Points for Big Door All

        righthalfpointsdown=[prerightpoint1,rightpoint1,rightpoint14,rightpoint4,rightpoint4pre]
        print("righthalfpointsdown:", righthalfpointsdown)

        #Right Position Homing
        rightpositionhoming=[p2[0]-10,p2[1]/2,80,0,0,0]

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #UpRightPoints
        distance=(p4[0]-p1[0])/2
        print("distance:", distance)
        uprightpoint4=[-distance*2-3,p2[1]/2,5,0,0,0]
        print("uprightpoint4:", uprightpoint4)
        uprightpoint4pre=[-distance*2-10,p2[1]/2,5,0,0,0]
        print("uprightpoint4pre:", uprightpoint4pre)
        uprightpoint4up=[-distance*2-3,p2[1]/2+2,5,0,0,0]
        print("uprightpoint4up:", uprightpoint4up)
        uprightpoint2=[-distance*2-3,p2[1],5,0,0,0]
        print("uprightpoint2:", uprightpoint2)
        uprightpoint2pre=[-distance*2-10,p2[1],5,0,0,0]
        print("uprightpoint2pre:", uprightpoint2pre)
        
        uprightpoints=[uprightpoint4pre,uprightpoint4,uprightpoint4up,uprightpoint2,uprightpoint2pre]
        print("uprightpoints:", uprightpoints)

        
        #prehome Up right point
        prehomeuprightpoint=[(-distance*2)-10,p2[1]/2,120,0,0,0]
        print("prehomeuprightpoint:", prehomeuprightpoint)
        #ExtraUpright
        extraupright=[(-distance*2)-10,p2[1]+30,80,0,0,-90]
        print("extraupright:",extraupright)

        #Extraright
        extraright=[p2[0]-20,p2[1]+10,5,0,0,-90]

        #top points
        toppoint1=[p2[0],p2[1],5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1],5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1],5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Up points for Big Door
        uptoppoint1=[-distance*2,p2[1]+4,5,0,0,-90]
        print("uptoppoint1:", uptoppoint1)
        preuptoppoint1=[-distance*2,p2[1]+10,5,0,0,-90]
        print("preuptoppoint1:", preuptoppoint1)
        uptoppoint12=[-distance*2+2,p2[1]+4,5,0,0,-90]
        print("uptoppoint12:", uptoppoint12)
        uptoppoint2=[p1[0],p3[1]+4,5,0,0,-90]
        print("uptoppoint2:", uptoppoint2)
        preuptoppoint2=[p1[0],p3[1]+10,5,0,0,-90]
        print("preuptoppoint2:", preuptoppoint2)

        uptoppoints=[preuptoppoint1,uptoppoint1,uptoppoint12,uptoppoint2,preuptoppoint2]
        print("uptoppoints:", uptoppoints)

        #Up extra top points
        upextratop=[p1[0]+10,p3[1],80,0,0,-180]
        print("upextratop:", upextratop)

        #Extratop
        extratop=[p3[0]+20,p3[1]+30,80,0,0,-180]
        print("extratop:", extratop)

        #LeftPoints 
        leftpoint1=[p3[0]+2,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+3,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+2,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        #Lelft Points for Big Door
        leftpoint5=[p3[0]+3,p3[1]/2,5,0,0,-180]
        print("leftpoint5:", leftpoint5)
        leftpoint5down=[p3[0]+3,p3[1]/2-2,5,0,0,-180]
        print("leftpoint5down:", leftpoint5down)
        leftpoint5up=[p3[0]+2,p3[1]/2+2,5,0,0,-180]
        print("leftpoint5up:", leftpoint5up)
        leftpoint5pre=[p3[0]+10,p3[1]/2,5,0,0,-180]
        print("leftpoint5pre:", leftpoint5pre)

        #Left Points for Bigdoor
        lefthalfpointsdown=[leftpoint5pre,leftpoint5,leftpoint5down,leftpoint2,preleftpoint2]
        print("lefthalfpointsdown:", lefthalfpointsdown)

        #Left Points for Bigdoor UP
        upleftpoint3=[p1[0]+3,p3[1],5,0,0,-180]
        print("upleftpoint3:", upleftpoint3)
        upleftpoint3pre=[p1[0]+10,p3[1],5,0,0,-180]
        print("upleftpoint3pre:", upleftpoint3pre)
        upleftpoint3down=[p1[0]+3,p3[1]-2,5,0,0,-180]
        print("upleftpoint3down:", upleftpoint3down)
        upleftpoint5=[p1[0]+3,p3[1]/2,5,0,0,-180]
        print("upleftpoint5:", upleftpoint5)
        upleftpoint5pre=[p1[0]+10,p3[1]/2,5,0,0,-180]
        print("upleftpoint5pre:", upleftpoint5pre)

        upleftpoints=[upleftpoint3pre,upleftpoint3,upleftpoint3down,upleftpoint5,upleftpoint5pre]
        print("upleftpoints:", upleftpoints)

        #Extrea Upleft Points Homing
        upleftextrahome=[p1[0]+10,p3[1]/2,100,0,0,-180]
        print("upleftextrahome:", upleftextrahome)


        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Extra for Bottom
        bottomextra=[p1[0]-10,p1[1]-2-30,80,0,0,0]

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint14,
                force_func=putForceXplus,
                vibration_point=rightpoint4,
            )
        def perform_process_upright(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uprightpoint4up,
                force_func=putForceXplus,
                vibration_point=uprightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_topup(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=uptoppoint12,
                force_func=putForceYminus1,
                vibration_point=uptoppoint2,
            )
        def perform_process_upleft(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=upleftpoint3down,
                force_func=putForceXminus,
                vibration_point=upleftpoint5,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint5down,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        #Cycles for Big Door
        # #Bottom Cycles
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_left(cps, config, points1=lefthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=bottomextra,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=righthalfpointsdown,force=force)
        communicate(cps=cps,config=config,point=rightpositionhoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top cycles
        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True,require_seventh_run_transition=True)
        communicate(cps=cps,config=config,point=prehomeuprightpoint,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upright(cps, config, points1=uprightpoints,force=force)
        communicate(cps=cps,config=config,point=extraupright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_topup(cps, config, points1=uptoppoints,force=force)
        communicate(cps=cps,config=config,point=upextratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_upleft(cps, config, points1=upleftpoints,force=force)
        communicate(cps=cps,config=config,point=upleftextrahome,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # cps.HRIF_DisConnect(0)

    def door4frametool2sidesmall(force,cps):
        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])
        json_config = load_json_config()
        robot_speed = float(json_config.get("robotSpeed", 0.9))
        sanding_speed = float(json_config.get("sandingSpeed", 0.75))

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
        prehoming=[0,0,100,0,0,0]
        print("prehoming:",prehoming)

        #Right Points
        rightpoint1=[p1[0]-3,p1[1],5,0,0,0]
        print("rightpoint1:", rightpoint1)
        prerightpoint1=[p1[0]-10,p1[1],5,0,0,0]
        print("prerightpoint1:", prerightpoint1)
        rightpoint2=[p2[0]-3,p2[1],5,0,0,0]
        print("rightpoint2:", rightpoint2)
        prerightpoint2=[p2[0]-10,p2[1],5,0,0,0]
        print("prerightpoint2:", prerightpoint2)
        rightpoint12=[p1[0]-3,p1[1]+2,5,0,0,0]
        print("rightpoint12:", rightpoint12)

        rightpoints=[prerightpoint1,rightpoint1,rightpoint12,rightpoint2,prerightpoint2]
        print("rightpoints:", rightpoints)

        #Extraright
        extraright=[p2[0]-10,p2[1]+10,80,0,0,0]

        #top points
        toppoint1=[p2[0],p2[1]+4,5,0,0,-90]
        print("toppoint1:", toppoint1)
        pretopoint1=[p2[0],p2[1]+10,5,0,0,-90]
        print("pretopoint1:", pretopoint1)
        toppoint2=[p3[0],p3[1]+4,5,0,0,-90]
        print("toppoint2:", toppoint2)
        pretopoint2=[p3[0],p3[1]+10,5,0,0,-90]
        print("pretopoint2:", pretopoint2)
        toppoint12=[p2[0]+2,p2[1]+4,5,0,0,-90]
        print("toppoint12:", toppoint12)

        toppoints=[pretopoint1,toppoint1,toppoint12,toppoint2,pretopoint2]
        print("toppoints:", toppoints)

        #Extratop
        extratop=[p3[0]+10,p3[1]+20,80,0,0,-90]
        print("extratop:", extratop)

        #LeftPoints
        leftpoint1=[p3[0]+4,p3[1],5,0,0,-180]
        print("leftpoint1:", leftpoint1)
        preleftpoint1=[p3[0]+1+10,p3[1],5,0,0,-180]
        print("preleftpoint1:", preleftpoint1)
        leftpoint2=[p4[0]+4,p4[1],5,0,0,-180]
        print("leftpoint2:", leftpoint2)
        preleftpoint2=[p4[0]+1+10,p4[1],5,0,0,-180]
        print("preleftpoint2:", preleftpoint2)
        leftpoint12=[p3[0]+4,p3[1]-2,5,0,0,-180]
        print("leftpoint12:", leftpoint12)

        leftpoints=[preleftpoint1,leftpoint1,leftpoint12,leftpoint2,preleftpoint2]
        print("leftpoints:", leftpoints)

        #Extra left
        extraleft0=[p4[0]+20,p4[1]-30,80,0,0,-180]
        extraleft1=[p4[0]+20,p4[1]-30,80,0,0,-90]
        extraleft2=[p4[0]+20,p4[1]-30,80,0,0,0]
        extraleft=[p4[0]+20,p4[1]-30,80,0,0,90]

        #Bottom Points
        bottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("bottompoint1:", bottompoint1)
        prebottompoint1=[p4[0],p4[1]-12,5,0,0,90]
        print("prebottompoint1:", prebottompoint1)
        bottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("bottompoint2:", bottompoint2)
        prebottompoint2=[p1[0],p1[1]-12,5,0,0,90]
        print("prebottompoint2:", prebottompoint2)
        bottompoint12=[p4[0]-2,p4[1]-12,5,0,0,90]
        print("bottompoint12:", bottompoint12)
        bottompoints=[prebottompoint1,bottompoint1,bottompoint12,bottompoint2,prebottompoint2]
        print("bottompoints:", bottompoints)

        #Posthoming
        posthoming=[p1[0],p1[1]-10,100,0,0,90]




        def perform_process_right(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=rightpoint12,
                force_func=putForceXplus,
                vibration_point=rightpoint2,
            )
        def perform_process_top(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=toppoint12,
                force_func=putForceYminus1,
                vibration_point=toppoint2,
            )
        def perform_process_left(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=leftpoint12,
                force_func=putForceXminus,
                vibration_point=leftpoint2,
            )
        def perform_process_bottom(cps, config, points1,force):
            return _run_tool2_side_process(
                cps,
                config,
                points1,
                force,
                sanding_speed,
                tcp=config["coords"]["tcptool2plane1"],
                ucs=config["coords"]["ucsTable1"],
                force_point=bottompoint12,
                force_func=putForceYplus1,
                vibration_point=bottompoint2,
            )
        # #Right Cycle
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=True,require_seventh_ok=True)
        communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)
        perform_process_right(cps, config, points1=rightpoints,force=force)
        communicate(cps=cps,config=config,point=extraright,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Top Cycle
        perform_process_top(cps, config, points1=toppoints,force=force)
        communicate(cps=cps,config=config,point=extratop,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Right Cycle
        perform_process_left(cps, config, points1=leftpoints,force=force)
        communicate(cps=cps,config=config,point=extraleft0,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        moveOnlyJ6r(cps, 270, config)
        # communicate(cps=cps,config=config,point=extraleft1,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft2,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=False)
        # communicate(cps=cps,config=config,point=extraleft,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        #Bottom Cycle
        perform_process_bottom(cps, config, points1=bottompoints,force=force)
        communicate(cps=cps,config=config,point=posthoming,tcp=config['coords']['tcptool2plane1'],ucs=config['coords']['ucsTable1'],seventh=-1,speed=robot_speed,velocity_profile="robotspeed",wait=True)

        # #Joint 6 movement 

        # cps.HRIF_DisConnect(0)

    return _run_tool2_side_by_ylen(
        4,
        force,
        cps,
        door4frametool2sidesmall,
        door4frametool2sidebig,
    )


if __name__ == "__main__":
    
    door1frametool2side(force=5)
    door2frametool2side(force=5)
    door3frametool2side(force=5)
    door4frametool2side(force=5)
   
    



