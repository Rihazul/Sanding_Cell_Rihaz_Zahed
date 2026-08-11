# testcommu.py

import sys
import os
import time
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus,putForceXminus,stop_requested
from smallTable.stop_lift import stop_lift_to_preheight_tableA
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

FORCE_APPROACH_Z_MM = 15.0
TOOL1_CONTACT_FORCE_THRESHOLD_N = 2.0


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config





def _load_cycle_config():
    """Loads cycleData.json for runtime speed settings."""
    try:
        with open('./configs/cycleData.json', 'r') as file:
            return json.load(file)
    except Exception:
        return {}


def _get_motion_speeds(config):
    """Return (robot_speed, sanding_speed) with safe fallbacks."""
    cycle_cfg = _load_cycle_config()
    ui_cfg = config.get('UI', {}) if isinstance(config, dict) else {}

    def _to_float(value, default):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    robot_default = ui_cfg.get('robotSpeed', 0.7)
    sand_default = ui_cfg.get('sandSpeed', 0.6)

    robot_speed = _to_float(cycle_cfg.get('robotSpeed', robot_default), robot_default)
    sanding_speed = _to_float(cycle_cfg.get('sandingSpeed', sand_default), sand_default)

    return robot_speed, sanding_speed


def _resolve_force(user_force, default_force=5.0):
    try:
        if user_force is None:
            return float(default_force)
        return float(user_force)
    except (TypeError, ValueError):
        return float(default_force)


def _run_tool1_force_locked_path(cps, config, points1, force, sanding_speed, *, label="tool1", cycles=1):
    """Run Tool 1 path while keeping Z under force control, like frame/zigzag/edge."""
    robot_speed, _ = _get_motion_speeds(config)
    points = [list(point) for point in (points1 or [])]
    if len(points) < 2:
        print(f"[Tool1ForcePath] {label}: skipped, not enough points")
        return

    approach_point = list(points[0])
    force_control_z = float(approach_point[2])
    exit_point = None
    try:
        if float(points[-1][2]) >= force_control_z:
            exit_point = list(points[-1])
    except (TypeError, ValueError, IndexError):
        exit_point = None

    contact_points = []
    for point in points[1:]:
        try:
            if float(point[2]) < force_control_z:
                locked_point = list(point)
                locked_point[2] = force_control_z
                contact_points.append(locked_point)
        except (TypeError, ValueError, IndexError):
            print(f"[Tool1ForcePath] {label}: skipped invalid point {point}")

    if not contact_points:
        print(f"[Tool1ForcePath] {label}: skipped, no contact points")
        return

    print(
        f"[Tool1ForcePath] {label}: approach={approach_point} "
        f"contact_points={len(contact_points)} force_control_z={force_control_z} "
        f"robot_speed={robot_speed} sanding_speed={sanding_speed}"
    )
    communicate(
        cps=cps,
        config=config,
        point=approach_point,
        tcp=config['coords']['tcptool1plane1'],
        ucs=config['coords']['ucsTable1'],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )

    force_start_point = list(contact_points[0])
    force_start_point[3:6] = [0, 0, 0]
    print(f"[Tool1ForcePath] {label}: force start {force_start_point}")
    communicate(
        cps=cps,
        config=config,
        point=force_start_point,
        tcp=config['coords']['tcptool1plane1'],
        ucs=config['coords']['ucsTable1'],
        seventh=-1,
        speed=sanding_speed,
        velocity_profile="sandingspeed",
        speed_mode="linear",
        wait=True,
    )

    force_applied = False
    vibration_on = False
    try:
        force_ok = putForceZminus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            config=config,
            search_linear_velocity=5.0,
            contact_force_threshold=TOOL1_CONTACT_FORCE_THRESHOLD_N,
        )
        if not force_ok:
            raise RuntimeError(f"[Tool1ForcePath] {label}: failed to establish Z- force contact.")
        force_applied = True

        turn_vibration_on(cps)
        vibration_on = True

        cycle_count = max(1, int(cycles))
        for cycle_index in range(cycle_count):
            if stop_requested():
                raise RuntimeError("[Tool1] Stop requested.")
            reverse_cycle = cycle_index % 2 == 1
            sanding_points = contact_points[-2::-1] if reverse_cycle else contact_points[1:]
            direction = "reverse" if reverse_cycle else "forward"
            print(
                f"[Tool1ForcePath] {label}: continuous cycle "
                f"{cycle_index + 1}/{cycle_count} direction={direction}"
            )
            for index, point in enumerate(sanding_points, start=1):
                is_last = index == len(sanding_points)
                print(
                    f"[Tool1ForcePath] {label}: move {index}/{len(sanding_points)} "
                    f"direction={direction} {point}"
                )
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    speed_mode="linear",
                    wait=is_last,
                )

            waitForBlending(cps=cps, config=config)
    finally:
        if stop_requested():
            # Operator Stop: release force and lift straight up to pre-height with no force,
            # so the tool releases the surface immediately (no drag/marks). From there the
            # operator can re-send the task or run homing.
            stop_lift_to_preheight_tableA(
                cps,
                config,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                preheight_z_mm=FORCE_APPROACH_Z_MM,
                robot_speed=robot_speed,
                last_xy=(approach_point[0], approach_point[1]),
            )
        else:
            if vibration_on:
                turn_vibration_off(cps)
            if force_applied:
                releaseForce(cps=cps, config=config)
            if exit_point is not None:
                print(f"[Tool1ForcePath] {label}: exit lift {exit_point}")
                communicate(
                    cps=cps,
                    config=config,
                    point=exit_point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=robot_speed,
                    velocity_profile="robotspeed",
                    wait=True,
                )




def _is_big_door(door_number):
    """Return True when the external contour requires two J7 positions."""
    x_data = get_x_values(door_number, default_on_error=True)
    y_data = get_y_values(door_number, default_on_error=True)
    xlen = x_data.get('xlen') if isinstance(x_data, dict) else None
    ylen = y_data.get('ylen') if isinstance(y_data, dict) else None

    print(f"door {door_number} xlen: {xlen}")
    print(f"door {door_number} ylen: {ylen}")

    if not isinstance(ylen, (int, float)):
        print("No valid door size data available - using small-door path")
        return False

    use_split_path = ylen > 600
    print(
        f"door {door_number} Model C external path: "
        f"{'two J7 positions' if use_split_path else 'one J7 position'}"
    )
    return use_split_path


def smalldoor1tool3(z, cps, force=None, cycles=1):
    applied_force = _resolve_force(force)
    def smalldoortool3big(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(1, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(1, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(1, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
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
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)

        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
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
        print("ttpoint5:", ttpoint5)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        toppoints=[prettpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        
        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        def perform_process_top(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_top",
                cycles=cycles,
            )
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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False)
        perform_process_bottom(cps, config, points1=tpoints)

        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False)
        perform_process_top(cps, config, points1=toppoints)
        # cps.HRIF_DisConnect(0)

    def smalldoortool3small(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(1, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(1, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(1, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(1, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(1)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+1, point0[2], 0, 0, 0]
        # print("tpoint1:", tpoint1)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)
        # cps.HRIF_DisConnect(0)

    try:
        if _is_big_door(1):
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    finally:
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        try:
            releaseForce(cps=cps, config=load_config())
        except Exception:
            pass

def smalldoor2tool3(z, cps, force=None, cycles=1):
    applied_force = _resolve_force(force)
    def smalldoortool3big(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(2, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(2, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(2, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(2, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
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
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)

        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
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
        print("ttpoint5:", ttpoint5)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        toppoints=[prettpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)
        
        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        def perform_process_top(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_top",
                cycles=cycles,
            )
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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)

        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False)
        perform_process_top(cps, config, points1=toppoints)
        # cps.HRIF_DisConnect(0)

    def smalldoortool3small(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]


        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(2, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(2, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(2, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(2, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(2)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+1, point0[2], 0, 0, 0]
        # print("tpoint1:", tpoint1)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)



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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)
        # cps.HRIF_DisConnect(0)

    try:
        if _is_big_door(2):
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    finally:
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        try:
            releaseForce(cps=cps, config=load_config())
        except Exception:
            pass

def smalldoor3tool3(z, cps, force=None, cycles=1):
    applied_force = _resolve_force(force)
    def smalldoortool3big(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(3, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(3, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(3, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(3, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
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
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)

        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
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
        print("ttpoint5:", ttpoint5)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        toppoints=[prettpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        
        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        def perform_process_top(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_top",
                cycles=cycles,
            )
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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)

        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False)
        perform_process_top(cps, config, points1=toppoints)
        # cps.HRIF_DisConnect(0)

    def smalldoortool3small(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(3, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(3, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(3, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(3, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(3)
        print("x1:", x1)
        
        #Offsets
    
    
        Tooloffset=0


        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+1, point0[2], 0, 0, 0]
        # print("tpoint1:", tpoint1)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)

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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)
        # cps.HRIF_DisConnect(0)

    try:
        if _is_big_door(3):
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    finally:
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        try:
            releaseForce(cps=cps, config=load_config())
        except Exception:
            pass

def smalldoor4tool3(z, cps, force=None, cycles=1):
    applied_force = _resolve_force(force)
    def smalldoortool3big(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(4, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(4, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(4, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(4, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
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
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        #Y length
        ylenth=point1[1]-point0[1]
        print("ylenth:",ylenth)

        #Point Extra Bottom half Cycles
        tpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint4:",tpoint4)
        tpoint40=[point0[0]+Tooloffset, ylenth/2+Tooloffset-2, point0[2], 0, 0, 0]
        print("tpoint40:",tpoint40)

        pretpoint4=[point0[0]+Tooloffset, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("tpoint4:",pretpoint4)

        tpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint5:",tpoint5)
        pretpoint5=[point3[0]-Tooloffset+5, ylenth/2+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint5;",pretpoint5)

        tpoints=[pretpoint4,tpoint40,tpoint0,tpoint3,tpoint5,pretpoint5]
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
        print("ttpoint5:", ttpoint5)
        prettpoint5=[xlength-Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint2:", prettpoint5)
        ttpoint1=[-xlength+Tooloffset, point2[1]-Tooloffset+2, point2[2]+1, 0, 0, 0]
        print("ttpoint1:", ttpoint1)
        ttpoint4=[-xlength+Tooloffset, point2[1]/2-Tooloffset, point2[2]+1, 0, 0, 0]
        print("ttpoint4:", ttpoint4)
        prettpoint4=[-xlength+Tooloffset+5, point2[1]/2-Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("prettpoint4:", prettpoint4)

        toppoints=[prettpoint5,ttpoint52,ttpoint2,ttpoint1,ttpoint4,prettpoint4]
        print("toppoints:",toppoints)

        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        def perform_process_top(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_top",
                cycles=cycles,
            )
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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release Force Control

        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)

        communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False)
        perform_process_top(cps, config, points1=toppoints)
        # cps.HRIF_DisConnect(0)

    def smalldoortool3small(z,cps):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])
        robot_speed, sanding_speed = _get_motion_speeds(config)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        
        # t1=[88.831,93.955,-10,0,0,0]
        # pret1=[88.831,93.955,10,0,0,0]
        # t2=[88.831,415.232,-10,0,0,0]
        # pret2=[88.831,415.232,10,0,0,0]
        # t12=[88.831,95.955,-10,0,0,0]

        # bottompoints=[pret1,t1,t12,t2]
        # print("bottompoints:", bottompoints)

        #Points Actual
        #Main Points
        b0 = get_outer_corner_point(4, 0)
        print("b0:", b0)
        b1 = get_outer_corner_point(4, 1)
        print("b1:", b1)
        b2= get_outer_corner_point(4, 2)
        print("b2:", b2)
        b3 = get_outer_corner_point(4, 3)
        print("b3", b3)
        #7th axis postion
        x1=get_door_position(4)
        print("x1:", x1)
        
        #Offsets
        Tooloffset=0

        point0=[ b0[0], b0[1], 0, 0, 0, 0]
        print("point0:", point0)
        point1=[ b1[0], b1[1], 0, 0, 0, 0]
        print("point1:", point1)
        point2=[ b2[0], b2[1], 0, 0, 0, 0]
        print("point2:", point2)
        point3=[ b3[0], b3[1], 0, 0, 0, 0]
        print("point3:", point3)
    
        #Points with Tool Offset
        tpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, point0[2], 0, 0, 0]
        print("tpoint0:", tpoint0)
        tpoint01=[point0[0]+Tooloffset, point0[1]+Tooloffset+1, point0[2], 0, 0, 0]
        # print("tpoint1:", tpoint1)
        tpoint1=[point1[0]+Tooloffset, point1[1]-Tooloffset+2, point1[2], 0, 0, 0]
        print("tpoint1:", tpoint1)
        tpoint2=[point2[0]-Tooloffset+5, point2[1]-Tooloffset+2, point2[2], 0, 0, 0]
        print("tpoint2:", tpoint2)
        tpoint3=[point3[0]-Tooloffset+5, point3[1]+Tooloffset, point3[2], 0, 0, 0]
        print("tpoint3:", tpoint3)
        pretpoint0=[point0[0]+Tooloffset, point0[1]+Tooloffset, FORCE_APPROACH_Z_MM, 0, 0, 0]
        print("pretpoint0:", pretpoint0)

        tpoints=[pretpoint0,tpoint01,tpoint1,tpoint2,tpoint3,tpoint0,pretpoint0]
        print("tpoints:",tpoints)



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
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control

        def perform_process_bottom(cps, config, points1):
            _run_tool1_force_locked_path(
                cps=cps,
                config=config,
                points1=points1,
                force=applied_force,
                sanding_speed=sanding_speed,
                label="external_bottom",
                cycles=cycles,
            )
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=robot_speed,velocity_profile="robotspeed",wait=False,require_seventh_ok=True)
        perform_process_bottom(cps, config, points1=tpoints)
        # cps.HRIF_DisConnect(0)

    try:
        if _is_big_door(4):
            smalldoortool3big(z,cps)
        else:
            smalldoortool3small(z,cps)
    finally:
        try:
            turn_vibration_off(cps)
        except Exception:
            pass
        try:
            releaseForce(cps=cps, config=load_config())
        except Exception:
            pass

if __name__ == "__main__":
    smalldoor1tool3(z=-10)
    smalldoor2tool3(z=-10)
    smalldoor3tool3(z=-10)
    smalldoor4tool3(z=-10)
