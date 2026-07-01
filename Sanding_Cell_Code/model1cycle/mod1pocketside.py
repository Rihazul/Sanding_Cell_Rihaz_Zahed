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
from model1cycle.mod1bottomb import model1bottombig
from model1cycle.mod1bottoms import model1bottomsmall








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








def model1pocket1side(force,cps):
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


   
   
    z=-4
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z=-4
    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    sanding_speed = float(json_config['sandingSpeed'])
    speed_profile = {
        'travel': speeed,
        'approach': speeed,
        'return': speeed,
        'tool_change': speeed,
        'contact': sanding_speed,
    }
    if speed_profile['contact'] <= 0:
        speed_profile['contact'] = speed_profile['travel']
        print("[model1pocket1side] sandingSpeed<=0, using travel speed for contact moves")
    safe_approach_speed = speed_profile['approach'] if speed_profile['approach'] > 0 else 0.2
    safe_contact_speed = speed_profile['contact'] if speed_profile['contact'] > 0 else safe_approach_speed

   
     
    # Calculate new points outside the pocket
    p5 = [p13[0] + outeroffset, p13[1] - outeroffset, z, 180, 0, 0]
    p6 = [p14[0] + outeroffset, p14[1] + outeroffset, z, 180, 0, 0]
    p7 = [p15[0] - outeroffset, p15[1] + outeroffset, z, 180, 0, 0]
    p8 = [p16[0] - outeroffset, p16[1] - outeroffset, z, 180, 0, 0]


    print("offset p5=", p5)
    print("offset p6=", p6)
    print("offset p7=", p7)
    print("offset p8=", p8)


    #7th axis movement
    x1=p8[0]
    print("x1:", x1)
    seventhdistance=(p5[0] - p8[0])/2
    print("seventhdistance:", seventhdistance)
    x2=x1+seventhdistance
    print("x2:", x2)
    x3=x1+seventhdistance*2
    print("x3:", x3)




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -25, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance, point5[1], -25, 180, 0, 0]
    bpoint2nd=[distance, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    mid_x=(bpoint1st[0] + bpoint2nd[0]) / 2
    bpointmiddle=[mid_x, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)
    prebpointmiddle=[bpointmiddle[0], bpointmiddle[1], -25, 180, 0, 0]
    print("prebpointmiddle:", prebpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[bpoint1st, bpointmiddle, bpoint2nd]
    print("bpoints:", bpoints)
    bottom_right_to_mid=[bpoint1st, bpointmiddle]
    bottom_mid_to_left=[bpointmiddle, bpoint2nd]

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [distance, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [distance, point5[1], -25, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [distance, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [distance, point6[1], -25, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [distance, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[lpoint1st, lpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [distance, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -25, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [mid_x, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    pretpointmiddle = [topointmiddle[0], topointmiddle[1], -25, 180, 0, 0]
    print("pretpointmiddle:", pretpointmiddle)
    tpoint2nd = [distance, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [distance, point7[1], -25, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -50, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[tpoint1st, topointmiddle, tpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_right=[topointmiddle, tpoint1st]
    top_left_to_mid=[tpoint2nd, topointmiddle]

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -25, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -25, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[rpoint1st, rpoint2nd]
    print("rightpoints:",rightpoints)

    # Shifted points for pass 2 (centered on mid_x)
    shift_x = mid_x
    def shift_point(pt):
        return [pt[0] - shift_x, pt[1], pt[2], pt[3], pt[4], pt[5]]

    prebpointmiddle_shifted = shift_point(prebpointmiddle)
    bottom_mid_to_left_shifted = [shift_point(p) for p in bottom_mid_to_left]
    leftpoints_shifted = [shift_point(p) for p in leftpoints]
    top_left_to_mid_shifted = [shift_point(p) for p in top_left_to_mid]
    pretpointmiddle_shifted = shift_point(pretpointmiddle)
    htoppoints_shifted = shift_point(htoppoints)

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -25, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -25, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)





    force_active = False

    def start_force_if_needed():
        nonlocal force_active
        if force_active:
            return
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        force_active = True

    force_approach_lift = 20.0

    def move_to_contact_point(first_point):
        # Table B Z+ searches downward, so prepoint lift is negative Z.
        force_prepoint = list(first_point)
        force_prepoint[2] = first_point[2] - force_approach_lift
        communicate(
            cps=cps,
            config=config,
            point=force_prepoint,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=safe_approach_speed,
            velocity_profile="robot",
            wait=True
        )
        return True
    def perform_process_bottom(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_left(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_top(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_right(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_middle(cps, config, points1,force):
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
        for idx, point in enumerate(points1):
            if point == pmiddile21:
                if not move_to_contact_point(point):
                    return
                putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )

            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == 0 or idx == len(points1) - 1)
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        # Release Force Control
        import time

    

    def move_axis_then_robot(to_point, seventh_axis_point, cps, config):
        """
        Move 7th axis and robot prepoint transition simultaneously from current pose.
        Then hold at prepoint until 7th-axis target is fully reached.
        """
        # Start 7th-axis move without blocking so arm move can run simultaneously.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=False
        )
        # Move to next prepoint while 7th axis is traveling.
        communicate(
            cps=cps,
            config=config,
            point=to_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
        # Ensure 7th-axis move is complete before entering contact from prepoint.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
        
    # Ensure we always start from a released force/vibration state.
    releaseForce(cps=cps, config=config, wait_for_blending=False)
    turn_vibration_off(cps)

    # Pass 1: top -> right -> bottom (x1)
    move_axis_then_robot(to_point=pretpointmiddle, seventh_axis_point=x1, cps=cps, config=config)
    communicate(cps=cps,config=config,point=top_mid_to_right[0],tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['approach'],velocity_profile="robot",wait=True)
    # Vibration is enabled on first real contact inside start_force_if_needed().
    # turn_tool_spin_on(cps)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force)
    perform_process_right(cps, config, points1=rightpoints,force=force)
    perform_process_bottom(cps, config, points1=bottom_right_to_mid,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False

    # Pass 2: bottom -> left -> top (x2, centered)
    move_axis_then_robot(to_point=prebpointmiddle_shifted, seventh_axis_point=x2, cps=cps, config=config)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left_shifted,force=force)
    perform_process_left(cps, config, points1=leftpoints_shifted,force=force)
    perform_process_top(cps, config, points1=top_left_to_mid_shifted,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=pretpointmiddle_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    communicate(cps=cps,config=config,point=htoppoints_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    # turn_tool_spin_off(cps)

def model1pocket2side(force,cps):
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


   
   
    z=-4
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z=-4

    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    sanding_speed = float(json_config['sandingSpeed'])
    speed_profile = {
        'travel': speeed,
        'approach': speeed,
        'return': speeed,
        'tool_change': speeed,
        'contact': sanding_speed,
    }
    if speed_profile['contact'] <= 0:
        speed_profile['contact'] = speed_profile['travel']
        print("[model1pocket2side] sandingSpeed<=0, using travel speed for contact moves")
    safe_approach_speed = speed_profile['approach'] if speed_profile['approach'] > 0 else 0.2
    safe_contact_speed = speed_profile['contact'] if speed_profile['contact'] > 0 else safe_approach_speed

   
     
    # Calculate new points outside the pocket
    p5 = [p9[0] + outeroffset, p9[1] - outeroffset, z, 180, 0, 0]
    p6 = [p10[0] + outeroffset, p10[1] + outeroffset, z, 180, 0, 0]
    p7 = [p11[0] - outeroffset, p11[1] + outeroffset, z, 180, 0, 0]
    p8 = [p12[0] - outeroffset, p12[1] - outeroffset, z, 180, 0, 0]


    print("offset p5=", p5)
    print("offset p6=", p6)
    print("offset p7=", p7)
    print("offset p8=", p8)


    #7th axis movement
    x1=p8[0]
    print("x1:", x1)
    seventhdistance=(p5[0] - p8[0])/2
    print("seventhdistance:", seventhdistance)
    x2=x1+seventhdistance
    print("x2:", x2)
    x3=x1+seventhdistance*2
    print("x3:", x3)




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -25, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance, point5[1], -25, 180, 0, 0]
    bpoint2nd=[distance, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    mid_x=(bpoint1st[0] + bpoint2nd[0]) / 2
    bpointmiddle=[mid_x, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)
    prebpointmiddle=[bpointmiddle[0], bpointmiddle[1], -25, 180, 0, 0]
    print("prebpointmiddle:", prebpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[bpoint1st, bpointmiddle, bpoint2nd]
    print("bpoints:", bpoints)
    bottom_right_to_mid=[bpoint1st, bpointmiddle]
    bottom_mid_to_left=[bpointmiddle, bpoint2nd]

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [distance, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [distance, point5[1], -25, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [distance, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [distance, point6[1], -25, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [distance, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[lpoint1st, lpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [distance, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -25, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [mid_x, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    pretpointmiddle = [topointmiddle[0], topointmiddle[1], -25, 180, 0, 0]
    print("pretpointmiddle:", pretpointmiddle)
    tpoint2nd = [distance, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [distance, point7[1], -25, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -50, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[tpoint1st, topointmiddle, tpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_right=[topointmiddle, tpoint1st]
    top_left_to_mid=[tpoint2nd, topointmiddle]

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -25, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -25, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[rpoint1st, rpoint2nd]
    print("rightpoints:",rightpoints)

    # Shifted points for pass 2 (centered on mid_x)
    shift_x = mid_x
    def shift_point(pt):
        return [pt[0] - shift_x, pt[1], pt[2], pt[3], pt[4], pt[5]]

    prebpointmiddle_shifted = shift_point(prebpointmiddle)
    bottom_mid_to_left_shifted = [shift_point(p) for p in bottom_mid_to_left]
    leftpoints_shifted = [shift_point(p) for p in leftpoints]
    top_left_to_mid_shifted = [shift_point(p) for p in top_left_to_mid]
    pretpointmiddle_shifted = shift_point(pretpointmiddle)
    htoppoints_shifted = shift_point(htoppoints)

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -25, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -25, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)





    force_active = False

    def start_force_if_needed():
        nonlocal force_active
        if force_active:
            return
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        force_active = True

    force_approach_lift = 20.0

    def move_to_contact_point(first_point):
        # Table B Z+ searches downward, so prepoint lift is negative Z.
        force_prepoint = list(first_point)
        force_prepoint[2] = first_point[2] - force_approach_lift
        communicate(
            cps=cps,
            config=config,
            point=force_prepoint,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=safe_approach_speed,
            velocity_profile="robot",
            wait=True
        )
        return True
    def perform_process_bottom(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_left(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_top(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_right(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_middle(cps, config, points1,force):
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
        for idx, point in enumerate(points1):
            if point == pmiddile21:
                if not move_to_contact_point(point):
                    return
                putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )

            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == 0 or idx == len(points1) - 1)
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def move_axis_then_robot(to_point, seventh_axis_point, cps, config):
        """
        Move 7th axis and robot prepoint transition simultaneously from current pose.
        Then hold at prepoint until 7th-axis target is fully reached.
        """
        # Start 7th-axis move without blocking so arm move can run simultaneously.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=False
        )
        # Move to next prepoint while 7th axis is traveling.
        communicate(
            cps=cps,
            config=config,
            point=to_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
        # Ensure 7th-axis move is complete before entering contact from prepoint.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
    # def run_single_movement(robot_point, seventh_axis_point, cps, config):
    #     """
    #     Moves the robot and seventh axis simultaneously (parallel motion).
    #     Waits until both movements are completed before returning.
    #     """
        
    #     # === Step 1: Start both movements at the same time ===
        
    #     communicate(
    #         cps=cps,
    #         config=config,
    #         point=robot_point,
    #         tcp=config['coords']['tcptool4plane2'],
    #         ucs=config['coords']['ucsTable2'],
    #         seventh=-1,
    #         speed=speed_profile['travel'],
                # velocity_profile="robot",
    #         wait=True
    #     )
    #     communicate(
    #         cps=cps,
    #         config=config,
    #         seventh=seventh_axis_point,
    #         tcp=config['coords']['tcptool4plane2'],
    #         ucs=config['coords']['ucsTable2'],
    #         speed=speed_profile['travel'],
                # velocity_profile="robot",
    #         wait=True
    #     )


    #     # === Step 2: Wait for both to finish ===
    #     robot_complete = False
    #     seventh_complete = False

    #     while not (robot_complete and seventh_complete):
    #         # Check robot state
    #         robotRes = []
    #         nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
    #         if len(robotRes) > 11 and robotRes[11] == '1':  # 1 = idle
    #             robot_complete = True

    #         # Check seventh-axis state - FIXED: Check if NOT RUNNING
    #         # Check seventh-axis state
    #         pluginRes = []
    #         nret = cps.HRIF_HRApp(0, 'HR_Motor', 'MotorGetState', ["J7"], pluginRes)
    #         if len(pluginRes) > 2 and pluginRes[2] == '0':  # 0 = motion complete
    #             seventh_complete = True

    #         time.sleep(0.01)

    #     return True
   
    # Ensure we always start from a released force/vibration state.
    releaseForce(cps=cps, config=config, wait_for_blending=False)
    turn_vibration_off(cps)

    # Pass 1: top -> right -> bottom (x1)
    move_axis_then_robot(to_point=pretpointmiddle, seventh_axis_point=x1, cps=cps, config=config)
    communicate(cps=cps,config=config,point=top_mid_to_right[0],tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['approach'],velocity_profile="robot",wait=True)
    # Vibration is enabled on first real contact inside start_force_if_needed().
    # turn_tool_spin_on(cps)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force)
    perform_process_right(cps, config, points1=rightpoints,force=force)
    perform_process_bottom(cps, config, points1=bottom_right_to_mid,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False

    # Pass 2: bottom -> left -> top (x2, centered)
    move_axis_then_robot(to_point=prebpointmiddle_shifted, seventh_axis_point=x2, cps=cps, config=config)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left_shifted,force=force)
    perform_process_left(cps, config, points1=leftpoints_shifted,force=force)
    perform_process_top(cps, config, points1=top_left_to_mid_shifted,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=pretpointmiddle_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    communicate(cps=cps,config=config,point=htoppoints_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    # turn_tool_spin_off(cps)

    
def model1pocket3side(force,cps):
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


   
   
    z=-4
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z=-4

    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    sanding_speed = float(json_config['sandingSpeed'])
    speed_profile = {
        'travel': speeed,
        'approach': speeed,
        'return': speeed,
        'tool_change': speeed,
        'contact': sanding_speed,
    }
    if speed_profile['contact'] <= 0:
        speed_profile['contact'] = speed_profile['travel']
        print("[model1pocket3side] sandingSpeed<=0, using travel speed for contact moves")
    safe_approach_speed = speed_profile['approach'] if speed_profile['approach'] > 0 else 0.2
    safe_contact_speed = speed_profile['contact'] if speed_profile['contact'] > 0 else safe_approach_speed

   
     
    # Calculate new points outside the pocket
    p5 = [p5[0] + outeroffset, p5[1] - outeroffset, z, 180, 0, 0]
    p6 = [p6[0] + outeroffset, p6[1] + outeroffset, z, 180, 0, 0]
    p7 = [p7[0] - outeroffset, p7[1] + outeroffset, z, 180, 0, 0]
    p8 = [p8[0] - outeroffset, p8[1] - outeroffset, z, 180, 0, 0]


    print("offset p5=", p5)
    print("offset p6=", p6)
    print("offset p7=", p7)
    print("offset p8=", p8)


    #7th axis movement
    x1=p8[0]
    print("x1:", x1)
    seventhdistance=(p5[0] - p8[0])/2
    print("seventhdistance:", seventhdistance)
    x2=x1+seventhdistance
    print("x2:", x2)
    x3=x1+seventhdistance*2
    print("x3:", x3)




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -25, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance, point5[1], -25, 180, 0, 0]
    bpoint2nd=[distance, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    mid_x=(bpoint1st[0] + bpoint2nd[0]) / 2
    bpointmiddle=[mid_x, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)
    prebpointmiddle=[bpointmiddle[0], bpointmiddle[1], -25, 180, 0, 0]
    print("prebpointmiddle:", prebpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[bpoint1st, bpointmiddle, bpoint2nd]
    print("bpoints:", bpoints)
    bottom_right_to_mid=[bpoint1st, bpointmiddle]
    bottom_mid_to_left=[bpointmiddle, bpoint2nd]

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [distance, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [distance, point5[1], -25, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [distance, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [distance, point6[1], -25, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [distance, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[lpoint1st, lpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [distance, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -25, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [mid_x, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    pretpointmiddle = [topointmiddle[0], topointmiddle[1], -25, 180, 0, 0]
    print("pretpointmiddle:", pretpointmiddle)
    tpoint2nd = [distance, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [distance, point7[1], -25, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -50, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[tpoint1st, topointmiddle, tpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_right=[topointmiddle, tpoint1st]
    top_left_to_mid=[tpoint2nd, topointmiddle]

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -25, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -25, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[rpoint1st, rpoint2nd]
    print("rightpoints:",rightpoints)

    # Shifted points for pass 2 (centered on mid_x)
    shift_x = mid_x
    def shift_point(pt):
        return [pt[0] - shift_x, pt[1], pt[2], pt[3], pt[4], pt[5]]

    prebpointmiddle_shifted = shift_point(prebpointmiddle)
    bottom_mid_to_left_shifted = [shift_point(p) for p in bottom_mid_to_left]
    leftpoints_shifted = [shift_point(p) for p in leftpoints]
    top_left_to_mid_shifted = [shift_point(p) for p in top_left_to_mid]
    pretpointmiddle_shifted = shift_point(pretpointmiddle)
    htoppoints_shifted = shift_point(htoppoints)

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -25, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -25, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)





    force_active = False

    def start_force_if_needed():
        nonlocal force_active
        if force_active:
            return
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        force_active = True

    force_approach_lift = 20.0

    def move_to_contact_point(first_point):
        # Table B Z+ searches downward, so prepoint lift is negative Z.
        force_prepoint = list(first_point)
        force_prepoint[2] = first_point[2] - force_approach_lift
        communicate(
            cps=cps,
            config=config,
            point=force_prepoint,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=safe_approach_speed,
            velocity_profile="robot",
            wait=True
        )
        return True
    def perform_process_bottom(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_left(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_top(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_right(cps, config, points1,force):
        if not points1:
            return
        first_point = points1[0]
        if not move_to_contact_point(first_point):
            return
        # Force is enabled from the lifted prepoint; the first path point is reached under force.
        start_force_if_needed()
        for idx, point in enumerate(points1):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=safe_contact_speed,
                velocity_profile="sanding",
                wait=(idx == len(points1) - 1)
            )

        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def perform_process_middle(cps, config, points1,force):
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
        for idx, point in enumerate(points1):
            if point == pmiddile21:
                if not move_to_contact_point(point):
                    return
                putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )

            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcptool4plane2'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speed_profile['contact'],
                velocity_profile="sanding",
                wait=(idx == 0 or idx == len(points1) - 1)
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        # Release Force Control
    def move_axis_then_robot(to_point, seventh_axis_point, cps, config):
        """
        Move 7th axis and robot prepoint transition simultaneously from current pose.
        Then hold at prepoint until 7th-axis target is fully reached.
        """
        # Start 7th-axis move without blocking so arm move can run simultaneously.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=False
        )
        # Move to next prepoint while 7th axis is traveling.
        communicate(
            cps=cps,
            config=config,
            point=to_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
        # Ensure 7th-axis move is complete before entering contact from prepoint.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_axis_point,
            tcp=config['coords']['tcptool4plane2'],
            ucs=config['coords']['ucsTable2'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True
        )
    # def run_single_movement(robot_point, seventh_axis_point, cps, config):
    #     """
    #     Moves the robot and seventh axis simultaneously (parallel motion).
    #     Waits until both movements are completed before returning.
    #     """
        
    #     # === Step 1: Start both movements at the same time ===
    #     communicate(
    #         cps=cps,
    #         config=config,
    #         point=robot_point,
    #         tcp=config['coords']['tcptool4plane2'],
    #         ucs=config['coords']['ucsTable2'],
    #         seventh=-1,
    #         speed=speed_profile['travel'],
                # velocity_profile="robot",
    #         wait=True
    #     )

    #     communicate(
    #         cps=cps,
    #         config=config,
    #         seventh=seventh_axis_point,
    #         tcp=config['coords']['tcptool4plane2'],
    #         ucs=config['coords']['ucsTable2'],
    #         speed=speed_profile['travel'],
                # velocity_profile="robot",
    #         wait=True
    #     )

        
    #     # === Step 2: Wait for both to finish ===
    #     robot_complete = False
    #     seventh_complete = False

    #     while not (robot_complete and seventh_complete):
    #         # Check robot state
    #         robotRes = []
    #         nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
    #         if len(robotRes) > 11 and robotRes[11] == '1':  # 1 = idle
    #             robot_complete = True

    #         # Check seventh-axis state
    #         pluginRes = []
    #         nret = cps.HRIF_HRApp(0, 'HR_Motor', 'MotorGetState', ["J7"], pluginRes)
    #         if len(pluginRes) > 2 and pluginRes[2] == '0':  # 0 = motion complete
    #             seventh_complete = True

    #         time.sleep(0.01)

    #     return True
    # Ensure we always start from a released force/vibration state.
    releaseForce(cps=cps, config=config, wait_for_blending=False)
    turn_vibration_off(cps)

    # Pass 1: top -> right -> bottom (x1)
    move_axis_then_robot(to_point=pretpointmiddle, seventh_axis_point=x1, cps=cps, config=config)
    communicate(cps=cps,config=config,point=top_mid_to_right[0],tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['approach'],velocity_profile="robot",wait=True)
    # Vibration is enabled on first real contact inside start_force_if_needed().
    # turn_tool_spin_on(cps)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force)
    perform_process_right(cps, config, points1=rightpoints,force=force)
    perform_process_bottom(cps, config, points1=bottom_right_to_mid,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False

    # Pass 2: bottom -> left -> top (x2, centered)
    move_axis_then_robot(to_point=prebpointmiddle_shifted, seventh_axis_point=x2, cps=cps, config=config)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left_shifted,force=force)
    perform_process_left(cps, config, points1=leftpoints_shifted,force=force)
    perform_process_top(cps, config, points1=top_left_to_mid_shifted,force=force)
    releaseForce(cps=cps, config=config)
    force_active = False
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=pretpointmiddle_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    communicate(cps=cps,config=config,point=htoppoints_shifted,tcp=config['coords']['tcptool4plane2'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speed_profile['travel'],velocity_profile="robot",wait=True)
    # turn_tool_spin_off(cps)

    
def mod1tool1siderun(force,cps):
    config = load_config()




    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])




    #Establish connection with robot
    # cps = CPSClient()
    # IP = config['server']['cpip']
    # port = config['server']['cps']
    # ret = cps.HRIF_Connect(0, IP, port)

    model1pocket1side(force,cps)
    time.sleep(0.5)
    model1pocket2side(force,cps)
    time.sleep(0.5)
    model1pocket3side(force,cps)
    time.sleep(0.5)
    # Keep current 7th-axis position from pocket-side; bottom routine will handle its own indexed passes.

    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        # if xlen > 1000:
        #     model1bottombig(force,cps)
        # else:
        model1bottomsmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(xlen)} - expected number or 'null'")

if __name__ == "__main__":
    # Call the function with an appropriate force value
    # Replace 10 with your desired force value
    # model1pocket1side(5)
    # model1pocket2side(5)
    # model1pocket3side(5)
    mod1tool1siderun(5)
