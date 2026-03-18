# testcommu.py




import sys
import os
import time
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))




import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from Table1Model.exportpointsmodule import exported_points
from model1cycle.mod1bottomb import model1bottombig
from model1cycle.mod1bottoms import model1bottomsmall

def _frame_point(pt, z):
    return [pt[0], pt[1], z, 180, 0, 0]

def _midpoint(a, b, z):
    return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, z, 180, 0, 0]

def _build_frame_paths(p13, p14, p15, p16, z_work, z_clear):
    # Corner mapping: bottom-left p13, top-left p14, top-right p15, bottom-right p16
    blw = _frame_point(p13, z_work)
    tlw = _frame_point(p14, z_work)
    trw = _frame_point(p15, z_work)
    brw = _frame_point(p16, z_work)
    blc = _frame_point(p13, z_clear)
    tlc = _frame_point(p14, z_clear)
    trc = _frame_point(p15, z_clear)
    brc = _frame_point(p16, z_clear)

    bottom_mid_w = _midpoint(blw, brw, z_work)
    top_mid_w = _midpoint(tlw, trw, z_work)
    left_mid_w = _midpoint(blw, tlw, z_work)
    right_mid_w = _midpoint(brw, trw, z_work)
    bottom_mid_c = _midpoint(blw, brw, z_clear)
    top_mid_c = _midpoint(tlw, trw, z_clear)

    return {
        "blw": blw,
        "tlw": tlw,
        "trw": trw,
        "brw": brw,
        "blc": blc,
        "tlc": tlc,
        "trc": trc,
        "brc": brc,
        "bottom_mid_w": bottom_mid_w,
        "top_mid_w": top_mid_w,
        "left_mid_w": left_mid_w,
        "right_mid_w": right_mid_w,
        "bottom_mid_c": bottom_mid_c,
        "top_mid_c": top_mid_c,
        "bottom_mid_to_left": [bottom_mid_c, bottom_mid_w, blw, blc],
        "bottom_mid_to_right": [bottom_mid_c, bottom_mid_w, brw, brc],
        "top_mid_to_left": [top_mid_c, top_mid_w, tlw, tlc],
        "top_mid_to_right": [top_mid_c, top_mid_w, trw, trc],
        "leftpoints": [blc, blw, left_mid_w, tlw, tlc],
        "rightpoints": [trc, trw, right_mid_w, brw, brc],
    }








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


   
   
    z_clear = -10
    z_work = -4
    z = z_work
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z = z_work
    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])

   
     
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




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -10, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance/2, point5[1], -10, 180, 0, 0]
    bpoint2nd=[distance/2, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    bpointmiddle=[0+2, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[prebpoint1st,bpoint1st, bpointmiddle, bpoint2nd,prebpoint2nd]
    print("bpoints:", bpoints)
    bottom_mid_to_left=[bpointmiddle,bpoint1st,prebpoint1st]
    bottom_mid_to_right=[bpointmiddle,bpoint2nd,prebpoint2nd]
    print("bottom_mid_to_left:", bottom_mid_to_left)
    print("bottom_mid_to_right:", bottom_mid_to_right)

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [0, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [0, point5[1], -10, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [0, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [0, point6[1], -10, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [0, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[prelpoint1st,lpoint1st,lpointmiddle,lpoint2nd,prelpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [0, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -10, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [0+2, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    tpoint2nd = [-distance/2, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [-distance/2, point7[1], -10, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -20, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[pretpoint1st,tpoint1st,topointmiddle,tpoint2nd,pretpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_left=[topointmiddle,tpoint1st,pretpoint1st]
    top_mid_to_right=[topointmiddle,tpoint2nd,pretpoint2nd]
    print("top_mid_to_left:", top_mid_to_left)
    print("top_mid_to_right:", top_mid_to_right)

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -10, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -10, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[prepoint1st,rpoint1st,rpointmiddle,rpoint2nd,prepoint2nd]
    print("rightpoints:",rightpoints)

    # Override frame paths using offset corners (p5..p8) and fixed Z values
    frame_paths = _build_frame_paths(p5, p6, p7, p8, z_work, z_clear)

    x1 = frame_paths["brw"][0]
    x2 = x1 + (frame_paths["blw"][0] - frame_paths["brw"][0]) / 2

    bpointmiddle = frame_paths["bottom_mid_w"]
    bpoint1st = frame_paths["blw"]
    bpoint2nd = frame_paths["brw"]
    prebpoint1st = frame_paths["blc"]
    prebpoint2nd = frame_paths["brc"]

    lpoint1st = frame_paths["blw"]
    lpoint2nd = frame_paths["tlw"]
    lpointmiddle = frame_paths["left_mid_w"]
    prelpoint1st = frame_paths["blc"]
    prelpoint2nd = frame_paths["tlc"]

    tpoint1st = frame_paths["tlw"]
    tpoint2nd = frame_paths["trw"]
    topointmiddle = frame_paths["top_mid_w"]
    pretpoint1st = frame_paths["tlc"]
    pretpoint2nd = frame_paths["trc"]

    rpoint1st = frame_paths["trw"]
    rpoint2nd = frame_paths["brw"]
    rpointmiddle = frame_paths["right_mid_w"]
    prepoint1st = frame_paths["trc"]
    prepoint2nd = frame_paths["brc"]

    bottom_mid_to_left = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["blw"]]
    bottom_mid_to_right = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["brw"]]
    top_mid_to_left = frame_paths["top_mid_to_left"]
    top_mid_to_right = frame_paths["top_mid_to_right"]
    leftpoints = [prelpoint1st, lpoint1st, lpoint2nd, prelpoint2nd]
    rightpoints = [prepoint1st, rpoint1st, rpoint2nd, prepoint2nd]

    bpoints = [prebpoint1st, bpoint1st, bpointmiddle, bpoint2nd]

    toppoints = [pretpoint1st, tpoint1st, topointmiddle, tpoint2nd, pretpoint2nd]

    hbpoint1st = [frame_paths["bottom_mid_w"][0], frame_paths["bottom_mid_w"][1], -50, 180, 0, 0]

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -10, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -10, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)






    

    def perform_process_bottom(cps, config, points1,force, finalize=True):
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
            if point==bpointmiddle:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
            if point==bpoint2nd:
                turn_vibration_on(cps)
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_left(cps, config, points1,force, finalize=True):
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
            if point==lpoint1st:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
            if point==lpoint2nd:
                turn_vibration_on(cps)
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_top(cps, config, points1,force, finalize=True):
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
            if point==topointmiddle:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
            if point==tpoint2nd:
                turn_vibration_on(cps)
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_right(cps, config, points1,force, finalize=True):
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
            if point==rpoint1st:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
            if point==rpoint2nd:
                turn_vibration_on(cps)
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_middle(cps, config, points1,force):
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
            if point==pmiddile21:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
            if point==pmiddile1:
                turn_vibration_on(cps)
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
        import time

    

    def run_single_movement(robot_point, seventh_axis_point, cps, config):
        lock = threading.Lock()
        # time.sleep(0.2)
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
   
    # Pass 1 (x1): top mid -> right side -> bottom mid
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force, finalize=False)
    perform_process_right(cps, config, points1=rightpoints,force=force, finalize=False)
    perform_process_bottom(cps, config, points1=bottom_mid_to_right,force=force, finalize=True)

    # Pass 2 (x2): bottom mid -> left side -> top mid
    # Move to safe height, shift 7th axis, then re-approach at the new axis position
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left,force=force, finalize=False)
    perform_process_left(cps, config, points1=leftpoints,force=force, finalize=False)
    perform_process_top(cps, config, points1=top_mid_to_left,force=force, finalize=True)
    #Last movement
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    
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


   
   
    z_clear = -10
    z_work = -4
    z = z_work
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z = z_work

    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])

   
     
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




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -10, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance/2, point5[1], -10, 180, 0, 0]
    bpoint2nd=[distance/2, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    bpointmiddle=[0+2, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[prebpoint1st,bpoint1st, bpointmiddle, bpoint2nd,prebpoint2nd]
    print("bpoints:", bpoints)
    bottom_mid_to_left=[bpointmiddle,bpoint1st,prebpoint1st]
    bottom_mid_to_right=[bpointmiddle,bpoint2nd,prebpoint2nd]
    print("bottom_mid_to_left:", bottom_mid_to_left)
    print("bottom_mid_to_right:", bottom_mid_to_right)

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [0, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [0, point5[1], -10, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [0, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [0, point6[1], -10, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [0, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[prelpoint1st,lpoint1st,lpointmiddle,lpoint2nd,prelpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [0, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -10, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [0+2, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    tpoint2nd = [-distance/2, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [-distance/2, point7[1], -10, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -20, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[pretpoint1st,tpoint1st,topointmiddle,tpoint2nd,pretpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_left=[topointmiddle,tpoint1st,pretpoint1st]
    top_mid_to_right=[topointmiddle,tpoint2nd,pretpoint2nd]
    print("top_mid_to_left:", top_mid_to_left)
    print("top_mid_to_right:", top_mid_to_right)

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -10, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -10, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[prepoint1st,rpoint1st,rpointmiddle,rpoint2nd,prepoint2nd]
    print("rightpoints:",rightpoints)

    # Override frame paths using offset corners (p5..p8) and fixed Z values
    frame_paths = _build_frame_paths(p5, p6, p7, p8, z_work, z_clear)

    x1 = frame_paths["brw"][0]
    x2 = x1 + (frame_paths["blw"][0] - frame_paths["brw"][0]) / 2

    bpointmiddle = frame_paths["bottom_mid_w"]
    bpoint1st = frame_paths["blw"]
    bpoint2nd = frame_paths["brw"]
    prebpoint1st = frame_paths["blc"]
    prebpoint2nd = frame_paths["brc"]

    lpoint1st = frame_paths["blw"]
    lpoint2nd = frame_paths["tlw"]
    lpointmiddle = frame_paths["left_mid_w"]
    prelpoint1st = frame_paths["blc"]
    prelpoint2nd = frame_paths["tlc"]

    tpoint1st = frame_paths["tlw"]
    tpoint2nd = frame_paths["trw"]
    topointmiddle = frame_paths["top_mid_w"]
    pretpoint1st = frame_paths["tlc"]
    pretpoint2nd = frame_paths["trc"]

    rpoint1st = frame_paths["trw"]
    rpoint2nd = frame_paths["brw"]
    rpointmiddle = frame_paths["right_mid_w"]
    prepoint1st = frame_paths["trc"]
    prepoint2nd = frame_paths["brc"]

    bottom_mid_to_left = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["blw"]]
    bottom_mid_to_right = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["brw"]]
    top_mid_to_left = frame_paths["top_mid_to_left"]
    top_mid_to_right = frame_paths["top_mid_to_right"]
    leftpoints = [prelpoint1st, lpoint1st, lpoint2nd, prelpoint2nd]
    rightpoints = [prepoint1st, rpoint1st, rpoint2nd, prepoint2nd]

    bpoints = [prebpoint1st, bpoint1st, bpointmiddle, bpoint2nd]

    toppoints = [pretpoint1st, tpoint1st, topointmiddle, tpoint2nd, pretpoint2nd]

    hbpoint1st = [frame_paths["bottom_mid_w"][0], frame_paths["bottom_mid_w"][1], -50, 180, 0, 0]

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -10, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -10, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)






    

    def perform_process_bottom(cps, config, points1,force, finalize=True):
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
            if point==bpointmiddle:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==bpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_left(cps, config, points1,force, finalize=True):
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
            if point==lpoint1st:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==lpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_top(cps, config, points1,force, finalize=True):
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
            if point==topointmiddle:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==tpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_right(cps, config, points1,force, finalize=True):
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
            if point==rpoint1st:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==rpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_middle(cps, config, points1,force):
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
            if point==pmiddile21:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==pmiddile1:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def run_single_movement(robot_point, seventh_axis_point, cps, config):
        lock = threading.Lock()
        # time.sleep(0.2)
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
    #         tcp=config['coords']['tcpReal'],
    #         ucs=config['coords']['ucsTable2'],
    #         seventh=-1,
    #         speed=0.8,
    #         wait=False
    #     )
    #     communicate(
    #         cps=cps,
    #         config=config,
    #         seventh=seventh_axis_point,
    #         tcp=config['coords']['tcpReal'],
    #         ucs=config['coords']['ucsTable2'],
    #         speed=0.5,
    #         wait=False
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
   
    # Pass 1 (x1): top mid -> right side -> bottom mid
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force, finalize=False)
    perform_process_right(cps, config, points1=rightpoints,force=force, finalize=False)
    perform_process_bottom(cps, config, points1=bottom_mid_to_right,force=force, finalize=True)

    # Pass 2 (x2): bottom mid -> left side -> top mid
    # Move to safe height, shift 7th axis, then re-approach at the new axis position
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left,force=force, finalize=False)
    perform_process_left(cps, config, points1=leftpoints,force=force, finalize=False)
    perform_process_top(cps, config, points1=top_mid_to_left,force=force, finalize=True)
    #Last movement
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    
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


   
   
    z_clear = -10
    z_work = -4
    z = z_work
    # #Outer Pocket Offset
    # outeroffset=p7[0]/2
    # print("outeroffset=",outeroffset)
    # outeroffset1=p7[0]
    # print("outeroffset1=",outeroffset1)
    framesize=p16[0]
    print("framesize=",framesize)
    outeroffset=framesize/2
    print("outeroffset=",outeroffset)
    z = z_work

    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])

   
     
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




    #Bottom Points
    point8 = [p8[0], p8[1], z, 180, 0, 0]
    print("point8:", point8)
    point5 = [p5[0], p5[1], z, 180, 0, 0]
    print("point5:", point5)
    distance= p5[0] - p8[0]
    print("distance:", distance)
    #Bottom Final Points
    prebpoint1st=[0, point8[1], -10, 180, 0, 0]
    bpoint1st=[0, point8[1], z, 180, 0, 0]
    print("bpoint1st:", bpoint1st)
    prebpoint2nd=[distance/2, point5[1], -10, 180, 0, 0]
    bpoint2nd=[distance/2, point5[1], z, 180, 0, 0]
    print("bpoint2nd:", bpoint2nd)
    bpointmiddle=[0+2, point8[1], z, 180, 0, 0]
    print("bpointmiddle:", bpointmiddle)


    #Hpoints for bottom
    hbpoint1st=[0, point8[1], -50, 180, 0, 0]
    print("hbpoint1st:", hbpoint1st)
    bpoints=[prebpoint1st,bpoint1st, bpointmiddle, bpoint2nd,prebpoint2nd]
    print("bpoints:", bpoints)
    bottom_mid_to_left=[bpointmiddle,bpoint1st,prebpoint1st]
    bottom_mid_to_right=[bpointmiddle,bpoint2nd,prebpoint2nd]
    print("bottom_mid_to_left:", bottom_mid_to_left)
    print("bottom_mid_to_right:", bottom_mid_to_right)

    #Points Left
    point6 = [p6[0], p6[1], z, 180, 0, 0]
    print("point6:", point6)

    #Left Final Points
    lpoint1st = [0, point5[1], z, 180, 0, 0]
    print("lpoint1st:", lpoint1st)
    prelpoint1st = [0, point5[1], -10, 180, 0, 0]
    print("prelpoint1st:", prelpoint1st)
    lpoint2nd = [0, point6[1], z, 180, 0, 0]
    print("lpoint2nd:", lpoint2nd)
    prelpoint2nd = [0, point6[1], -10, 180, 0, 0]
    print("prelpoint2nd:", prelpoint2nd)
    lpointmiddle = [0, point5[1] + 2, z, 180, 0, 0]
    print("lpointmiddle:", lpointmiddle)

    leftpoints=[prelpoint1st,lpoint1st,lpointmiddle,lpoint2nd,prelpoint2nd]
    print("leftpoints:",leftpoints)


    #Hpoints for left
    hleft2nd = [0, point6[1], -20, 180, 0, 0]
    print("hleft2nd:", hleft2nd)

    #Top Points
    point7= [p7[0], p7[1], z, 180, 0, 0]
    print("point7:", point7)

    #Top Final Pooints
    tpoint1st = [0, point6[1], z, 180, 0, 0]
    print("tpoint1st:", tpoint1st)
    pretpoint1st = [0, point6[1], -10, 180, 0, 0]
    print("pretpoint1st:", pretpoint1st)
    topointmiddle = [0+2, point6[1], z, 180, 0, 0]
    print("topointmiddle:", topointmiddle)
    tpoint2nd = [-distance/2, point7[1], z, 180, 0, 0]
    print("tpoint2nd:", tpoint2nd)
    pretpoint2nd = [-distance/2, point7[1], -10, 180, 0, 0]
    print("pretpoint2nd:", pretpoint2nd)


    #Hpoints for top
    htoppoints=[0, point7[1], -20, 180, 0, 0]
    print("htoppoints:", htoppoints)


    toppoints=[pretpoint1st,tpoint1st,topointmiddle,tpoint2nd,pretpoint2nd]
    print("toppoints:", toppoints)
    top_mid_to_left=[topointmiddle,tpoint1st,pretpoint1st]
    top_mid_to_right=[topointmiddle,tpoint2nd,pretpoint2nd]
    print("top_mid_to_left:", top_mid_to_left)
    print("top_mid_to_right:", top_mid_to_right)

    #Right Side
    rpoint1st = [0, point7[1], z, 180, 0, 0]
    print("rpoint1st:", rpoint1st)
    rpoint2nd = [0, point8[1], z, 180, 0, 0]
    print("rpoint2nd:", rpoint2nd)
    prepoint1st = [0, point7[1], -10, 180, 0, 0]
    print("prepoint1st:", prepoint1st)
    prepoint2nd = [0, point8[1], -10, 180, 0, 0]
    print("prepoint2nd:", prepoint2nd)
    rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
    print("rpointmiddle:", rpointmiddle)

    rightpoints=[prepoint1st,rpoint1st,rpointmiddle,rpoint2nd,prepoint2nd]
    print("rightpoints:",rightpoints)

    # Override frame paths using offset corners (p5..p8) and fixed Z values
    frame_paths = _build_frame_paths(p5, p6, p7, p8, z_work, z_clear)

    x1 = frame_paths["brw"][0]
    x2 = x1 + (frame_paths["blw"][0] - frame_paths["brw"][0]) / 2

    bpointmiddle = frame_paths["bottom_mid_w"]
    bpoint1st = frame_paths["blw"]
    bpoint2nd = frame_paths["brw"]
    prebpoint1st = frame_paths["blc"]
    prebpoint2nd = frame_paths["brc"]

    lpoint1st = frame_paths["blw"]
    lpoint2nd = frame_paths["tlw"]
    lpointmiddle = frame_paths["left_mid_w"]
    prelpoint1st = frame_paths["blc"]
    prelpoint2nd = frame_paths["tlc"]

    tpoint1st = frame_paths["tlw"]
    tpoint2nd = frame_paths["trw"]
    topointmiddle = frame_paths["top_mid_w"]
    pretpoint1st = frame_paths["tlc"]
    pretpoint2nd = frame_paths["trc"]

    rpoint1st = frame_paths["trw"]
    rpoint2nd = frame_paths["brw"]
    rpointmiddle = frame_paths["right_mid_w"]
    prepoint1st = frame_paths["trc"]
    prepoint2nd = frame_paths["brc"]

    bottom_mid_to_left = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["blw"]]
    bottom_mid_to_right = [frame_paths["bottom_mid_c"], frame_paths["bottom_mid_w"], frame_paths["brw"]]
    top_mid_to_left = frame_paths["top_mid_to_left"]
    top_mid_to_right = frame_paths["top_mid_to_right"]
    leftpoints = [prelpoint1st, lpoint1st, lpoint2nd, prelpoint2nd]
    rightpoints = [prepoint1st, rpoint1st, rpoint2nd, prepoint2nd]

    bpoints = [prebpoint1st, bpoint1st, bpointmiddle, bpoint2nd]

    toppoints = [pretpoint1st, tpoint1st, topointmiddle, tpoint2nd, pretpoint2nd]

    hbpoint1st = [frame_paths["bottom_mid_w"][0], frame_paths["bottom_mid_w"][1], -50, 180, 0, 0]

    #Middle Points
    pmiddile1=[0+ outeroffset, p10[1] - outeroffset, z, 180, 0, 0]
    print("pmiddile1=", pmiddile1)
    pmiddile2=[0 + outeroffset, p9[1] + outeroffset, z, 180, 0, 0]
    print("pmiddile2=", pmiddile2)
    pmiddile21=[0 + outeroffset, p9[1] + outeroffset+2, z, 180, 0, 0]
    print("pmiddile21=", pmiddile21)
   




    #processing Middle Points
    premiddle1=[0 + outeroffset, p10[1] - outeroffset, -10, 180, 0, 0]
    print("premiddle1=", premiddle1)
    premiddle2=[0 + outeroffset, p9[1] + outeroffset, -10, 180, 0, 0]
    print("premiddle2=", premiddle2)


    middlepoints=[premiddle2,pmiddile2,pmiddile21,pmiddile1,premiddle1]
    print("middlepoints=", middlepoints)

    #Middle 7th axis
    mx=p10[0]
    print("mx:",mx)






    

    def perform_process_bottom(cps, config, points1,force, finalize=True):
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
            if point==bpointmiddle:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==bpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_left(cps, config, points1,force, finalize=True):
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
            if point==lpoint1st:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==lpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_top(cps, config, points1,force, finalize=True):
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
            if point==topointmiddle:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==tpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_right(cps, config, points1,force, finalize=True):
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
            if point==rpoint1st:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==rpoint2nd:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def perform_process_middle(cps, config, points1,force):
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
            if point==pmiddile21:putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            if point==pmiddile1:turn_vibration_on(cps)
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
        
        if finalize:
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config) 
    def run_single_movement(robot_point, seventh_axis_point, cps, config):
        lock = threading.Lock()
        # time.sleep(0.2)
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
    #         tcp=config['coords']['tcpReal'],
    #         ucs=config['coords']['ucsTable2'],
    #         seventh=-1,
    #         speed=0.8,
    #         wait=False
    #     )

    #     communicate(
    #         cps=cps,
    #         config=config,
    #         seventh=seventh_axis_point,
    #         tcp=config['coords']['tcpReal'],
    #         ucs=config['coords']['ucsTable2'],
    #         speed=0.5,
    #         wait=False
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
    # Pass 1 (x1): top mid -> right side -> bottom mid
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    perform_process_top(cps, config, points1=top_mid_to_right,force=force, finalize=False)
    perform_process_right(cps, config, points1=rightpoints,force=force, finalize=False)
    perform_process_bottom(cps, config, points1=bottom_mid_to_right,force=force, finalize=True)

    # Pass 2 (x2): bottom mid -> left side -> top mid
    # Move to safe height, shift 7th axis, then re-approach at the new axis position
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    perform_process_bottom(cps, config, points1=bottom_mid_to_left,force=force, finalize=False)
    perform_process_left(cps, config, points1=leftpoints,force=force, finalize=False)
    perform_process_top(cps, config, points1=top_mid_to_left,force=force, finalize=True)
    #Last movement
    communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    
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
    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable2'],speed=0.7,wait=True)

    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 1000:
            model1bottombig(force,cps)
        else:
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








