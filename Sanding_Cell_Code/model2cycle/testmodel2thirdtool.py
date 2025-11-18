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
from Table2Model.exportpointsmodule import exported_points
from model2cycle.testmodelthirdtoolb import model2thirdbig








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





def model2thirdtoolrun(force,cps):
    def model2thirdtoolsmall(force,cps):
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
    
        p51 = exported_points["p5"]
        p61 = exported_points["p6"]
        p71 = exported_points["p7"]
        p81 = exported_points["p8"]
        # p9 = exported_points["p9"]
        # p10 = exported_points["p10"]
        # p11 = exported_points["p11"]
        # p12 = exported_points["p12"]




    
        print("p51=",p51)
        print("p61=",p61)
        print("p71=",p71)
        print("p81=",p81)
        # print("p9=",p9)
        # print("p10=",p10)
        # print("p11=",p11)
        # print("p12=",p12)
        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        print(speeed)
        z=-40
        #Outer Pocket Offset
        outeroffset=23
        print("outeroffset=",outeroffset)
        #Outeroffsetorg
        outeroffset1=p71[0]/2
        print("outeroffset=",outeroffset1)
    
        
        # Calculate new points outside the pocket
        # Assuming p1: bottom-left, p2: bottom-right, p3: top-right, p4: top-left
        p5 = [p51[0] - outeroffset, p51[1] + outeroffset, z, 180, 0, 0]
        p6 = [p61[0] - outeroffset, p61[1] - outeroffset, z, 180, 0, 0]
        p7 = [p71[0] + outeroffset, p71[1] - outeroffset, z, 180, 0, 0]
        p8 = [p81[0] + outeroffset, p81[1] + outeroffset, z, 180, 0, 0]


        print("offset p5=", p5)
        print("offset p6=", p6)
        print("offset p7=", p7)
        print("offset p8=", p8)

        #7th axis movement
        x1=p8[0]
        print("x1:", x1)
        seventhdistance=(p51[0] - p81[0])/2
        print("seventhdistance:", seventhdistance)
        x2=outeroffset+outeroffset1+seventhdistance
        print("x2:", x2)
        x3=outeroffset+outeroffset1+(seventhdistance*2)
        print("x3:", x3)

        #Points for Bottom movement


        #Bottom Points
        point8 = [p8[0], p8[1], z, 180, 0, 0]
        print("point8:", point8)
        point5 = [p5[0], p5[1], z, 180, 0, 0]
        print("point5:", point5)
        distance= p51[0] - p81[0]
        print("distance:", distance)
        #Bottom Final Points
        prebpoint1st=[0, point8[1], -55, 180, 0, 0]
        bpoint1st=[0, point8[1], z, 180, 0, 0]
        print("bpoint1st:", bpoint1st)
        prebpoint2nd=[distance/2, point5[1], -55, 180, 0, 0]
        bpoint2nd=[distance/2, point5[1], z, 180, 0, 0]
        print("bpoint2nd:", bpoint2nd)
        bpointmiddle=[0+2, point8[1], z, 180, 0, 0]
        print("bpointmiddle:", bpointmiddle)


        #Hpoints for bottom
        hbpoint1st=[0, point8[1], -70, 180, 0, 0]
        print("hbpoint1st:", hbpoint1st)
        bpoints=[prebpoint1st,bpoint1st, bpointmiddle, bpoint2nd,prebpoint2nd]
        print("bpoints:", bpoints)

        #Points Left
        point6 = [p6[0], p6[1], z, 180, 0, 0]
        print("point6:", point6)

        #Left Final Points
        lpoint1st = [0, point5[1], z, 180, 0, 0]
        print("lpoint1st:", lpoint1st)
        prelpoint1st = [0, point5[1], -55, 180, 0, 0]
        print("prelpoint1st:", prelpoint1st)
        lpoint2nd = [0, point6[1], z, 180, 0, 0]
        print("lpoint2nd:", lpoint2nd)
        prelpoint2nd = [0, point6[1], -55, 180, 0, 0]
        print("prelpoint2nd:", prelpoint2nd)
        lpointmiddle = [0, point5[1] + 2, z, 180, 0, 0]
        print("lpointmiddle:", lpointmiddle)

        leftpoints=[prelpoint1st,lpoint1st,lpointmiddle,lpoint2nd,prelpoint2nd]
        print("leftpoints:",leftpoints)


        #Hpoints for left
        hleft2nd = [0, point6[1], -70, 180, 0, 0]
        print("hleft2nd:", hleft2nd)

        #Top Points
        distancetop= p5[0] - p8[0]
        point7= [p7[0], p7[1], z, 180, 0, 0]
        print("point7:", point7)

        #Top Final Pooints
        tpoint1st = [0, point6[1]+3, z, 180, 0, 0]
        print("tpoint1st:", tpoint1st)
        pretpoint1st = [0, point6[1]+3, -55, 180, 0, 0]
        print("pretpoint1st:", pretpoint1st)
        topointmiddle = [0+2, point6[1]+3, z, 180, 0, 0]
        print("topointmiddle:", topointmiddle)
        tpoint2nd = [-distancetop/2, point7[1]+3, z, 180, 0, 0]
        print("tpoint2nd:", tpoint2nd)
        pretpoint2nd = [-distancetop/2, point7[1]+3, -55, 180, 0, 0]
        print("pretpoint2nd:", pretpoint2nd)


        #Hpoints for top
        htoppoints=[0, point7[1], -70, 180, 0, 0]
        print("htoppoints:", htoppoints)


        toppoints=[pretpoint1st,tpoint1st,topointmiddle,tpoint2nd,pretpoint2nd]
        print("toppoints:", toppoints)

        #Right Side
        rpoint1st = [0, point7[1], z, 180, 0, 0]
        print("rpoint1st:", rpoint1st)
        rpoint2nd = [0, point8[1], z, 180, 0, 0]
        print("rpoint2nd:", rpoint2nd)
        prepoint1st = [0, point7[1], -55, 180, 0, 0]
        print("prepoint1st:", prepoint1st)
        prepoint2nd = [0, point8[1], -55, 180, 0, 0]
        print("prepoint2nd:", prepoint2nd)
        rpointmiddle = [0, point7[1] - 2, z, 180, 0, 0]
        print("rpointmiddle:", rpointmiddle)

        rightpoints=[prepoint1st,rpoint1st,rpointmiddle,rpoint2nd,prepoint2nd]
        print("rightpoints:",rightpoints)


        def perform_process_bottom(cps, config, points1,force):
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
                if point==bpointmiddle:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==bpoint2nd:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                if point==lpointmiddle:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==lpoint2nd:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                if point==topointmiddle:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==tpoint2nd:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                #tcp=config['coords']['tcptool3plane1'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==rpointmiddle:putForceZplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool3plane1'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==rpoint2nd:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool3plane1'],
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
                        tcp=config['coords']['tcptool3plane1'],
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
                        tcp=config['coords']['tcptool3plane1'],
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
    
        # #Bottom Cycles
        communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
        communicate(cps=cps,config=config,point=hbpoint1st,tcp=config['coords']['tcptool3plane1'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
        perform_process_bottom(cps, config, points1=bpoints,force=force)

        # # #Cycles With Loops
        run_single_movement(robot_point=hbpoint1st, seventh_axis_point=x2, cps=cps, config=config)
        perform_process_bottom(cps, config, points1=bpoints,force=force)
        
        # #BottomExtra Cycle
        run_single_movement(robot_point=hbpoint1st, seventh_axis_point=x3, cps=cps, config=config)

        #LeftPoints
        perform_process_left(cps, config, points1=leftpoints,force=force)
        
        #Top Cycles
        perform_process_top(cps, config, points1=toppoints,force=force)

        # #Cycles
        run_single_movement(robot_point=htoppoints, seventh_axis_point=x2, cps=cps, config=config)
        perform_process_top(cps, config, points1=toppoints,force=force)

        # #Extra Cycle
        run_single_movement(robot_point=htoppoints, seventh_axis_point=x1, cps=cps, config=config)

        #Left Cycle
        perform_process_right(cps, config, points1=rightpoints,force=force)
    #Main Cycle
    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 1000:
            model2thirdbig(force,cps)
        else:
            model2thirdtoolsmall(force,cps)
    else:
        print(f"Invalid ylen value type: {type(xlen)} - expected number or 'null'")

if __name__ == "__main__":
    # Call the function with an appropriate force value
    # Replace 10 with your desired force value
    model2thirdtoolrun(5)

