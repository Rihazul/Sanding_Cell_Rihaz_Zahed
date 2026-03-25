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






def testmodel2sidebig(force,cps):
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
    # p9 = exported_points["p9"]
    # p10 = exported_points["p10"]
    # p11 = exported_points["p11"]
    # p12 = exported_points["p12"]
    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    print(speeed)




    print("p1=",p1)
    print("p2=",p2)
    print("p3=",p3)
    print("p4=",p4)
    print("p5=",p5)
    print("p6=",p6)
    print("p7=",p7)
    print("p8=",p8)
    # print("p9=",p9)
    # print("p10=",p10)
    # print("p11=",p11)
    # print("p12=",p12)
    z=-4


    #Boottom Points
    botlenght=p8[1]-p4[1]
    print("botlenght=",botlenght)
    framesize=p8[0]
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
    length_full=(p1[0]-p4[0])
    print("length_full=",length_full)
    x1=p3[0]
    print("x1=",x1)
    x2=length_full/2
    print("x2=",x2)

    prehoming=[0,0,-20,180,0,0]


    #Movement to bottom points for b
    pointybottomb1=[p4[0],pointybottomb,z,180,0,0]
    print("pointybottomb1=",pointybottomb1)
    prepointybottomb1=[p4[0],pointybottomb,-10,180,0,0]
    print("prepointybottomb1=",prepointybottomb1)
    pointybottomb12m=[p4[0]+2,pointybottomb,z,180,0,0]
    print("pointybottomb12m=",pointybottomb12m)
    pointybottomb2m=[length_full/2,pointybottomb,z,180,0,0]
    print("pointybottomb2m=",pointybottomb2m)
    prepointybottomb2m=[length_full/2,pointybottomb,-10,180,0,0]
    print("prepointybottomb2m=",prepointybottomb2m)


    #Movement to bottom points for a
    pointybottoma1=[p4[0],pointybottoma,z,180,0,0]
    print("pointybottoma1=",pointybottoma1)
    prepointybottoma1=[p4[0],pointybottoma,-10,180,0,0]
    print("prepointybottoma1=",prepointybottoma1)
    pointybottoma12m=[length_full/2+2,pointybottoma,z,180,0,0]
    print("pointybottoma12m=",pointybottoma12m)
    pointybottoma2m=[length_full/2,pointybottoma,z,180,0,0]
    print("pointybottoma2m=",pointybottoma2m)
    prepointybottoma2m=[length_full/2,pointybottoma,-10,180,0,0]
    print("prepointybottoma2m=",prepointybottoma2m)


    bottompointsb=[prepointybottomb1,pointybottomb1,pointybottomb12m,pointybottomb2m,prepointybottomb2m]
    print("bottompointsb=",bottompointsb)
    bottompointsa=[prepointybottoma2m,pointybottoma2m,pointybottoma12m,pointybottoma1,prepointybottoma1]
    print("bottompointsa=",bottompointsa)


    def perform_process_bottom_u(cps, config, points1,force):
        force_applied = False
        vibration_on = False
        force_points = {tuple(pointybottomb12m), tuple(pointybottoma12m)}
        vib_points = {tuple(pointybottomb2m), tuple(pointybottoma1)}

        for point in points1:
            if (not force_applied) and tuple(point) in force_points:
                putForceZplus(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
                force_applied = True
            if (not vibration_on) and tuple(point) in vib_points:
                turn_vibration_on(cps)
                vibration_on = True
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

        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
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

    # Two-pass U pattern for big side:
    u_points = bottompointsb + bottompointsa
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    perform_process_bottom_u(cps, config, points1=u_points,force=force)

    run_single_movement(robot_point=prehoming, seventh_axis_point=x2, cps=cps, config=config)
    perform_process_bottom_u(cps, config, points1=u_points,force=force)

    communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)

    
    # #Bottom Cycle a
    # communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    # perform_process_bottoma(cps, config, points1=bottompointsa,force=force)
    # run_single_movement(robot_point=prehoming, seventh_axis_point=x1, cps=cps, config=config)
    # perform_process_bottoma(cps, config, points1=bottompointsa,force=force)

    # #last cycle
    # communicate(cps=cps,config=config,point=prehoming,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)


if __name__ == "__main__":
    testmodel2sidebig(force=5)
