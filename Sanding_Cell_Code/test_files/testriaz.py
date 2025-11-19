import yaml
import math
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus,putForceXplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading
from Table3Model.exportpointsmodule import exported_points
import time
import multiprocessing
import logging

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def applying_movement_process_bottom(config, points1, movement_done):
    # Set up logging for this process
    logger = setup_logger(config['settings']['debug'])
    config['logger'] = logger
    config['logger'].info("Starting movement process for bottom")

    # Create a new CPS client connection for this process
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    if ret != 0:
        config['logger'].error(f"Failed to connect to robot in movement process: {ret}")
        movement_done.set()  # Signal that movement is complete even if there's an error
        return

    # Signal that we're about to start moving to allow force to be applied
    config['logger'].info("Movement about to start, allowing force application...")
    time.sleep(1.0)  # Give time for force process to start applying force

    for i, point in enumerate(points1):
          communicate(
              cps=cps,
              config=config,
              tcp=config['coords']['tcpSideTool'],
              ucs=config['coords']['ucsTable2'],
              point=point,
              seventh=-1,
              speed=0.2,  # Slower speed to allow more time for force application
              wait=True   # Wait for each point to complete
          )
          config['logger'].info(f"Moving to point {i+1}/{len(points1)}")
          time.sleep(0.5)  # Small delay between points

    # Wait for blending to finish
    waitForBlending(cps=cps, config=config)

    # Add a delay before setting movement_done to ensure force is applied through the full movement
    config['logger'].info("Movement paths completed, waiting before finishing...")
    time.sleep(2.0)

    movement_done.set()  # Signal that movement is complete
    config['logger'].info("Movement process completed")

    # Disconnect the CPS client
    cps.HRIF_DisConnect(0)

def applying_force_process_bottom(config, force, points1, movement_done):
    # Set up logging for this process
    logger = setup_logger(config['settings']['debug'])
    config['logger'] = logger
    config['logger'].info(f"Starting force control process with force: {force}N")

    # Create a new CPS client connection for this process
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    if ret != 0:
        config['logger'].error(f"Failed to connect to robot in force process: {ret}")
        return

    # Apply initial force setup
    config['logger'].info("Setting up initial force control...")
    putForceYplus1(
        cps=cps,
        force=force,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        config=config
    )
    config['logger'].info(f"Initial force of {force}N applied, will maintain throughout movement")

    # Continuously monitor and maintain force until movement is done
    force_count = 0
    while not movement_done.is_set():
        try:
            # Apply force less frequently but maintain it throughout movement
            if force_count % 10 == 0:  # Apply force every ~1 second (10 * 0.1s)
                putForceYplus1(
                    cps=cps,
                    force=force,
                    tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'],
                    config=config
                )
                config['logger'].info(f"Maintaining force: {force}N")
            force_count += 1
            time.sleep(0.001)  # Small delay between force checks
        except Exception as e:
            config['logger'].error(f"Error in force control: {str(e)}")
            break

    config['logger'].info("Force process completed")
    # Disconnect the CPS client
    cps.HRIF_DisConnect(0)

def perform_process_bottom(cps, config, points1):
    # Vibration on
    turn_vibration_on(cps)
    config['logger'].info("Starting bottom process with parallel force control")

    # Create a shared event for process synchronization
    movement_done = multiprocessing.Event()

    # Start force process first to ensure force is established before movement
    

    # Give time for force control to initialize before starting movement
    # time.sleep(1.0)

    # Start movement process
    movement_process = multiprocessing.Process(target=applying_movement_process_bottom, args=(config, points1, movement_done))
    movement_process.start()

    force_process = multiprocessing.Process(target=applying_force_process_bottom, args=(config, 25, points1, movement_done))
    force_process.start()

    # Wait for both processes to complete
    movement_process.join()
    force_process.join()
    config['logger'].info("Movement and force processes completed")

    # Wait for blending to complete
    waitForBlending(cps=cps, config=config)

    # Add a small delay to ensure all processes have fully completed
    time.sleep(0.5)

    # Release Force Control first
    releaseForce(cps=cps, config=config)
    config['logger'].info("Force control released")

    # Then turn off vibration
    turn_vibration_off(cps)
    config['logger'].info("Vibration turned off")

    # Add another small delay to ensure vibration is fully off
    time.sleep(0.5)
    config['logger'].info("Bottom process completed")

def main():
    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    # Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

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

    #Frame of Outside
    frameOutside=p12[0]-p4[0]
    print("frameOutside=",frameOutside)

    #Bottom Points
    pointhome=[p4[0],p4[1],-150,0,0,-90]
    print("pointhome=",pointhome)
    point1pre=[p4[0],p4[1]-5,p4[2]+45,0,0,-90]
    print("point1pre=",point1pre)
    point1=[p4[0],p4[1]+0,p4[2]+45,0,0,-90]
    print("point1=",point1)
    point12=[p4[0]+1,p4[1]+0,p4[2]+45,0,0,-90]
    print("point12=",point12)
    point2=[p1[0]/5,p1[1]+0,p1[2]+45,0,0,-90]
    print("point2=",point2)
    point2pre=[p1[0]/5,p1[1]-5,p1[2]+45,0,0,-90]
    print("point2pre=",point2pre)
    point2air=[p1[0]/5,p1[1]-20,p1[2]+45,0,0,-90]
    print("point2air=",point2air)
    point1Combo=[0,p1[1]-20,p1[2]+45,0,0,-90]
    print("point1Combo=",point1Combo)

    #Bottom Extra point
    point2bottomextra=[p1[0]/5+30,p1[1]-20,p1[2]+45,0,0,0]
    print("point2bottomextra=",point2bottomextra)

    #Left Points
    pointleft1=[(p1[0]/5)+3,frameOutside/2,p1[2]+45,0,0,0]
    print("pointleft1=",pointleft1)
    pointleft1pre=[(p1[0]/5)+2+5,frameOutside/2,p1[2]+45,0,0,0]
    print("pointleft1pre=",pointleft1pre)
    pointleft12=[(p1[0]/5)+3,(frameOutside/2)+1,p1[2]+45,0,0,0]
    print("pointleft12=",pointleft12)
    pointleft2=[(p1[0]/5)+3,p2[1]-(frameOutside/2),p1[2]+45,0,0,0]
    print("pointleft2=",pointleft2)
    pointleft2pre=[(p1[0]/5)+2+5,p2[1]-(frameOutside/2),p1[2]+45,0,0,0]
    print("pointleft2pre=",pointleft2pre)
    #Left Extra Points
    pointLeftExtra=[(p1[0]/5)+21,p2[1]+20,p1[2]+15,0,0,90]
    print("pointLeftExtra=",pointLeftExtra)


    #Top Cycle Points
    pointtop1=[(p1[0]/5),p2[1]+2,p1[2]+45,0,0,90]
    print("pointtop1=",pointtop1)
    pointtop1pre=[(p1[0]/5),p2[1]+2+8,p1[2]+45,0,0,90]
    print("pointtop1pre=",pointtop1pre)
    pointtop12=[(p1[0]/5)-1,p2[1]+2,p1[2]+45,0,0,90]
    print("pointtop12=",pointtop12)
    pointtop2=[(p4[0]/5),p2[1]+2,p1[2]+45,0,0,90]
    print("pointtop2=",pointtop2)
    pointtop2pre=[(p4[0]/5),p2[1]+2+8,p1[2]+45,0,0,90]
    print("pointtop2pre =",pointtop2pre)
    pointtop2air=[(p4[0]/5),p2[1]+2+3+10,p1[2]+45,0,0,90]
    print("pointtop2air =",pointtop2air)
    pointtop1Combo=[(p1[0]/5),p2[1]+2+3+10,p1[2]+45,0,0,90]
    print("pointtop1Combo =",pointtop1Combo)

    #Top Cycle Points Extra
    pointtopExtra=[(p4[0]/5)-6,p2[1]+2+3+10,p1[2]+15,0,0,180]
    print("pointtopExtra =",pointtopExtra)

    #RightCycle
    pointright1=[(p4[0]/5)-1,p2[1],p1[2]+45,0,0,180]
    print("pointright1 =",pointright1)
    pointright1pre=[(p4[0]/5)-1-4,p2[1],p1[2]+45,0,0,180]
    print("pointright1pre =",pointright1pre)
    pointright12=[(p4[0]/5)-1,p2[1]-1,p1[2]+45,0,0,180]
    print("pointright12 =",pointright12)
    pointright2=[(p4[0]/5)-1,p4[0],p1[2]+45,0,0,180]
    print("pointright2 =",pointright2)
    pointright2pre=[(p4[0]/5)-1-4,p4[0],p1[2]+45,0,0,180]
    print("pointright2pre =",pointright2pre)
    homelast=[0,0,-50,0,0,180]
    print("homelast =",homelast)


    #Converyer Points for X axis
    cx=p4[0]
    print("cx=",cx)
    incDistance=(p1[0]-p4[0])/5
    print("incDistance=",incDistance)
    cx1=incDistance
    print("cx1=",cx1)
    cx2=incDistance*2
    print("cx2=",cx2)
    cx3=incDistance*3
    print("cx3=",cx3)
    cx4=incDistance*4
    print("cx4=",cx4)
    cx5=incDistance*5
    print("cx5=",cx5)

    #Array of points
    pointsb=[point1pre,point1,point12,point2,point2pre]
    print("pointsb=",pointsb)
    pointsleft=[pointleft1pre,pointleft1,pointleft12,pointleft2,pointleft2pre]
    print("pointsleft=",pointsleft)
    pointstop=[pointtop1pre,pointtop1,pointtop12,pointtop2,pointtop2pre]
    print("pointstop=",pointstop)
    pointsright=[pointright1pre,pointright1,pointright12,pointright2,pointright2pre]
    print("pointsright=",pointsright)



    #Bottom Cycle 1
    communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.7,wait=True)
    communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.7,wait=True)
    perform_process_bottom(cps, config, points1=pointsb)
    #Bottom Cycle 2








if __name__ == "__main__":
    main()