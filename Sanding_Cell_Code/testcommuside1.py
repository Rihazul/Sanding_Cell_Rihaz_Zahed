# testcommu.py

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config







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

    #Bottom Side Cycle
    vhome=[237.625,84.861,-194.081,0,0,-90]
    verticalpre=[0,-20.708,18.892,0,0,-90]
    verticalp1pre=[0,-4,45,0,0,-90]
    verticalp1=[0,1,45,0,0,-90]
    verticalpoint2=[660,1,45,0,0,-90]
    verticalpoint2pre=[660,-4,45,0,0,-90]
    verticalpoint2preair=[660,-20,45,0,0,-90]
    points1=[verticalp1pre,verticalp1,verticalpoint2,verticalpoint2pre]

    #Left Cycle
    vleftair=[3,-30,0,0,0,0]
    vleftprepoint1=[3,0,45,0,0,0]
    vleftp1=[0,0,45,0,0,0]
    vleftp2=[0,917.575,45,0,0,0]
    vleftprepoint2=[3,917.575,45,0,0,0]
    pointsleft=[vleftprepoint1,vleftp1,vleftp2,vleftprepoint2]

    #Top Cycle
    vtoppointleftsafe=[5,928.114,4.833,0,0,90]
    #Top Cycle Main
    vtopair=[0,928,45,0,0,90]
    vtopairprepoint1=[0,919,45,0,0,90]
    vtopairpoint1=[0,916,45,0,0,90]
    vtopairpoint2=[-440,916,45,0,0,90]
    vtopairprepoint2=[-440,919,45,0,0,90]

    pointstop=[vtopairprepoint1,vtopairpoint1,vtopairpoint2,vtopairprepoint2]

    def perform_process(cps, config, points1):
        # Vibration on
        turn_vibration_on(cps)
        
        # Force Control Activated
        putForceYplus1(
            cps=cps,
            force=20,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        
        # Communicate to each point in points1
        for point in points1:
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=0.2,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)
    
    def perform_processXminus(cps, config, points1):
        # Vibration on
        turn_vibration_on(cps)
        
        # Force Control Activated
        putForceXminus(
            cps=cps,
            force=20,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        
        # Communicate to each point in points1
        for point in points1:
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=0.2,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)
    
    def perform_process_top(cps, config, points1):
        # Vibration on
        turn_vibration_on(cps)
        
        # Force Control Activated
        putForceYminus1(
            cps=cps,
            force=20,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        
        # Communicate to each point in points1
        for point in points1:
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=0.2,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        
        # Release Force Control
        releaseForce(cps=cps, config=config)

    # Define wrapper functions for the two commands you want to run concurrently.
    #Bottom Cycle 1
    def run_robot_movement():
        communicate(cps=cps, config=config, point=verticalpre, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement():
        communicate(cps=cps, config=config, seventh=660, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    #Bottom Cycle 2
    def run_robot_movement1():
        communicate(cps=cps, config=config, point=verticalpre, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement1():
        communicate(cps=cps, config=config, seventh=1320, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    #Bottom Cycle 1
    robot_thread = threading.Thread(target=run_robot_movement)
    axis_thread = threading.Thread(target=run_axis_movement)
    #Bottom Cycle 2
    robot_thread1 = threading.Thread(target=run_robot_movement1)
    axis_thread1 = threading.Thread(target=run_axis_movement1)
    #communicate(cps=cps,config=config,seventh=conbx,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #turn_vibration_on(cps)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p4bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p1bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p4bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #putForceYplus(cps=cps, force=5, tcp=config['coords']['tcpSideTool'], ucs=config['coords']['ucsTable2'], config=config)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p4bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p1bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #releaseForce(cps=cps, config=config)
    #for point in points:
        #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
        #putForceYplus(cps=cps, force=1, tcp=config['coords']['tcpSideTool'], ucs=config['coords']['ucsTable2'], config=config)
        #communicate(
        #cps=cps,
        #config=config,
        #point=point,
        #tcp=config['coords']['tcpSideTool'],
        #ucs=config['coords']['ucsTable2'],
        #seventh=-1,
        #speed=0.3,
        #wait=False
    #)
    #waitForBlending(cps=cps, config=config)
    #releaseForce(cps=cps, config=config)

    # Define the points to iterate over


    #communicate(cps=cps,config=config,point=vhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=verticalpre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=verticalp1,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #putForceYplus(cps=cps, force=5, tcp=config['coords']['tcpSideTool'], ucs=config['coords']['ucsTable2'], config=config)
    #communicate(cps=cps,config=config,point=verticalp1,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=verticalpoint2,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #releaseForce(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    communicate(cps=cps,config=config,point=vhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    communicate(cps=cps,config=config,point=verticalpre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    communicate(cps=cps,config=config,point= verticalp1pre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process(cps, config, points1=points1)
    communicate(cps=cps,config=config,point= verticalpoint2preair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    axis_thread.start()
    robot_thread.start()
    # Wait for both threads to complete
    robot_thread.join()
    axis_thread.join()
    #communicate(cps=cps,config=config,seventh=660,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    #communicate(cps=cps,config=config,point=verticalpre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    communicate(cps=cps,config=config,point= verticalp1pre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process(cps, config, points1=points1)
    communicate(cps=cps,config=config,point= verticalpoint2preair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    axis_thread1.start()
    robot_thread1.start()
    # Wait for both threads to complete
    robot_thread1.join()
    axis_thread1.join()
    communicate(cps=cps,config=config,point=vleftair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,point=vleftprepoint1,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_processXminus(cps, config, points1=pointsleft)
    communicate(cps=cps,config=config,point=vtoppointleftsafe,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,point=vtopair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=vtopairprepoint1,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_top(cps, config, points1=pointstop)
    communicate(cps=cps,config=config,point=vtopair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=880,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    perform_process_top(cps, config, points1=pointstop)
    communicate(cps=cps,config=config,point=vtopair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=440,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    perform_process_top(cps, config, points1=pointstop)

    
    


if __name__ == "__main__":
    main()
    