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
    verticalp12=[3,1,45,0,0,-90]
    verticalpoint2=[660,1,45,0,0,-90]
    verticalpoint2pre=[660,-4,45,0,0,-90]
    verticalpoint2preair=[660,-20,45,0,0,-90]
    points1=[verticalp1pre, verticalp1,verticalp12,verticalpoint2,verticalpoint2pre,verticalpoint2,verticalp1]

    def perform_process(cps, config, points1):
        # Vibration on
        turn_vibration_on(cps)
        
        # Force Control Activated
        #putForceYplus1(
            #cps=cps,
            #force=20,
            #tcp=config['coords']['tcpSideTool'],
            #ucs=config['coords']['ucsTable2'],
            #config=config
        #)
        
        # Communicate to each point in points1
        for point in points1:
            if point==verticalp12:putForceYplus1(
            cps=cps,
            force=10,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            #elif point==verticalpoint2:releaseForce(cps=cps, config=config)
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
        releaseForce(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    communicate(cps=cps,config=config,point=vhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    communicate(cps=cps,config=config,point=verticalpre,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    perform_process(cps, config, points1=points1)
    

    
    


if __name__ == "__main__":
    main()
    