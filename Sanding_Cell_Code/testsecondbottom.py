# testcommu.py

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus
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
    point1=[33.371,51.061,-1,180,0,0]
    point12=[34.371,51.061,-1,180,0,0]
    point2=[472.367,51.064,-1,180,0,0]
    point3=[472.361,97.970,-1,180,0,0]
    point4=[33.971,97.970,-1,180,0,0]
    point4pre=[33.971,97.970,-3,180,0,0]
    point1pre=[33.371,51.061,-3,180,0,0]
    pointhome=[33.971,97.970,-114,180,0,0]

    points1=[point1pre,point1,point12,point2,point3,point4,point4pre]

    def perform_process_top(cps, config, points1):
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
            if point==point12:putForceZplus(
            cps=cps,
            force=30,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcpReal'],
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


    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    #turn_vibration_on(cps)
    #communicate(cps=cps,config=config,point=point1pre,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_top(cps, config, points1=points1)
    communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    

    
    


if __name__ == "__main__":
    main()
    