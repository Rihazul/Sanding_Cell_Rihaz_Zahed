# testcommu.py 2nd logic
import time
import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Define points and conveyor positions
    spoint=[307.855,291.182,-516.483,180,0,0]
    smpoint=[0,200.182,-30.483,180,0,0]
    point1 = [36.26, 48.86,-1, 180, 0, 0]
    point10=[0, 48.86,-1, 180, 0, 0]
    point2= [624.14,48.86,-1, 180, 0, 0]
    point3 = [1284.54, 48.86, -1, 180, 0, 0]
    point4 = [1284.54, 92, -1, 180, 0, 0]
    point5 = [624.14,92,-1,180, 0, 0]
    point6 = [36.26, 92,-1, 180, 0, 0]
    point2y= [0, 0, -1, 180, 0, 0]

    #Conveyer Movement
    x=point1[0]
    x1=660#Need to take whole table distance/2
    x2=point3[0]
    #Robot Movement
    hdistance=(point3[0]-point1[0])/2
    point1u=[0,point1[1],point1[2],point1[3],point1[4],point1[5]]
    point2mu=point2
    point2my=point1u
    point2u=point2
    point3y=[hdistance,point4[1],point4[2],point4[3],point4[4],point4[5]]
    point3m=[0,point6[1],point6[2],point6[3],point6[4],point6[5]]
    point4m=[hdistance,point4[1],point4[2],point4[3],point4[4],point4[5]]
    point4u=point3m

    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    # Initial robot movement (without conveyor)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_int,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point1upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=smpoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_2nd,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point2upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
        # First, move to the starting point:
    communicate(cps=cps,
            config=config,
            point=smpoint,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.7,
            wait=True)
    turn_vibration_on(cps)
    

# Define the seventh axis positions and corresponding point sequences:
    seventh_axis_sequences = [
        { 
        "axis": x,
        "points": [point1u, point2mu, point3y, point3m, point1u, smpoint]
        },
        { 
        "axis": x1,
        "points": [point1u, point2mu, point3y, point3m, point1u,smpoint]
            }
    ]

# Loop through each seventh axis move and execute the corresponding point moves:
    for seq in seventh_axis_sequences:
    # Change the seventh axis to the desired value:
        communicate(cps=cps,
                config=config,
                seventh=seq["axis"],
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                speed=0.5,
                wait=True)
    
    # Now move through the list of points (with seventh set to -1):
        for pt in seq["points"]:
            communicate(cps=cps,
                    config=config,
                    point=pt,
                    tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=0.3,
                    wait=False)
        waitForBlending(cps=cps, config=config)
    communicate(cps=cps,
            config=config,
            point=smpoint,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.7,
            wait=True)
    turn_vibration_off(cps)
    communicate(cps=cps,
            config=config,
            point=spoint,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.7,
            wait=True)
    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)
    
    communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)


if __name__ == "__main__":
    main()