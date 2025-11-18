# testcommu.py

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus,putForceXplus
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
    pointhome=[0,0,-150,0,0,-90]
    point1pre=[0,-5,45,0,0,-90]
    point1=[0,1,45,0,0,-90]
    point12= [1,1,45,0,0,-90]
    point2=[500,1,45,0,0,-90]
    point2pre=[500,-5,45,0,0,-90]
    point2air=[530,-20,45,0,0,0]

    #Left Cycle 
    pointleft1pre=[517.991,10.699,44.994,0,0,0]
    pointleft1=[507.582,10.699,44.994,0,0,0]
    pointleft12=[507.582,11.699,44.994,0,0,0]
    pointleft2=[507.582,645.122,44.994,0,0,0]
    pointleft2pre=[517.991,645.122,44.994,0,0,0]
    pointleft2air=[521.172,667.503,15,0,0,90]

    #Top Cycle
    pointtop1pre=[500,658,45,0,0,90]
    pointtop1=[500,655,45,0,0,90]
    pointtop12=[499,655,45,0,0,90]
    pointtop2=[0,655,45,0,0,90]
    pointtop2pre=[0,658,45,0,0,90]
    pointtopair= [-35,693,4,0,0,-180]

    #Right Cycle
    topright1=[-3,645,45,0,0,-180]
    topright12=[-3,644,45,0,0,-180]
    topright1pre=[-6,645,45,0,0,-180]
    topright2=[-3,0,45,0,0,-180]
    topright2pre=[-6,0,45,0,0,-180]
    toprightout=[-10,0,45,0,0,-180]
    toprighthoming=[-10,0,-100,0,0,-180]


    points1=[point1pre,point1,point12,point2,point2pre]
    pointl1=[pointleft1pre,pointleft1,pointleft12,pointleft2,pointleft2pre]
    pointt1=[pointtop1pre,pointtop1,pointtop12,pointtop2,pointtop2pre]
    pointr1=[topright1pre,topright1,topright12,topright2,topright2pre]

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
            if point==point12:putForceYplus1(
            cps=cps,
            force=10,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
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
            if point==pointleft12:putForceXminus(
            cps=cps,
            force=10,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
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
    
    def perform_process_lefttop(cps, config, points1):
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
            if point==pointleft12:putForceYminus1(
            cps=cps,
            force=10,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
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
    
    def perform_process_right(cps, config, points1):
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
            if point==topright12:putForceXplus(
            cps=cps,
            force=10,
            tcp=config['coords']['tcpSideTool'],
            ucs=config['coords']['ucsTable2'],
            config=config
            )
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


    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    #turn_vibration_on(cps)
    #communicate(cps=cps,config=config,point=point1pre,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_top(cps, config, points1=points1)
    communicate(cps=cps,config=config,point=point2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_left(cps, config, points1=pointl1)
    communicate(cps=cps,config=config,point=pointleft2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_lefttop(cps, config, points1=pointt1)
    communicate(cps=cps,config=config,point=pointtopair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    perform_process_right(cps, config, points1=pointr1)
    communicate(cps=cps,config=config,point=toprightout,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    communicate(cps=cps,config=config,point=toprighthoming,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)

    
    


if __name__ == "__main__":
    main()
    