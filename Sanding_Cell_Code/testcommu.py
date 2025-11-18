# bottomcycle
import time
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Define points and conveyor positions
    spoint=[307.855,291.182,-516.483,180,0,0]

    #Actual Condition
    point1 = [36.26, 48.86,-20, 180, 0, 0]
    point10=[0, 48.86,-20, 180, 0, 0]
    point2= [623.87,48.86,-20, 180, 0, 0]
    point3 = [1284.54, 48.86, -20, 180, 0, 0]
    point5 = [623.87,92,-20,180, 0, 0]
    point6 = [36.26, 92,-20, 180, 0, 0]
    point2y= [0, 0, -20, 180, 0, 0]
    x=36.26  #1st position TCP
    x1=660   #2nd real position\
    x3=0     #initial position
    #x2=1284.54
    #Working Condtion
    #point1= [0, 0, -20, 180, 0, 0]
    #point2= [660, 0, -20, 180, 0, 0]
    #point3= [1320,0,-20,180,0,0]
    #x=0
    #x1=660
    #point1 = [0, 0, -20, 180, 0, 0]
    #point2 = [1320, 0, -20, 180, 0, 0]

    #point3 = [1320, 92, -20, 180, 0, 0]
    #point4 = [0, 92, -20, 180, 0, 0]
    #point1 = [36.26, 48.86, 0, 180, 0, 0]
    #point2 = [1284.54, 48.86, -20, 180, 0, 0]
    #point3 = [1284.54, 90.506, -20, 180, 0, 0]
    #point4 = [0, 90.506, -20, 180, 0, 0]
    #New Logic half and half

    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    # Define wrapper functions for the two commands you want to run concurrently.
    def run_robot_movement():
        communicate(cps=cps, config=config, point=point2, tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.5, wait=True)

    def run_axis_movement():
        communicate(cps=cps, config=config, seventh=x1, tcp=config['coords']['tcpReal'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    
    #For 2nd co-currency
    def run_robot_movement_2():
        communicate(cps=cps,config=config,point=point6,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
        
    
    def run_axis_movement_2():
        communicate(cps=cps,config=config,seventh=x3,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    
     # Create threads for both functions
    robot_thread = threading.Thread(target=run_robot_movement)
    axis_thread = threading.Thread(target=run_axis_movement)
    # For 2nd Thread
    robot_thread_2 = threading.Thread(target=run_robot_movement_2)
    axis_thread_2 = threading.Thread(target=run_axis_movement_2)

    # Initial robot movement (without conveyor)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_int,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point1upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_2nd,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point2upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    ##New movement
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,point=point10,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    axis_thread.start()
    robot_thread.start()
    # Wait for both threads to complete
    #robot_thread.join()
    #axis_thread.join()
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,point=point5,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    #communicate(cps=cps,config=config,point=point6,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    #communicate(cps=cps,config=config,seventh=x3,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    #waitForBlending(cps=cps, config=config)
    robot_thread_2.start()
    axis_thread_2.start()
    #robot_thread_2.join()
    #axis_thread_2.join()
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    


    
    
if __name__ == "__main__":
    main()