# bottomcycle
import time
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient
import math

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
    

def euclidean_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points in 3D space.

    Args:
        point1 (list or tuple): Coordinates of the first point [x1, y1, z1].
        point2 (list or tuple): Coordinates of the second point [x2, y2, z2].

    Returns:
        float: Euclidean distance between the two points.
    """
    if len(point1) != 3 or len(point2) != 3:
        raise ValueError("Both points must have exactly three coordinates (x, y, z).")
    
    distance = math.sqrt((point2[0] - point1[0]) ** 2 + 
                        (point2[1] - point1[1]) ** 2 + 
                        (point2[2] - point1[2]) ** 2)
        
    return distance

class RobotInteractions:
    
    def __init__(self):
        # Load configuration with console logging enabled
        self.config = load_config()
        self.config['logger'] = setup_logger(self.config['settings']['debug'])
        self.cps = CPSClient()
        pass 

    def initialize_comms(self):
        print("Initializing communications...")
        # Connect to robot
        IP = self.config['server']['cpip']
        port = self.config['server']['cps']
        ret = self.cps.HRIF_Connect(0, IP, port)

        if ret != 0:
            print(f"Error {ret}, Failed to connect to robot.")
        else:
            print("Successfully connected to robot.")

        
    def seventhGoToPos(self, cps, position, speed, config, wait=True):
        pluginRes = []
        robotRes = []
        PosRes = []
        result = []

        nret = cps.HRIF_HRApp(0, 'HR_Motor','GetMotorList', [], PosRes)
        time.sleep(0.2)
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorConnect', ["J7"], result)
        time.sleep(0.2)
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorGetState', ["J7"], pluginRes)
        time.sleep(0.2)
        nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
        time.sleep(0.2)

        # if(config['settings']['debug']):
        #     config['logger'].info(f'motorList: {PosRes}')
        #     config['logger'].info(f'motorConnect: {result}')
        #     config['logger'].info(f'pluginRes: {pluginRes}')
        #     config['logger'].info(f'robotRes: {robotRes}')
        
        # input("ok?")

        
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorMovePositionSpeed', ["J7", position, speed], result)
        config['logger'].info(f"[7thAxisMove] Going to position: {position}mm with speed: {speed}mm/s")
        time.sleep(0.0001) 
        
        while wait:
            nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorGetState', ["J7"], pluginRes)
            
            # config['logger'].info(f"Trying: {pluginRes}")
            time.sleep(0.00001)
            if pluginRes[2] == '0':
                # means that the robot has reached the position
                break
        
        config['logger'].info(f"[7thAxisMove] Reached position: {position}mm")

class Analysis(RobotInteractions):

    def __init__(self):
        print(
            "This is a placeholder class for analysis purposes."
        )
        super.__init__()
        pass 

    def TestComms(self):
        super.initialize_comms()
        
        pass

    def GoToBaseSafePoint(self):
        speeed = float(self.config['robotSpeed'])
        communicate(cps=self.cps, 
                    point=self.config['point']['safePoint'], 
                    tcp=self.config['coords']['tcpDefault'], 
                    ucs=self.config['coords']['ucsDefault'], 
                    seventh=-1, 
                    config=self.config, 
                    speed=speeed, 
                    wait=True)

        pass

    def run(self):
        pass

    
def communicate(cps, config, tcp, ucs, point=None, seventh= -1, doMeasure = 0, speed=None, stopWhenNan=False, wait=None):

    def euclidean_distance(point1, point2):
        """
        Calculate the Euclidean distance between two points in 3D space.

        Args:
            point1 (list or tuple): Coordinates of the first point [x1, y1, z1].
            point2 (list or tuple): Coordinates of the second point [x2, y2, z2].

        Returns:
            float: Euclidean distance between the two points.
        """
        if len(point1) != 3 or len(point2) != 3:
            raise ValueError("Both points must have exactly three coordinates (x, y, z).")
        
        distance = math.sqrt((point2[0] - point1[0]) ** 2 + 
                            (point2[1] - point1[1]) ** 2 + 
                            (point2[2] - point1[2]) ** 2)
        
        return distance

    def customMoveL(cps, point, tcp, ucs, config, speed, wait=True):
        RawACSpoints = [ 0, 0, 0, 0, 0, 0] # Define the tool coordinate variable 

        nIsSeek = 0 # Define The DI index of the detection 
        nIOBit = 0 # Define The DI status of the detected state www.hansrobot.com 164Interface Description 
        nIOState = 0 # Defining Road Point ID 
        stdCmdID = "0" # Perform road point movement

        setUCS_TCP(cps=cps, tcp=tcp, ucs=ucs, config=config)
        
        # input("Is this fine?")

        nRet = cps.HRIF_MoveL(0,0, point, RawACSpoints, tcp, ucs, config['coords']['roboVelocity'], config['coords']['roboAcceleration'], config['coords']['transitionRadius'], nIsSeek, nIOBit, nIOState, stdCmdID)
        # time.sleep(0.2)
        # config['logger'].info(f"[moveL] nret for moving: {nRet}")
        if nRet == 0:
            robotRes = []
            while wait:
                nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                # config['logger'].info(robotRes)
                if robotRes[11] == '1':
                    break
                # time.sleep(0.2)
            config['logger'].info(f'\n[customMoveL] Could move to {point}')
        else:
            config['logger'].error(f"\n[customMoveL] Couldn't move to {point}")
            msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message="Error In Moving! Please Check Door Size/Robot Settings and Try Again. Terminating Process...")
            exit(-1)

    def data_collection(cps, config, stopWhenNan=False):
        measurements = []
        instrument = getInstrument()
        result = []
        nRet = cps.HRIF_ReadActPos(0,0, result)
        initPos = [float(result[6]), float(result[7]), float(result[8])]
        keepgoing = True
        nan_count = 0
        # Define the consecutive NaN threshold
        consecutive_nan_threshold = 5
        startStopNan = False
        msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Scanning Door Using the Laser Sensor...⚙️")

        while keepgoing:
            height = getRawHeight(instrument)
            
            # Read position and compute distance
            cps.HRIF_ReadActPos(0, 0, result)
            pos = [float(result[6]), float(result[7]), float(result[8])]
            # if config['settings']['debug']:
            #     config['logger'].info(f"pos: {pos}, height: {height}                       ")
            
            if not math.isnan(height):
                startStopNan = True
            # Calculate the distance from initial position
            dist = euclidean_distance(point1=initPos, point2=pos)
            
            # Append measurement
            measurements.append({'height': scale_value(height), 'dist': dist})

            if stopWhenNan and startStopNan:
                # Check the height for NaN and update nan_count
                if math.isnan(height):
                    nan_count += 1  # Increment the NaN counter
                else:
                    nan_count = 0  # Reset the NaN counter if height is valid

                # Check if NaN count reaches the threshold
                if nan_count >= consecutive_nan_threshold:
                    keepgoing = False
                    nRet = cps.HRIF_GrpStop(0, 0)

            # Read robot state and check for another stop condition
            cps.HRIF_ReadRobotState(0, 0, result)
            if result[0] == '0':
                keepgoing = False

        # config['logger'].info("\n\n")
        
        return measurements
    
    def waitWhileMovingRobot(cps, keepgoing=True):
        # keep going?
        # keepgoing = True
        result = []
        while keepgoing:
            cps.HRIF_ReadRobotState(0,0, result)  
            # config['logger'].info(f"Here! waiting now {result}")
            if result[11] == '1':
                keepgoing = False

    def seventhGoToPos(cps, position, speed, config, wait=True):
        pluginRes = []
        robotRes = []
        PosRes = []
        result = []

        nret = cps.HRIF_HRApp(0, 'HR_Motor','GetMotorList', [], PosRes)
        time.sleep(0.2)
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorConnect', ["J7"], result)
        time.sleep(0.2)
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorGetState', ["J7"], pluginRes)
        time.sleep(0.2)
        nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
        time.sleep(0.2)

        # if(config['settings']['debug']):
        #     config['logger'].info(f'motorList: {PosRes}')
        #     config['logger'].info(f'motorConnect: {result}')
        #     config['logger'].info(f'pluginRes: {pluginRes}')
        #     config['logger'].info(f'robotRes: {robotRes}')
        
        # input("ok?")

        
        nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorMovePositionSpeed', ["J7", position, speed], result)
        config['logger'].info(f"[7thAxisMove] Going to position: {position}mm with speed: {speed}mm/s")
        time.sleep(0.0001) 
        
        while wait:
            nret = cps.HRIF_HRApp(0, 'HR_Motor','MotorGetState', ["J7"], pluginRes)
            
            # config['logger'].info(f"Trying: {pluginRes}")
            time.sleep(0.00001)
            if pluginRes[2] == '0':
                # means that the robot has reached the position
                break
        
        config['logger'].info(f"[7thAxisMove] Reached position: {position}mm")
    
    time.sleep(0.0001)

    measurements = []
    # setting the speed of the client
    setSpeed(cps, speed, config=config)
    # time.sleep(0.1)

    if seventh != -1:
        seventhGoToPos(cps=cps, position=seventh, speed=config['UI']['seventhAxisSpeed'] / 100 * config['seventhAxis']['maxSpeed'], config=config, wait=wait if wait is not None else bool(1 - doMeasure))

    if point: 
        customMoveL(cps, point=point, tcp = tcp, ucs= ucs, speed=speed, config=config, wait=wait if wait is not None else bool(1 - doMeasure))

    # the list to collect values in
    if doMeasure: 
        measurements = data_collection(cps, config, stopWhenNan)
    # else:
    #     waitWhileMovingRobot(cps)
    
    # pause script
    # time.sleep(1)
    # try:
    #     input("proceed to the next one?")
    # except EOFError:
    #     config['logger'].info("No input available, proceeding without user confirmation.")

    return measurements

if __name__ == "__main__":
    main()