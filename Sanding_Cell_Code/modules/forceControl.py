from CPS import CPSClient
import time, yaml
import math

cps = CPSClient()
IP = "192.168.0.10"
port = 10003
nret = cps.HRIF_Connect(0, IP, port)
print(f"connected: ")
pluginRes = []

def setSpeed(cps, speed, config):
        """Sets the speed of the cobot

        Args:
            cps (cps): cps
            speed (float): expects speed to be in [0,1] range
        """
        if speed is not None:
            nRet = cps.HRIF_SetOverride(0,0, speed)
            time.sleep(0.2)
            if(config['settings']['debug']): 
                if nRet == 0:
                    print(f'[setSpeed] Could set speed to {speed * 100}%')
                else:
                    print("[setSpeed] Couldn't set speed")

def setUCS_TCP(cpsclient, tcp, ucs, config):
    # Step 1: Set the UCS
    nRet = cpsclient.HRIF_SetUCSByName(0,0,ucs)
    time.sleep(0.5)
    if(config['settings']['debug']): 
        if nRet == 0: print(f'[setUCS_TCP] Success in setting UCS to: {ucs}')
        else        : 
            print(f'[setUCS_TCP] Failure in setting UCS to: {ucs}')
            # return
    
    # Step 2: Set the TCP
    nRet = cpsclient.HRIF_SetTCPByName(0,0,tcp)
    time.sleep(0.5)
    if(config['settings']['debug']): 
        if nRet == 0: print(f'[setUCS_TCP] Success in setting TCP to: {tcp}')
        else        : 
            print(f'[setUCS_TCP] Failure in setting TCP to: {tcp}')
            # return

def putForce(cps, force, tcp, ucs, config, goal=[0, 0, 1]):
    # Initialize parameters
    boxID = 0         # Control box ID
    rbtID = 0         # Robot ID

    
    result = []
    nret = 0
    
    setUCS_TCP(cpsclient=cps, tcp=tcp, ucs=ucs, config=config)
    setSpeed(cps, speed=0.1, config=config)
    nRet = cps.HRIF_SetForceZero(0,0)
    print("Force State: ", nRet)
    time.sleep(0.1)

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0, result)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"forcetoolcoordinate: {nret}, result: {result}")
    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 1)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"force strategy: {nret}")

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    # freedom = [1 if x != 0 else 0 for x in goal]
    # freedom += [0,0,0]
    freedom = goal + [0,0,0]
    print(f"freedom : {freedom}")
    time.sleep(0.2)
    cps.HRIF_SetControlFreedom(0,0,freedom)  # force control degree of freedom
    # Set maximum search velocities for force control
    linear_velocity = 50  # mm/s
    angular_velocity = 10  # °/s
    nret = cps.HRIF_SetMaxSearchVelocities(boxID, rbtID, linear_velocity, angular_velocity)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"search velocities: {nret}")
    
    # SetthePIDparameter 
    # Set PID parameters to ensure stability in force control
    dFp=0.8
    dFi=0.001
    dFd=0.02
    dTp=0.8
    dTi=0.001
    dTd=0.02

    # dFp=0.5
    # dFi=0.1
    # dFd=0
    # dTp=0.5
    # dTi=0.1
    # dTd=0
    #SetthePIDparameter 
    nRet=cps.HRIF_SetPIDControlParams(0,0,dFp,dFi,dFd,dTp,dTi,dTd)
    time.sleep(0.2)
    
    force_goal = [force * goal[0], force * goal[1], -force * goal[2], 0, 0, 0, 0]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    print(f"force goal: {force_goal}")
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"force control goal: {nret}")
    # Enable force control

    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.1)

    notFound = True
    while notFound:
        result = []
        nRet = cps.HRIF_ReadFTCabData(0,0,result)
        print(f"Force in y that is coming is: {result}")
        # nRet = cps.HRIF_ReadForceControlState(0,0,result)
        # print(f"result_1 that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[i])) > abs(force):
                time.sleep(0.2)
                notFound = False
                break
        
        time.sleep(0.2)
    
    if (config['settings']['debug']) : print(f"[forceControl] applying force: {force}N")

def putForce_old(cps, force, tcp, ucs, config, goal=[0, 0, 1]):
    # Initialize parameters
    boxID = 0         # Control box ID
    rbtID = 0         # Robot ID

    
    result = []
    nret = 0
    
    setUCS_TCP(cpsclient=cps, tcp=tcp, ucs=ucs, config=config)

    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 0, result)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"forcetoolcoordinate: {nret}, result: {result}")
    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"force strategy: {nret}")

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = goal + [0,0,0]
    time.sleep(0.2)
    cps.HRIF_SetControlFreedom(0,0,freedom)  # force control degree of freedom
    # Set maximum search velocities for force control
    linear_velocity = 100  # mm/s
    angular_velocity = 5  # °/s
    nret = cps.HRIF_SetMaxSearchVelocities(boxID, rbtID, linear_velocity, angular_velocity)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"search velocities: {nret}")
    
    # SetthePIDparameter 
    # Set PID parameters to ensure stability in force control
    dFp=0.7
    dFi=0.001
    dFd=0.04
    dTp=0.7
    dTi=0.001
    dTd=0.04

    # dFp=0.5
    # dFi=0.1
    # dFd=0
    # dTp=0.5
    # dTi=0.1
    # dTd=0
    #SetthePIDparameter 
    nRet=cps.HRIF_SetPIDControlParams(0,0,dFp,dFi,dFd,dTp,dTi,dTd)
    
    force_goal = [force * goal[0], force * goal[1], -force * goal[2], 0, 0, 0, 0]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    time.sleep(0.2)
    if (config['settings']['debug']) : print(f"force control goal: {nret}")
    # Enable force control

    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    time.sleep(0.1)

    notFound = True
    while notFound:
        result = []
        nRet = cps.HRIF_ReadFTCabData(0,0,result)
        # print(f"Force in y that is coming is: {result}")
        # nRet = cps.HRIF_ReadForceControlState(0,0,result)
        # print(f"result_1 that is coming is: {result}")

        for i, val in enumerate(goal):
            if val and abs(float(result[1])) > force:
                time.sleep(0.2)
                notFound = False
                break
        
        time.sleep(0.2)
    
    if (config['settings']['debug']) : print(f"[forceControl] applying force: {force}N")
    
def releaseForce(cps, config):
    # Initialize parameters
    boxID = 0         # Control box ID
    rbtID = 0         # Robot ID
    # Disable force control after the movement is completed
    cps.HRIF_SetForceControlState(boxID, rbtID, 0)
    if (config['settings']['debug']) : print(f"[forceControl] releasing force: 0N")

with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)

tcp = "TCP_toolVert"
ucs = "Plane_1"

print("in force")

# putForce(cps=cps, force=20, goal=[0, -1, 0], tcp=tcp, ucs=ucs, config=config)
# input("Stop?")
# releaseForce(cps=cps, config=config)
# pluginRes = []

# # Define the Return value list 
result = [] 
nErrorCode = 20006
# Call the interface 
nRet = cps.HRIF_GetErrorCodeStr(0,nErrorCode,result)
print(f"for error: {nRet}, {result}")

# nret = cps.HRIF_ReadCurFSM(0, 0, result)
# print(f"result: {result}")