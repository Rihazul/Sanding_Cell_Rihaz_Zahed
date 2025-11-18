# cc_client.socket_send_string('Hi I am the cobot', name)

from final_async_reader import getRawHeight, getInstrument, scale_value
from getPoints import changeCoordinateFinder
import copy
import math, csv
import yaml
import math, time
from CPS import CPSClient
import numpy as np

filename = 'sensor.csv'

def robot_enable_disable(cps):
    result = []
    nRet = cps.HRIF_ReadRobotState(0, 0, result)
    
    # Enable or disable the group based on the current state
    if result[1] == '0':
        nRet = cps.HRIF_GrpReset(0, 0)
        nRet = cps.HRIF_GrpEnable(0, 0)
    else:
        nRet = cps.HRIF_GrpDisable(0, 0)
    
    return nRet  # Return the result of the enable/disable operation

def adjust_heights(data):
    # Step 1: Filter out dictionaries with non-NaN heights
    non_nan_heights = [d['height'] for d in data if not np.isnan(d['height'])]
    
    # Get the first three and last three non-NaN heights
    first_three_heights = non_nan_heights[:5]
    last_three_heights = non_nan_heights[-7:-3]
    
    # Step 2: Calculate averages
    first_avg = np.mean(first_three_heights)
    last_avg = np.mean(last_three_heights)
    
    # Step 3: Calculate the difference in averages
    avg_diff = last_avg - first_avg
    print(f"averages: {first_avg}, {last_avg}")
    
    # Step 4: Identify indices of first and last non-NaN height occurrences
    first_non_nan_index = next(i for i, d in enumerate(data) if not np.isnan(d['height']))
    last_non_nan_index = next(i for i, d in reversed(list(enumerate(data))) if not np.isnan(d['height']))
    
    # Calculate the number of steps (dicts) between the first and last occurrences
    steps = last_non_nan_index - first_non_nan_index
    
    # Step 5: Incrementally adjust heights
    adjusted_data = []
    cnt = 0
    for i, d in enumerate(data):
        if not np.isnan(d['height']):
            cnt += 1
            # Calculate the incremental adjustment factor
            factor = (cnt / steps) if steps != 0 else 0
            # Adjust the height
            adjusted_height = d['height'] - factor * avg_diff
        else:
            adjusted_height = d['height']  # Keep NaN as is
        adjusted_data.append({'dist': d['dist'], 'height': adjusted_height})
    
    return adjusted_data


def toggle_stopper_status(cps, digital_number=2):
    stopper_status = []
    nRet = cps.HRIF_ReadBoxDO(0, digital_number, stopper_status)
    
    # Check current status and toggle it
    if stopper_status[0] == '1':
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
    else:
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)


# def control_table_open(cps, tableState, config):
#         if tableState == "close":
#             if config['settings']['debug']: print("Closing the table...")

#             digital_number = 0  # DOnumber=0,1,2,3,4
#             nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
#             time.sleep(1)
#             nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
#             if config['settings']['debug']: print("table is now close.")

#         elif tableState == "open":
#             # Open table Code
#             if config['settings']['debug']: print("Opening the table...")

#             digital_number = 1  # DOnumber=0,1,2,3,4
#             nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
#             time.sleep(1)
#             nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
#             if config['settings']['debug']: print("table is now opened.")
#         time.sleep(2)
def control_table_status(cps):
    openTableStatus = []
    nRet = cps.HRIF_ReadBoxDO(0, 1, openTableStatus)
    if openTableStatus[0] == '1':
        nRet = cps.HRIF_SetBoxDO(0, 1, 0) # (0, digital output number, states)
        nRet = cps.HRIF_SetBoxDO(0, 0, 1)
    else:
        nRet = cps.HRIF_SetBoxDO(0, 0, 0)
        nRet = cps.HRIF_SetBoxDO(0, 1, 1)


def tool_changer_status(cps, tool_status, digital_number=5):
    if tool_status == 1:
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
    else:
        nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
    
    return nRet  # Return the result of the last operation

def putForce(cps, force, config):
    # Initialize parameters
    boxID = 0         # Control box ID
    rbtID = 0         # Robot ID

    
    result = []
    nret = 0
    
    # Set tool coordinate system mode for force control
    nret = cps.HRIF_SetForceToolCoordinateMotion(boxID, rbtID, 1, result)
    # if (config['settings']['debug']) : print(f"forcetoolcoordinate: {nret}, result: {result}")
    # Set the force control strategy to constant force mode
    nret = cps.HRIF_SetForceControlStrategy(boxID, rbtID, 0)
    # if (config['settings']['debug']) : print(f"force strategy: {nret}")

    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    freedom = [0,0,1,0,0,0]
    
    cps.HRIF_SetControlFreedom(0,0,freedom)  # force control degree of freedom
    # Set maximum search velocities for force control
    linear_velocity = 50  # mm/s
    angular_velocity = 1  # °/s
    nret = cps.HRIF_SetMaxSearchVelocities(boxID, rbtID, linear_velocity, angular_velocity)
    # if (config['settings']['debug']) : print(f"search velocities: {nret}")
    
    yForce = force * math.cos(math.radians(config['table']['angle']))
    zForce = force * math.sin(math.radians(config['table']['angle']))
    # Define the target force control values (e.g., maintain fixed force in y and z axis)
    
    force_goal = [0, yForce, zForce, 0, 0, 0]  # Target force: [X, Y, Z, Rx, Ry, Rz]
    
    nret = cps.HRIF_SetForceControlGoal(boxID, rbtID, force_goal)
    if (config['settings']['debug']) : print(f"force control goal: {nret}")
    # Enable force control
    cps.HRIF_SetForceControlState(boxID, rbtID, 1)
    # Set PID parameters to ensure stability in force control
    result = []
    while True:
        nRet = cps.HRIF_ReadForceControlState(0,0,result)
        print(f"[force] result that is coming is: {result}     ", end='\r')
        if float(result[0]) == 2:
            time.sleep(0.2)
            break
        time.sleep(0.2)
    
    if (config['settings']['debug']) : print(f"[forceControl] applying force: {force}N")
    toggle_stopper_status(cps, digital_number=4)
    if (config['settings']['debug']) : print(f"[forceControl] Turned on vibration")
    input("Proceed with force?")
    
def releaseForce(cps, config):
    # Initialize parameters
    boxID = 0         # Control box ID
    rbtID = 0         # Robot ID
    # Disable force control after the movement is completed
    toggle_stopper_status(cps, digital_number=4)
    if (config['settings']['debug']) : print(f"[forceControl] Turned on vibration")
    cps.HRIF_SetForceControlState(boxID, rbtID, 0)
    if (config['settings']['debug']) : print(f"[forceControl] releasing force: 0N")

# def homingFunction(cps, config):
#         print(config)
#         def check_return_value(val):
#             if (val != 0):
#                 print("error code: ", val)

#         result = []
#         app_name = 'MT_Kinco'
#         # Go back to safe posture
#         move_to_save_pos = 'MotorMoveToSavePos'
#         velocity = [str(config['UI']['seventhAxisVelocity'])]

#         if(config['settings']['debug']): print("go back to safe posture")

#         #### check if it is behind the 0 line or not, if yes, then only do safePointTool
#         result = [ ] # Read the current actual location information 
#         nRet = cps.HRIF_ReadActPos(0,0, result) # Read the joint position variable 
#         dX = float(result[6])
#         print(f"result5: {result[5]}, result6: {result[6]}")

#         if (dX > 0): 
#             communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
#         communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
#         # ret = cpsclient.HRIF_HRApp(0, app_name, move_to_save_pos, velocity, result) # start to go back safe posture
#         # if(config['settings']['debug']): check_return_value(ret)

#         # while True: # wait for robot move done
#         #     time.sleep(1)
#         #     ret = cps.HRIF_ReadCurFSM(0,0, result)
#         #     if(config['settings']['debug']): check_return_value(ret)
#         #     if (result[0] == '33'):
#         #         break

#         # #
#         # # Find origin
#         # #
#         power_on = 'MotorPowerOn'
#         find_origin_params = ['6'] # aims to 'Find Origin'
#         ret = cps.HRIF_HRApp(0, app_name, power_on, find_origin_params, result) # start to find origin
#         check_return_value(ret)

#         print("start to find origin")
#         while True: # wait for motor move done
#             time.sleep(1)
#             ret = cps.HRIF_HRApp(0, app_name, 'MotorReadCurFSM', [], result) # start to find origin
#             check_return_value(ret)
#             time.sleep(3)
#             if result[0] == '5' or result[0] == '6':
#                 print(result)
#                 break

#         print("success to find origin")

def handle_client(config, homingState=False, startSanding=True):
    # Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)



    def homingFunction(cps, config):
        def check_return_value(val):
            if (val != 0):
                print("error code: ", val)

        print(config)
        result = []
        app_name = 'MT_Kinco'

        if(config['settings']['debug']): print("go back to safe posture")
        #### check if it is behind the 0 line or not, if yes, then only do safePointTool
        result = [ ] # Read the current actual location information 
        nRet = cps.HRIF_ReadActPos(0,0, result) # Read the joint position variable 
        dX = float(result[6])
        print(f"result5: {result[5]}, result6: {result[6]}")

        if (dX > 0): 
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        # ret = cpsclient.HRIF_HRApp(0, app_name, move_to_save_pos, velocity, result) # start to go back safe posture
        # if(config['settings']['debug']): check_return_value(ret)

        # while True: # wait for robot move done
        #     time.sleep(1)
        #     ret = cps.HRIF_ReadCurFSM(0,0, result)
        #     if(config['settings']['debug']): check_return_value(ret)
        #     if (result[0] == '33'):
        #         break

        # #
        # # Find origin
        # #
        power_on = 'MotorPowerOn'
        find_origin_params = ['6'] # aims to 'Find Origin'
        ret = cps.HRIF_HRApp(0, app_name, power_on, find_origin_params, result) # start to find origin
        check_return_value(ret)

        print("start to find origin")
        while True: # wait for motor move done
            time.sleep(1)
            ret = cps.HRIF_HRApp(0, app_name, 'MotorReadCurFSM', [], result) # start to find origin
            # print(f"Homing thing: {result}")
            # robotRes = []
            # nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
            # print(f"Robot thing: {robotRes}")
            time.sleep(0.2)
            check_return_value(ret)
            if result[0] == '5' or result[0] == '6':
                print(result)
                break

        print("success to find origin")

    def mm_to_inches(mm):
        inches = mm / 25.4
        return inches
    def inches_to_mm(inches):
        mm = inches * 25.4
        return mm

    def addXVal(point, val):
        retPoint = copy.deepcopy(point)
        # print(f"val: (before) {point}")
        retPoint[0] -= val
        # print(f"val: (after) {retPoint}")
        return retPoint

    def addYVal(point, val):
        retPoint = copy.deepcopy(point)
        # print(f"val: (before) {point}")
        yVal = math.cos(math.radians(config['table']['angle'])) * val
        zVal = math.sin(math.radians(config['table']['angle'])) * val
        # print(f"yval: {yVal}")
        # print(f"zval: {zVal}")
        retPoint[1] -= yVal
        retPoint[2] += zVal
        # print(f"val: (after) {retPoint}")
        return retPoint
    
    def addZVal(point, val):
        retPoint = copy.deepcopy(point)
        # print(f"val: (before) {point}")
        yVal = math.sin(math.radians(config['table']['angle'])) * val
        zVal = math.cos(math.radians(config['table']['angle'])) * val
        # print(f"yval: {yVal}")
        # print(f"zval: {zVal}")
        retPoint[1] += yVal
        retPoint[2] += zVal
        # print(f"val: (after) {retPoint}")
        return retPoint
    
    def customMoveL(cpsclient, point, tcp, ucs, config):
        RawACSpoints = [ 0, 0, 0, 0, 0, 0] # Define the tool coordinate variable 

        nIsSeek = 0 # Define The DI index of the detection 
        nIOBit = 0 # Define The DI status of the detected state www.hansrobot.com 164Interface Description 
        nIOState = 0 # Defining Road Point ID 
        stdCmdID = "0" # Perform road point movement

        nRet = cpsclient.HRIF_MoveL(0,0, point, RawACSpoints, tcp, ucs, config['coords']['roboVelocity'], config['coords']['roboAcceleration'], config['coords']['transitionRadius'], nIsSeek, nIOBit, nIOState, stdCmdID)
        if(config['settings']['debug']): 
            if nRet == 0:
                print(f'could move to {point}')
            else:
                print("couldn't move to point")
    
    def setSpeed(cpsclient, speed):
        if speed is not None:
            nRet = cpsclient.HRIF_SetOverride(0,0, speed)
            if(config['settings']['debug']): 
                if nRet == 0:
                    print(f'could set speed to {speed * 100}%')
                else:
                    print("couldn't set speed")


    def getTool(cps, toolNumber, config):
        # return
        if (not config['settings']['useTool']):
            return
        
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}home'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        # customMoveL(cps, point=config['point'][f'tool{toolNumber}home'], tcp = config['coords']['tcpTool'], ucs=config['coords']['ucsTool'], config=config)
        control_valve(cps, valveState="open")
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}'], seventh=-1, config=config, speed=0.05)
        time.sleep(1)
        control_valve(cps, valveState="close")
        time.sleep(1)
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}home'], seventh=-1, config=config, speed=0.05)
        # communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])

    def keepTool(cps, toolNumber, config):
        if (not config['settings']['useTool']):
            return
        # return
        communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}home'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}'], seventh=-1, config=config, speed=0.05)
        time.sleep(1)
        control_valve(cps, valveState="open")
        time.sleep(1)
        communicate(cpsclient=cps, point=config['point'][f'tool{toolNumber}home'], seventh=-1, config=config, speed=0.05)
        # communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])

    def control_table(cps, tableState):
        print("Reached! Table State", tableState)
        if tableState == 'close':
            print("table closed")
            nRet = cps.HRIF_SetBoxDO(0, 1, 0) # (0, digital output number, states)
            time.sleep(1)
            nRet = cps.HRIF_SetBoxDO(0, 0, 1)
            time.sleep(1)
        elif tableState == "open":
            print("table opened")
            nRet = cps.HRIF_SetBoxDO(0, 0, 0)
            time.sleep(1)
            nRet = cps.HRIF_SetBoxDO(0, 1, 1)
            time.sleep(1)
        
        # if tableState == "close":
        #     # if config['settings']['debug']: print("Closing the table...")

        #     digital_number = 0  # DOnumber=0,1,2,3,4
        #     nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
        #     time.sleep(1)
        #     nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
        #     # if config['settings']['debug']: print("table is now close.")

        # elif tableState == "open":
        #     # Open table Code
        #     # if config['settings']['debug']: print("Opening the table...")

        #     digital_number = 1  # DOnumber=0,1,2,3,4
        #     nRet = cps.HRIF_SetBoxDO(0, digital_number, 1)
        #     time.sleep(1)
        #     nRet = cps.HRIF_SetBoxDO(0, digital_number, 0)
            # if config['settings']['debug']: print("table is now opened.")
        time.sleep(2)
            
    def control_valve(cps, valveState):
        if valveState == "close":
            # close Valve Code
            print("Opening the valve...")
            
            status = 0  # close=0 and open=1
            digOutput = 5  # DOnumber=0,1,2,3,4
            nRet = cps.HRIF_SetBoxDO(0, digOutput, status) 

            print("Valve is now close.")
        elif valveState == "open":
            # Open Valve Code
            print("Opening the valve...")

            status = 1  # close=0 and open=1
            digOutput = 5  # DOnumber=0,1,2,3,4
            nRet = cps.HRIF_SetBoxDO(0, digOutput, status)

            print("Valve is now opened.")

    def formattedInstruction(point, seventhAxis=-1, debug=False):
        do7th = 0
        if seventhAxis != -1:
            do7th = 1
        all = point + [do7th, seventhAxis]
        # input(f"points: {all}, {type(all)}")
        # input(f"th: {seventhAxis}")
        point_str = ",".join(str(x) for x in all)
        if debug:
            print(f"Points_str: {point_str}")
        return point_str
    
    # def addDistanceInMeasurements(measurements, tot_dist):
    #     print("Measurement: ", len(measurements))
    #     tot_time = measurements[-1]['ct'] - measurements[0]['ct']
    #     for measurement in measurements:
    #         measurement['dist'] = tot_dist * (measurement['ct'] - measurements[0]['ct']) / tot_time
    #         measurement['height'] = scale_value(measurement['height'])
    #     return measurements
    
    def saveAsCSV(fileName, measurements):
        # Specify the order of fieldnames
        ## here ct means 'current time'
        fieldnames = ['dist', 'height']

        # Writing to CSV
        with open(fileName, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            # Write the header
            writer.writeheader()
            # Write the data
            writer.writerows(measurements)
            
    def csv_to_dict_list(file_path):
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            headers = next(reader)  # Read the headers

            # Convert each row's values to float where possible
            def convert_to_float(value):
                try:
                    return float(value)
                except ValueError:
                    return value  # Keep the original value if conversion fails

            # Build the list of dictionaries with converted values
            return [
                {header: convert_to_float(value) for header, value in zip(headers, row)}
                for row in reader
            ]
    
    
    # Step 2: Function to calculate frames and pockets for each chunk using different thresholds
    def calculate_for_each_chunk(chunks, thresholds, default_threshold=(5, 5)):
        """
        Apply the frames and pocket calculation for each chunk with user-defined thresholds.
        If a chunk doesn't have a corresponding threshold, use the default threshold.
        Returns a list of dictionaries containing chunk index and results.
        """
        # Step 3: Function to calculate frames and pockets (the one you provided)
        # def calculate_frames_and_pockets(df, threshold_increase=5, threshold_decrease=5):
        def calculate_frames_and_pockets(data, threshold_increase=5, threshold_decrease=5):
            """
            Identify the significant increase (initial point) and significant decrease (second point)
            and return the frames and pocket based on the thresholds.
            
            Args:
                data (list): List of dictionaries with 'dist' and 'height' as keys.
                threshold_increase (int): The height increase threshold to identify the start of the pocket.
                threshold_decrease (int): The height decrease threshold to identify the end of the pocket.
            
            Returns:
                dict: A dictionary containing information about 'frame_1', 'pocket', and 'frame_2' or None.
            """

            # Filter out NaN values from the 'height' key
            data = [item for item in data if not math.isnan(item['height'])]

            # Ensure there's enough data to proceed
            if len(data) == 0:
                print("No valid data after removing NaN values.")
                return None

            # Get the first and last points in the dataset
            first_point = data[0]
            last_point = data[-1]

            # Find the first point where the height increases by 'threshold_increase' units from the first point
            pocket_start_idx = None
            for i in range(1, len(data)):
                if (data[i]['height'] - first_point['height']) >= threshold_increase:
                    pocket_start_idx = i
                    pocket_start = data[pocket_start_idx]
                    break

            if pocket_start_idx is None:
                print("No significant increase found.")
                return None

            # Find the first point after the pocket start where the height decreases by 'threshold_decrease' units
            pocket_end_idx = None
            for i in range(pocket_start_idx + 1, len(data)):
                if (pocket_start['height'] - data[i]['height']) >= threshold_decrease:
                    pocket_end_idx = i
                    pocket_end = data[pocket_end_idx]
                    break

            if pocket_end_idx is None:
                print("No significant decrease found after the initial increase.")
                return None

            # Calculate the 3 sets:
            # Frame 1: From the first point to the initial point (pocket start)
            distance_frame1 = pocket_start['dist'] - first_point['dist']
            
            # Pocket: From the initial point (pocket start) to the second point (pocket end)
            distance_pocket = pocket_end['dist'] - pocket_start['dist']
            
            # Frame 2: From the second point (pocket end) to the last point
            distance_frame2 = last_point['dist'] - pocket_end['dist']

            # Store the results in a dictionary
            result = {
                'frame_1': {
                    'first_point': first_point['dist'],
                    'second_point': pocket_start['dist'],
                    'distance': distance_frame1
                },
                'pocket': {
                    'first_point': pocket_start['dist'],
                    'second_point': pocket_end['dist'],
                    'distance': distance_pocket
                },
                'frame_2': {
                    'first_point': pocket_end['dist'],
                    'second_point': last_point['dist'],
                    'distance': distance_frame2
                }
            }

            return result


        results = []
        
        for i, chunk in enumerate(chunks):
            # Use provided thresholds if available, otherwise fallback to the default
            if i < len(thresholds):
                threshold_increase, threshold_decrease = thresholds[i]
            else:
                threshold_increase, threshold_decrease = default_threshold
                print(f"\nChunk {i + 1}: No specific threshold provided. Using default thresholds: (increase={threshold_increase}, decrease={threshold_decrease})")

            # Apply the calculation function for each chunk
            print(f"\nProcessing chunk {i + 1} with threshold (increase={threshold_increase}, decrease={threshold_decrease})")
            # chunk_df = pd.Dataframe.from_dict(chunk)
            result = calculate_frames_and_pockets(chunk, threshold_increase=threshold_increase, threshold_decrease=threshold_decrease)
            
            if result:
                # Store the result with the chunk number
                chunk_result = {
                    'chunk_index': i + 1,
                    'frame_1': result['frame_1']['distance'],
                    'pocket': result['pocket']['distance'],
                    'frame_2': result['frame_2']['distance']
                }
                results.append(chunk_result)
            else:
                print(f"No valid results for chunk {i + 1}")
        
        return results

    # def identify_gradient_change_points_dynamic(chunks, thresholds):
    #     """
    #     Identifies the exact points where the gradient starts and stops using start and stop thresholds.

    #     Args:
    #     - distances (list): List of distances.
    #     - heights (list): List of corresponding heights.
    #     - threshold (float): The threshold for detecting the start and stop of a gradient.
    #     - min_distance (float): Minimum distance after which to detect the first gradient change.

    #     Returns:
    #     - gradient_changes (list of dicts): List of points where gradients start and stop with the distance.
    #     """
    #     chunks = [item for item in chunks if not math.isnan(item['height'])]

    #     distances = [item['dist'] for item in chunks]
    #     heights = [item['height'] for item in chunks]
    #     start_threshold = thresholds[0]  # Threshold for detecting the start and stop of a gradient
    #     stop_threshold = thresholds[1]
    #     min_distance = 15  

    #     gradient_changes = []
    #     gradients = np.diff(heights) / np.diff(distances)

    #     positive_start_idx = None
    #     negative_start_idx = None

    #     # Loop through gradients, starting only after the specified minimum distance
    #     for i in range(1, len(gradients)):
    #         if distances[i] < min_distance:
    #             continue  # Skip points until the distance is at least 10mm

    #         # Detect start of positive gradient
    #         if abs(gradients[i]) >= start_threshold and positive_start_idx is None:
    #             positive_start_idx = i

    #         # Detect end of positive gradient
    #         elif positive_start_idx is not None and abs(gradients[i]) < stop_threshold:
    #             gradient_changes.append({
    #                 'type': 'positive_start',
    #                 'distance': distances[positive_start_idx],
    #                 'gradient': gradients[positive_start_idx]
    #             })
    #             gradient_changes.append({
    #                 'type': 'positive_stop',
    #                 'distance': distances[i],
    #                 'gradient': gradients[i]
    #             })
    #             positive_start_idx = None  # Reset

    #         # Detect start of negative gradient
    #         # if gradients[i] <= -start_threshold and negative_start_idx is None:
    #         #     negative_start_idx = i

    #         # # Detect end of negative gradient
    #         # elif negative_start_idx is not None and gradients[i] > -stop_threshold:
    #         #     gradient_changes.append({
    #         #         'type': 'negative_start',
    #         #         'distance': distances[negative_start_idx],
    #         #         'gradient': gradients[negative_start_idx]
    #         #     })
    #         #     gradient_changes.append({
    #         #         'type': 'negative_stop',
    #         #         'distance': distances[i],
    #         #         'gradient': gradients[i]
    #         #     })
    #         #     negative_start_idx = None  # Reset

    #     frame1_point1 = float(chunks[0]['dist'])
    #     frame1_point2 = gradient_changes[0]['distance']
    #     pocket_point1 = gradient_changes[1]['distance']
    #     pocket_point2 = gradient_changes[2]['distance']
    #     frame2_point1 = gradient_changes[3]['distance']
    #     frame2_point2 = float(chunks[-1]['dist'])
    #     # print("frame2_point2: ", frame2_point2)
    #     # print("frame2_point1: ", frame2_point1)
        
    #     frame1 = frame1_point2 - frame1_point1
    #     pocket = pocket_point2 - pocket_point1
    #     threeD1 = pocket_point1 - frame1_point2
    #     threeD2 = frame2_point1 - pocket_point2
    #     frame2 = frame2_point2 - frame2_point1

    #     result = {
    #         'frame_1': frame1,
    #         'threeD_1': threeD1,
    #         'pocket': pocket,
    #         'threeD_2': threeD2,
    #         'frame_2': frame2
    #     }
    #     # print ("Result: ", result)
    #     return result

    def identify_gradient_change_points_dynamic(chunks, threshold, min_stable_distance):
        """
        Identifies stable regions where the change in height is below a specified threshold for a minimum distance.

        Args:
        - chunks (list of dicts): Each dict contains 'dist' and 'height'.
        - threshold (float): The threshold for detecting stable regions based on height change.
        - min_stable_distance (float): Minimum distance required for a region to be considered stable.

        Returns:
        - stable_regions (list of dicts): Each dict contains the start and end distances of a stable region.
        """
        # Filter out NaN values in height data
        chunks = [item for item in chunks if not math.isnan(item['height'])]

        # Extract distances and heights
        distances = [item['dist'] for item in chunks]
        heights = [item['height'] for item in chunks]
        
        stable_start_idx = None  # Track start of stable region
        stable_regions = []

        for i in range(1, len(heights)):
            # Calculate change in height from the previous point
            height_diff = abs(heights[i] - heights[i - 1])

            # Check if the change in height is below the threshold
            if height_diff < threshold:
                if stable_start_idx is None:
                    stable_start_idx = i - 1  # Mark the start of a stable region
            else:
                # If height change exceeds the threshold and we were in a stable region
                if stable_start_idx is not None:
                    # Check if stable region meets the minimum distance requirement
                    start_dist = distances[stable_start_idx]
                    end_dist = distances[i - 1]
                    if end_dist - start_dist >= min_stable_distance:
                        stable_regions.append({
                            'start_distance': start_dist,
                            'end_distance': end_dist
                        })
                    stable_start_idx = None  # Reset stable start index

        # Check the final region if the data ends in a stable region
        if stable_start_idx is not None:
            start_dist = distances[stable_start_idx]
            end_dist = distances[-1]
            if end_dist - start_dist >= min_stable_distance:
                stable_regions.append({
                    'start_distance': start_dist,
                    'end_distance': end_dist
                })

        frame1_point1 = float(chunks[0]['dist'])
        frame1_point2 = stable_regions[0]['start_distance']
        pocket_point1 = stable_regions[0]['start_distance']
        pocket_point2 = stable_regions[0]['end_distance']
        frame2_point1 = stable_regions[0]['end_distance']
        frame2_point2 = float(chunks[-1]['dist'])


        frame1 = frame1_point2 - frame1_point1
        pocket = pocket_point2 - pocket_point1
        threeD1 = pocket_point1 - frame1_point2
        threeD2 = frame2_point1 - pocket_point2
        frame2 = frame2_point2 - frame2_point1

        result = {
                'frame_1': frame1,
                'threeD_1': threeD1,
                'pocket': pocket,
                'threeD_2': threeD2,
                'frame_2': frame2
            }
        
        return result


    
    
    def find_constant_height_periods(measurements, threshold):
        def entireFrame(measurements):
            """Find total frame dist."""
            res = 0
            for i, entry in enumerate((measurements)):
                if entry['height'] > 100:
                    break
                res += entry['length']
            return res 
        
        def flatFrame(measurements):
            """Find total frame dist."""
            res = 0
            for i, entry in enumerate(reversed(measurements)):
                if entry['height'] > 92.5:
                    break
                res += entry['length']
            return res 
        

        periods = []
        current_height = None
        start_dist = None

        # Iterate through the dictionary and remove entries with NaN height
        # measurements_filtered = [measurement for measurement in measurements if not  math.isnan(measurement.get("height", float('inf')))]
        measureNonNan = []
        for item in measurements:
            if math.isnan(item['height']):
                continue
            measureNonNan.append(item)

        for idx, measurement in enumerate(measureNonNan):
            dist = measurement['dist']
            height = measurement['height']
            if math.isnan(height):
                continue
            if current_height is None:
                # Initialize for the first valid measurement
                current_height = height
                start_dist = dist
            elif abs(height - current_height) > threshold:
                # Height changed
                length = dist - start_dist
                periods.append({'height': current_height, 'length': length})
                # Reset for new height
                current_height = height
                start_dist = dist

            # Handle the last period
            elif idx + 1 == len(measureNonNan):
                length = dist - start_dist
                periods.append({'height': current_height, 'length': length})

    
        sumLen = sum(entry['length'] for entry in periods)
        frameFlat = flatFrame(periods)
        frameTot = entireFrame(periods)

        return sumLen, frameFlat, frameTot

    def scan_table(cps, config):
        def find_x_groups(data, n):
            """
            This will return the groups in which valid heights are not NaN.
            A new group is started if n consecutive NaN values are encountered.

            Args:
                data (list): A list of dictionaries containing 'height' values.
                n (int): The number of consecutive NaN values required to start a new group.

            Returns:
                list: A list of groups (each group is a list of dictionaries).
            """
            grouped_valids = []  # To store lists of valid dictionaries
            current_group = []   # To accumulate valid dictionaries in the current group
            nan_count = 0        # To track consecutive NaN values

            for item in data:
                height_value = item.get('height', float('nan'))  # Get the 'height' value, default to NaN if not found

                if math.isnan(float(height_value)):
                    nan_count += 1  # Increment the NaN counter
                else:
                    nan_count = 0   # Reset the NaN counter if a valid value is found
                    current_group.append(item)  # Add valid items to the current group

                # If we encounter n consecutive NaN values, we close the current group
                if nan_count == n:
                    if current_group:
                        grouped_valids.append(current_group)
                        current_group = []  # Reset the current group for the next batch
                    nan_count = 0  # Reset the NaN counter after splitting

            # If there are any valid entries left in the last group, add them too
            if current_group:
                grouped_valids.append(current_group)

            return grouped_valids
        

        # find the 7th axis positions
        robo7thPos = [(mm_to_inches(config['table'][f'lengthx{i}'])) + mm_to_inches(config['offset']['seventhAxis']) for i in range(config['table']['count'])]
        if(config['settings']['debug']): print("robo7thPos: ", robo7thPos)

        outerSandingPoints = []
        innerSandingPoints = []
        edgeSandingPoints = []
        own7thpos = []
        xVals = []
        yVals = []
        allXMeasurements = []

        # tableState = "open"
        control_table(cps, tableState="open")

        # input("Ok?")
        if config['settings']['actualScan']:
            for tblCnt, roboPos in enumerate(robo7thPos):
                communicate(cpsclient=cps, seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                if(config['settings']['debug']): print("table origin done")
                
                ###########################
                ####### x axis moving #####
                ###########################
                xStart = addXVal(addYVal(config['point']['tableOrigin'], config['door']['frame'] + config['offset']['doorYOffset']), -5)
                xEnd = addXVal(addYVal(config['point']['tableOrigin'], config['door']['frame'] + config['offset']['doorYOffset']), config['table'][f'length{tblCnt}'] -5)
                if(config['settings']['debug']): print(f'points: {xStart}, {xEnd}')

                if(config['settings']['debug']): print("x point start")
                communicate(cpsclient=cps, point=xStart, config=config, speed=config['UI']['robotSpeed'])
                if(config['settings']['debug']): print("x point start done")
                if(config['settings']['debug']): print("x point start end")
                xmeasurements = communicate(cpsclient=cps, point=xEnd, config=config, doMeasure=1, speed=config['UI']['scanSpeed'])
                
                if(config['settings']['debug']): print("x point start end done")
                # if(config['settings']['debug']): saveAsCSV('./static/xmeasures.csv', xmeasurements)
                # input("measurement saved, proceed?")
                # add sufficient x distance to the points
                for xmeasurement in xmeasurements:
                    xmeasurement['dist'] += inches_to_mm(roboPos)
                allXMeasurements += xmeasurements
                communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])

            # input("Should I save the values?")
            saveAsCSV('./static/prev_xmeasures.csv', allXMeasurements)
            allXMeasurements = adjust_heights(allXMeasurements)
            if(config['settings']['debug']): saveAsCSV('./static/xmeasures.csv', allXMeasurements)
            if(config['settings']['debug']): print("x values saved in xmeasures.csv")
        else:
            allXMeasurements = csv_to_dict_list('./static/xmeasures.csv')
        ###############################
        ####### group calcul x    #####
        ###############################
        allXMeasurements = find_x_groups(allXMeasurements, 4)
        # reversing to start from the last to first (to save time)
        allXMeasurements.reverse()
        # Define thresholds for each chunk
        xthresholds = [
            (4, 4),  # Thresholds for the fourth chunk (increase, decrease)
            (1, 2),  # Thresholds for the third chunk (increase, decrease)
            (3, 4),  # Thresholds for the second chunk (increase, decrease)
            (4, 4),  # Thresholds for the first chunk (increase, decrease)
        ]
        ythresholds = [
            (4, 3),  # Thresholds for the fourth chunk (increase, decrease)
            (3, 3),  # Thresholds for the third chunk (increase, decrease)
            (4, 3),  # Thresholds for the second chunk (increase, decrease)
            (4, 3),  # Thresholds for the first chunk (increase, decrease)
        ]
        for xcnt, xmeasurements in enumerate(allXMeasurements):
            ###############################
            ####### point calcu x     #####
            ###############################
            xpos = mm_to_inches(xmeasurements[0]['dist']) # this is position of 7th axis
            # xlen, xframe_1, xframe_2 = find_constant_height_periods(xmeasurements, threshold=2)
            # results = calculate_for_each_chunk([xmeasurements], thresholds=[xthresholds[xcnt]])
            results = identify_gradient_change_points_dynamic(xmeasurements, threshold=1, min_stable_distance=100)
            # results = identify_gradient_change_points_dynamic(xmeasurements, thresholds=[0.15, 0.1])
            print("Distance: ", results)
            xframe_1, x_td1,  x_pocket, x_td2, xframe_2 = results['frame_1'], results['threeD_1'], results['pocket'], results['threeD_2'], results['frame_2']
            xlen = xframe_1 + x_td1 + x_pocket + x_td2 + xframe_2
            xVals.append({'xlen':xlen, 'xframe_1': xframe_1 + x_td1, 'xframe_2': xframe_2 + x_td2})

            # xframe_1, x_pocket, xframe_2 = results[0]['frame_1'], results[0]['pocket'], results[0]['frame_2']
            # xlen = xframe_1 + x_pocket + xframe_2
            # xVals.append({'xlen': xlen, 'xframe_1': xframe_1, 'xframe_2': xframe_2})
            # go to the appropriate position in 7th axis
            own7thpos.append(xpos)
            ymeasurements = []
            
            if config['settings']['actualScan']:
                communicate(cpsclient=cps, seventh=xpos, config=config, speed=config['UI']['robotSpeed'])
                if(config['settings']['debug']): print(f"xpos done: {xpos}")
                ###########################
                ####### y axis moving #####
                ###########################
                yStart =addYVal(addXVal(config['point']['tableOrigin'], xlen / 4), -5)
                yEnd = addYVal(addYVal(addXVal(config['point']['tableOrigin'], xlen / 3), config['table']['width']), -5)
                if(config['settings']['debug']): print(f'points: {yStart}, {yEnd}')
                if(config['settings']['debug']): print("y point start")
                communicate(cpsclient=cps, point=yStart, config=config, speed=config['UI']['robotSpeed'])
                if(config['settings']['debug']): print("y point start done")
                if(config['settings']['debug']): print("y point start end")
                ymeasurements = communicate(cpsclient=cps, point=yEnd, config=config, doMeasure=1, speed=config['UI']['scanSpeed'], stopWhenNan=True)
                if(config['settings']['debug']): print("y point start end done")
                # ymeasurements = addDistanceInMeasurements(ymeasurements, config['table']['width'])
                saveAsCSV(f'./static/prev_ym{xcnt}.csv', ymeasurements)
                ymeasurements = adjust_heights(ymeasurements)
                if(config['settings']['debug']): saveAsCSV(f'./static/ym{xcnt}.csv', ymeasurements)
                communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            else:
                ymeasurements = csv_to_dict_list(f'./static/ym{xcnt}.csv')

            ###############################
            ####### point calcu y     #####
            ###############################

            # ylen, yframe_1, yframe_2 = find_constant_height_periods(ymeasurements, threshold=2)
            results = identify_gradient_change_points_dynamic(ymeasurements, threshold=1, min_stable_distance=100)
            # results = identify_gradient_change_points_dynamic(ymeasurements, thresholds=[0.2, 0.1])
            print(f"results: {results}")
            yframe_1, y_td1,  y_pocket, y_td2, yframe_2 = results['frame_1'], results['threeD_1'], results['pocket'], results['threeD_2'], results['frame_2']
            ylen = yframe_1 + y_td1 + y_pocket + y_td2 + yframe_2
            yVals.append({'ylen': ylen, 'yframe_1': yframe_1 + y_td1, 'yframe_2': yframe_2 + y_td2})

            if(config['settings']['debug']): 
                print("\n\n\n\nprinting the values\n\n")
                print("x stuffs: ",  xlen, xframe_1, xframe_2)
                print("y stuffs: ", ylen, yframe_1, yframe_2, y_td1, y_td2)
            
            outerSandingPoints.append({
                0: addZVal(addXVal(addYVal(config['point']['tableOrigin'], config['offset']['toolRadius']/2 - config['offset']['scannerOffsetInBottom']), + config['offset']['scannerOffsetInLeft'] + config['offset']['toolRadius']/2), -100 + config['tool']['tool2height']), 
                1: addZVal(addXVal(addYVal(config['point']['tableOrigin'],ylen - config['offset']['toolRadius']/2 - config['offset']['scannerOffsetInBottom']), + config['offset']['scannerOffsetInLeft'] + config['offset']['toolRadius']/2), -100 + config['tool']['tool2height']), 
                2: addZVal(addXVal(addYVal(config['point']['tableOrigin'],ylen - config['offset']['toolRadius']/2 - config['offset']['scannerOffsetInBottom']), xlen + config['offset']['scannerOffsetInLeft'] - config['offset']['toolRadius']/2), -100 + config['tool']['tool2height']), 
                3: addZVal(addXVal(addYVal(config['point']['tableOrigin'], config['offset']['toolRadius']/2 - config['offset']['scannerOffsetInBottom'] ) , xlen + config['offset']['scannerOffsetInLeft'] - config['offset']['toolRadius']/2), -100 + config['tool']['tool2height']) 
                })
            edgeSandingPoints.append({
                0: addZVal(addXVal(addYVal(config['point']['tableOrigin'],-config['offset']['edgeOffset'] -10 +  config['offset']['toolRadius'] - config['offset']['scannerOffsetInBottom']), -config['offset']['edgeOffset'] + config['offset']['scannerOffsetInLeft'] + config['offset']['toolRadius']), -90 + config['tool']['tool4height']), 
                1: addZVal(addXVal(addYVal(config['point']['tableOrigin'],+config['offset']['edgeOffset'] + ylen - config['offset']['toolRadius'] - config['offset']['scannerOffsetInBottom']), -config['offset']['edgeOffset'] + config['offset']['scannerOffsetInLeft'] + config['offset']['toolRadius']), -90 + config['tool']['tool4height']), 
                2: addZVal(addXVal(addYVal(config['point']['tableOrigin'],+config['offset']['edgeOffset'] + ylen - config['offset']['toolRadius'] - config['offset']['scannerOffsetInBottom']), config['offset']['edgeOffset'] + xlen + config['offset']['scannerOffsetInLeft'] - config['offset']['toolRadius']), -90 + config['tool']['tool4height']), 
                3: addZVal(addXVal(addYVal(config['point']['tableOrigin'],-config['offset']['edgeOffset'] -10 +  config['offset']['toolRadius'] - config['offset']['scannerOffsetInBottom'] ) , config['offset']['edgeOffset'] +  xlen + config['offset']['scannerOffsetInLeft'] - config['offset']['toolRadius']), -90 + config['tool']['tool4height']) 
                })
            innerSandingPoints.append({
                0: addZVal(addXVal(addYVal(config['point']['tableOrigin'], yframe_1 - config['offset']['scannerOffsetInBottom']),  xframe_1 - config['offset']['scannerOffsetInLeft']), config['tool']['tool2height']), 
                1: addZVal(addXVal(addYVal(config['point']['tableOrigin'],ylen - yframe_2 - 2 * config['offset']['toolRadius']- config['offset']['scannerOffsetInBottom']),  xframe_1 - config['offset']['scannerOffsetInLeft']), config['tool']['tool2height']), 
                2: addZVal(addXVal(addYVal(config['point']['tableOrigin'],ylen - yframe_2 - 2 * config['offset']['toolRadius']- config['offset']['scannerOffsetInBottom']), xlen - xframe_2 - 2 * config['offset']['toolRadius'] - config['offset']['scannerOffsetInLeft']), config['tool']['tool2height']), 
                3: addZVal(addXVal(addYVal(config['point']['tableOrigin'], yframe_1 - config['offset']['scannerOffsetInBottom']), xlen - xframe_2 - 2 * config['offset']['toolRadius'] - config['offset']['scannerOffsetInLeft']), config['tool']['tool2height'])
                })

            if(config['settings']['debug']): 
                print(f"outer sanding points: {outerSandingPoints}")
                print(f"inner sanding points: {innerSandingPoints}")
            if config['settings']['actualScan']:
                communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                if(config['settings']['debug']): print("safepoint done")
        
        own7thpos.reverse()
        outerSandingPoints.reverse()
        edgeSandingPoints.reverse()
        innerSandingPoints.reverse()
        xVals.reverse()
        yVals.reverse()

        return own7thpos, outerSandingPoints, innerSandingPoints, edgeSandingPoints, xVals, yVals

    def sand_table(cps, config, robo7thPos, outerSandingPoints, innerSandingPoints, edgeSandingPoints, xVals, yVals):
        def moveOnlyJ6(cps, J6, config):
                result = [ ] # Read the current actual location information 
                nRet = cps.HRIF_ReadActPos(0,0, result) # Read the joint position variable 
                dJ1 = float(result[0]) 
                dJ2 = float(result[1]) 
                dJ3 = float(result[2]) 
                dJ4 = float(result[3]) 
                dJ5 = float(result[4]) 
                dJ6 = float(result[5]) + J6 # Read the spatial position variable 
                
                setSpeed(cps, 0.6)
                
                sTcpName="TCP" #Definetheusercoordinatesvariable 
                sUcsName="Base" #Definemovementspeed
                dVelocity = 100 # Define the movement acceleration 
                dAcc = 100 # Define the transition radius 
                dRadius = 50 # Deine whether the joint angle is used 
                nIsUseJoint= 1 # Define whether to stop using the test DI 
                nIsSeek = 0 # Define The DI index of the detection 
                nIOBit = 0 # Define The DI status of the detected state 
                nIOState = 0 # Defining Road Point ID 
                stdCmdID = "0" # Perform road point movement 

                nRet = cps.HRIF_MoveJ(0,0, config['point']['tableOrigin'], [dJ1, dJ2, dJ3, dJ4, dJ5, dJ6], sTcpName , sUcsName, dVelocity, dAcc, dRadius,nIsUseJoint, nIsSeek, nIOBit, nIOState, stdCmdID)
                print(f"nret {nRet}")
                robotRes = []
                while 1:
                    nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                    if robotRes[0] == '0':
                        print(f'could move to joint pos {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}')
                        return
                    print(f"Trying to move joints to {[dJ1, dJ2, dJ3, dJ4, dJ5, dJ6]}", end="\r")
                    time.sleep(0.2)
        def goToBasicJoints(cps, config):
                
                sTcpName="TCP" #Definetheusercoordinatesvariable 
                sUcsName="Base" #Definemovementspeed
                dVelocity = 100 # Define the movement acceleration 
                dAcc = 100 # Define the transition radius 
                dRadius = 50 # Deine whether the joint angle is used 
                nIsUseJoint= 1 # Define whether to stop using the test DI 
                nIsSeek = 0 # Define The DI index of the detection 
                nIOBit = 0 # Define The DI status of the detected state 
                nIOState = 0 # Defining Road Point ID 
                stdCmdID = "0" # Perform road point movement 

                nRet = cps.HRIF_MoveJ(0,0, config['point']['tableOrigin'], config['point']['tableOriginAngle'], sTcpName , sUcsName, dVelocity, dAcc, dRadius,nIsUseJoint, nIsSeek, nIOBit, nIOState, stdCmdID)
                print(f"nret {nRet}")
                robotRes = []
                while 1:
                    nret = cps.HRIF_ReadRobotState(0, 0, robotRes)
                    if robotRes[0] == '0':
                        return
                    time.sleep(0.2)
                print(f'could move to joint pos')


        def frameCycle(cps, outerSandingPoints, config):
            # Frame Inner cycle
            for frameSandCnt in range(config['UI']['frameSandCount']):
                for i, point in enumerate([0, 1, 2, 3, 0]):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    if(config['settings']['debug']): print(f"[frameCycle] going to point: {point} of table: {tblCnt}")
                    if i != 0: putForce(cps=cps, force=config['UI']['controlPressure'], config=config)
                    cps.waitBlendingDone(0,0)
                    communicate(cpsclient=cps, point=outerSandingPoints[tblCnt][point], config=config, speed=config['UI']['robotSpeed'])
                    if i != 0: releaseForce(cps=cps, config=config)
                    if(config['settings']['debug']): print(f"[frameCycle] reached  point: {point} of table: {tblCnt}")
        
        
        def threeDCycle(cps, innerSandingPoints, config):
            # 3D Sanding Cycle
            for threeDSandCnt in range(config['UI']['threeDSandCount']):
                for point in [0, 1, 2, 3, 0]:
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    if(config['settings']['debug']): print(f"[innerCycle - 3D] going to point: {point} of table: {tblCnt}")
                    if i != 0: putForce(cps=cps, force=config['UI']['controlPressure'], config=config)
                    communicate(cpsclient=cps, point=innerSandingPoints[tblCnt][point], config=config, speed=config['UI']['robotSpeed'])
                    if i != 0: releaseForce(cps=cps, config=config)
                    if(config['settings']['debug']): print(f"[innerCycle - 3D] reached  point: {point} of table: {tblCnt}")

        
        def pocketCycle(cps, yVal, config):
            # pocket cycle
            for pocketSandCnt in range(config['UI']['pocketSandCnt']):
                yinner = yVal['ylen'] - yVal['yframe_1'] - yVal['yframe_2'] - config['offset']['tool3y']
                offset = 0
                toggle = 0
                # this was added to do one extra cycle
                while offset < yinner + config['offset']['innerSandingOffset']:
                    # print(f"calculation being done: {config['point']['tableOrigin'][0]} {xVals[tblCnt]['xlen']} - {xVals[tblCnt]['xframe_2']} - 2 * {config['offset']['toolRadius']} - {config['offset']['scannerOffsetInLeft']}")
                    points = [addZVal( addXVal(addYVal(config['point']['tableOrigin'], yVal['ylen'] - yVal['yframe_2'] - offset - config['offset']['tool3y']/2 - config['offset']['scannerOffsetInBottom']), xVals[tblCnt]['xframe_2']  + config['offset']['tool3x']/2  + config['offset']['scannerOffsetInLeft']), - 100 + config['tool']['tool3height']), 
                            addZVal( addXVal(addYVal(config['point']['tableOrigin'], yVal['ylen'] - yVal['yframe_2'] - offset - config['offset']['tool3y']/2 - config['offset']['scannerOffsetInBottom']), xVals[tblCnt]['xlen'] - xVals[tblCnt]['xframe_2'] - config['offset']['tool3x']/2 + config['offset']['scannerOffsetInLeft']), - 100 + config['tool']['tool3height'])]
                    
                    if toggle:
                        points.reverse()
                    
                    for i, point in enumerate(points):
                        if(config['settings']['debug']): print(f"[pocketCycle] going to point: {point} of table: {tblCnt}")
                        if i != 0: putForce(cps=cps, force=config['UI']['controlPressure'], config=config)
                        communicate(cpsclient=cps, point=point, config=config, speed=config['UI']['robotSpeed'])
                        if i != 0: releaseForce(cps=cps, config=config)
                        if(config['settings']['debug']): print(f"[pocketCycle] reached  point: {point} of table: {tblCnt}")

                    offset += config['offset']['innerSandingOffset']
                    toggle = 1 - toggle

                    if(offset > yinner and offset < yinner + config['offset']['innerSandingOffset']):
                        offset = yinner
            # keepTool(cps, toolNumber=2, config=config)

        def sideCycle(cps, sideSandingPoints, config):
            # outer sanding cycle
            angles = [-90, 90, 90, 90]
            rx = [-135, 180, 135, -180, -135]
            ry = [0, 45, 0, -45, 0]
            rz = [-180, 90, 0, -90, 180]
            for sideSandCnt in range(config['UI']['sideSandCount']):
                forwardOrBackward = 0
                pointList = []
                
                if sideSandCnt % 2:
                    # forwardOrBackward = -1
                    # pointList = [3, 2, 1, 0]
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                else:
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                
                for idxm, point in enumerate(pointList):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    moveOnlyJ6(cps, angles[idxm], config)
                    piont1 =  [sideSandingPoints[tblCnt][point][0], sideSandingPoints[tblCnt][point][1], sideSandingPoints[tblCnt][point][2], rx[idxm], ry[idxm], rz[idxm]]
                    piont2 =  [sideSandingPoints[tblCnt][(point + forwardOrBackward) % 4][0], sideSandingPoints[tblCnt][(point + forwardOrBackward) % 4][1], sideSandingPoints[tblCnt][(point + forwardOrBackward) % 4][2], rx[idxm], ry[idxm], rz[idxm]]
                    
                    if idxm == 0:
                        pointInit = addZVal(piont1, +50)
                        if(config['settings']['debug']): print(f"[sideCycle] going to init point: {pointInit} of table: {tblCnt}")
                        communicate(cpsclient=cps, point=pointInit, config=config, speed=config['UI']['robotSpeed'])
                        if(config['settings']['debug']): print(f"[sideCycle] reached init point: {pointInit} of table: {tblCnt}")
                    
                    if(config['settings']['debug']): print(f"[sideCycle] going to point: {piont1} of table: {tblCnt}")
                    communicate(cpsclient=cps, point=piont1, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print(f"[sideCycle] reached  point: {piont1} of table: {tblCnt}")

                    if(config['settings']['debug']): print(f"[sideCycle] going to point: {piont2} of table: {tblCnt}")
                    putForce(cps=cps, force=config['UI']['controlPressure'], config=config)
                    communicate(cpsclient=cps, point=piont2, config=config, speed=config['UI']['robotSpeed'])
                    releaseForce(cps=cps, config=config)
                    if(config['settings']['debug']): print(f"[sideCycle] reached  point: {piont2} of table: {tblCnt}")
                    
                    if idxm == len(pointList) - 1:
                        pointInit = addZVal(piont2, +50)
                        if(config['settings']['debug']): print(f"[sideCycle] going to init point: {pointInit} of table: {tblCnt}")
                        communicate(cpsclient=cps, point=pointInit, config=config, speed=config['UI']['robotSpeed'])
                        if(config['settings']['debug']): print(f"[sideCycle] reached init point: {pointInit} of table: {tblCnt}")

                        moveOnlyJ6(cps, -180, config)
        
        def edgeCycle(cps, edgeSandingPoints, config):
            # outer sanding cycle
            angles = [-90, 90, 90, 89]
            rx = [-135, 168, 133, 170]
            ry = [25, 69, 21, -6]
            rz = [-158, 82, -13, -86]
            for edgeSandCnt in range(config['UI']['edgeSandCount']):
                forwardOrBackward = 0
                pointList = []
                
                if edgeSandCnt % 2:
                    # forwardOrBackward = -1
                    # pointList = [3, 2, 1, 0]
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                else:
                    forwardOrBackward = 1
                    pointList = [0, 1, 2, 3]
                
                for idxm, point in enumerate(pointList):
                    # data = formattedInstruction(outerSandingPoints[tblCnt][point], debug=config['settings']['debug'])
                    moveOnlyJ6(cps, angles[idxm], config)
                    piont1 =  [edgeSandingPoints[tblCnt][point][0], edgeSandingPoints[tblCnt][point][1], edgeSandingPoints[tblCnt][point][2], rx[idxm], ry[idxm], rz[idxm]]
                    piont2 =  [edgeSandingPoints[tblCnt][(point + forwardOrBackward) % 4][0], edgeSandingPoints[tblCnt][(point + forwardOrBackward) % 4][1], edgeSandingPoints[tblCnt][(point + forwardOrBackward) % 4][2], rx[idxm], ry[idxm], rz[idxm]]
                    
                    if idxm == 0:
                        pointInit = addZVal(piont1, +100)
                        if(config['settings']['debug']): print(f"[edgeCycle] going to init point: {pointInit} of table: {tblCnt}")
                        communicate(cpsclient=cps, point=pointInit, config=config, speed=config['UI']['robotSpeed'])
                        if(config['settings']['debug']): print(f"[edgeCycle] reached init point: {pointInit} of table: {tblCnt}")
                    
                    if(config['settings']['debug']): print(f"[edgeCycle] going to point: {piont1} of table: {tblCnt}")
                    communicate(cpsclient=cps, point=piont1, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print(f"[edgeCycle] reached  point: {piont1} of table: {tblCnt}")

                    if(config['settings']['debug']): print(f"[edgeCycle] going to point: {piont2} of table: {tblCnt}")
                    putForce(cps=cps, force=config['UI']['controlPressure'], config=config)
                    communicate(cpsclient=cps, point=piont2, config=config, speed=config['UI']['robotSpeed'])
                    releaseForce(cps=cps, config=config)
                    if(config['settings']['debug']): print(f"[edgeCycle] reached  point: {piont2} of table: {tblCnt}")

                    if idxm == len(pointList) - 1:
                        pointInit = addZVal(piont2, +100)
                        if(config['settings']['debug']): print(f"[edgeCycle] going to init point: {pointInit} of table: {tblCnt}")
                        communicate(cpsclient=cps, point=pointInit, config=config, speed=config['UI']['robotSpeed'])
                        if(config['settings']['debug']): print(f"[edgeCycle] reached init point: {pointInit} of table: {tblCnt}")

                        moveOnlyJ6(cps, -180, config)


        communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        
        if(config['toolForModels'][f"model{config['UI']['model']}"][0] == 1):
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            getTool(cps,  toolNumber=3, config=config)
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            
            enum7thPos = []
            # flip the list for every odd number of elements
            if sum(config['toolForModels'][f"model{config['UI']['model']}"][:1]) % 2:
                enum7thPos = enumerate(robo7thPos)
            else:
                enum7thPos = reversed(list(enumerate(robo7thPos)))
            
            for tblCnt, roboPos in enum7thPos:
                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print("safepoint done")
                    frameCycle(cps=cps, outerSandingPoints=outerSandingPoints, config=config)
                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
            if(config['toolForModels'][f"model{config['UI']['model']}"][1] != 1):
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                keepTool(cps, toolNumber=3, config=config)

        if(config['toolForModels'][f"model{config['UI']['model']}"][1] == 1):
            if(config['toolForModels'][f"model{config['UI']['model']}"][0] != 1):
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                getTool(cps,  toolNumber=3, config=config)
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            
            enum7thPos = []
            # flip the list for every odd number of elements
            if sum(config['toolForModels'][f"model{config['UI']['model']}"][:2]) % 2:
                enum7thPos = enumerate(robo7thPos)
            else:
                enum7thPos = reversed(list(enumerate(robo7thPos)))
            
            for tblCnt, roboPos in enum7thPos:
            # for tblCnt, roboPos in enumerate(robo7thPos):

                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print("safepoint done")

                    pocketCycle(cps=cps, yVal=yVals[tblCnt], config=config)
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            keepTool(cps, toolNumber=3, config=config)
            # communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        
        if(config['toolForModels'][f"model{config['UI']['model']}"][2] == 1):
            getTool(cps,  toolNumber=2, config=config)
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            
            enum7thPos = []
            # flip the list for every odd number of elements
            if sum(config['toolForModels'][f"model{config['UI']['model']}"][:3]) % 2:
                enum7thPos = enumerate(robo7thPos)
            else:
                enum7thPos = reversed(list(enumerate(robo7thPos)))
            
            for tblCnt, roboPos in enum7thPos:

                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print("safepoint done")

                    sideCycle(cps=cps, sideSandingPoints=edgeSandingPoints, config=config)
                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    # goToBasicJoints(cps, config)
                    # communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
            
            # if don't have to do edge cycle, then keep the tool
            if config['toolForModels'][f"model{config['UI']['model']}"][3] != 1:
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                keepTool(cps, toolNumber=2, config=config)
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        
        
        if(config['toolForModels'][f"model{config['UI']['model']}"][3] == 1):
            
            # if didn't do sidecycle, then take the tool 
            if config['toolForModels'][f"model{config['UI']['model']}"][2] != 1:
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
                getTool(cps, toolNumber=2, config=config)
                communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            
            enum7thPos = []
            # flip the list for every odd number of elements
            if sum(config['toolForModels'][f"model{config['UI']['model']}"][:4]) % 2:
                enum7thPos = enumerate(robo7thPos)
            else:
                enum7thPos = reversed(list(enumerate(robo7thPos)))
            
            for tblCnt, roboPos in enum7thPos:

                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print("safepoint done")

                    edgeCycle(cps=cps, edgeSandingPoints=edgeSandingPoints, config=config)
                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    # goToBasicJoints(cps, config)
                    # communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            keepTool(cps, toolNumber=2, config=config)
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        
        
        if(config['toolForModels'][f"model{config['UI']['model']}"][4] == 1):
            
            getTool(cps, toolNumber=1, config=config)
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            
            enum7thPos = []
            # flip the list for every odd number of elements
            if sum(config['toolForModels'][f"model{config['UI']['model']}"][:5]) % 2:
                enum7thPos = enumerate(robo7thPos)
            else:
                enum7thPos = reversed(list(enumerate(robo7thPos)))
            
            for tblCnt, roboPos in enum7thPos:

                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
                    if(config['settings']['debug']): print("safepoint done")

                    threeDCycle(cps=cps, innerSandingPoints=innerSandingPoints, config=config)
                    communicate(cpsclient=cps, point=config['point']['safePoint'], config=config, seventh=-1, speed=config['UI']['robotSpeed'])
                    goToBasicJoints(cps, config)
                    # communicate(cpsclient=cps, point=config['point']['safePoint'], seventh=roboPos, config=config, speed=config['UI']['robotSpeed'])
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
            keepTool(cps, toolNumber=1, config=config)
            communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])
        
        communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=-1, config=config, speed=config['UI']['robotSpeed'])

        # break
    ##########################################
    ######At the beginning, do homing!!#######
    ##########################################
    if homingState:
        homingFunction(cps=cps, config=config)

    if startSanding:
        # homingFunction(cps=cps, config=config)
        robo7thPos, outerSandingPoints, innerSandingPoints, edgeSandingPoints, xVals, yVals = scan_table(cps=cps, config=config)
        sand_table(cps, config, robo7thPos, outerSandingPoints, innerSandingPoints, edgeSandingPoints, xVals, yVals)

        communicate(cpsclient=cps, point=config['point']['safePointTool'], seventh=2, config=config, speed=config['UI']['robotSpeed'])
        homingFunction(cps=cps, config=config)
        control_table(cps, tableState="close")


def communicate(cpsclient, config, point=None, seventh= -1, doMeasure = 0, speed=None, stopWhenNan=False):

    # print("Client handler stopped.")
    def remove_leading_nan_entries(measurements):
        """Remove leading entries where height is NaN."""
        for i, entry in enumerate(measurements):
            if not math.isnan(entry['height']):
                return measurements[i:]
        return []  # All values were NaN
    
    def setSpeed(cpsclient, speed):
        if speed is not None:
            # speed = 0.1
            nRet = cpsclient.HRIF_SetOverride(0,0, speed)
            if(config['settings']['debug']): 
                if nRet == 0:
                    print(f'could set speed to {speed * 100}%')
                else:
                    print("couldn't set speed")

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

    def customMoveL(cpsclient, point, tcp, ucs, config, wait=True):
        RawACSpoints = [ 0, 0, 0, 0, 0, 0] # Define the tool coordinate variable 

        nIsSeek = 0 # Define The DI index of the detection 
        nIOBit = 0 # Define The DI status of the detected state www.hansrobot.com 164Interface Description 
        nIOState = 0 # Defining Road Point ID 
        stdCmdID = "0" # Perform road point movement

        nRet = cpsclient.HRIF_MoveL(0,0, point, RawACSpoints, tcp, ucs, config['coords']['roboVelocity'], config['coords']['roboAcceleration'], config['coords']['transitionRadius'], nIsSeek, nIOBit, nIOState, stdCmdID)
        if(config['settings']['debug']): 
            if nRet == 0:
                robotRes = []
                while wait:
                    nret = cpsclient.HRIF_ReadRobotState(0, 0, robotRes)
                    print(f"robotState is: {robotRes}    ")
                    if robotRes[11] == '1':
                        break
                    time.sleep(0.2)
                print(f'\n\ncould move to {point}')
            else:
                print("couldn't move to point")

    def data_collection(cpsclient, config, stopWhenNan=False):
        measurements = []
        instrument = getInstrument()
        result = []
        nRet = cpsclient.HRIF_ReadActPos(0,0, result)
        initPos = [float(result[6]), float(result[7]), float(result[8])]
        keepgoing = True
        nan_count = 0
        # Define the consecutive NaN threshold
        consecutive_nan_threshold = 5
        startStopNan = False

        while keepgoing:
            height = getRawHeight(instrument)
            
            # Read position and compute distance
            cpsclient.HRIF_ReadActPos(0, 0, result)
            pos = [float(result[6]), float(result[7]), float(result[8])]
            if config['settings']['debug']:
                print(f"pos: {pos}, height: {height}                       ", end="\r")
            
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
                    nRet = cpsclient.HRIF_GrpStop(0, 0)

            # Read robot state and check for another stop condition
            cpsclient.HRIF_ReadRobotState(0, 0, result)
            if result[0] == '0':
                keepgoing = False

        if(config['settings']['debug']): print("\n\n")
        # Step 1: Find the first index where 'height' is not NaN
        start_index = 0
        for i, measurement in enumerate(measurements):
            if not math.isnan(measurement.get("height", float('inf'))):
                start_index = i
                break

        # Step 2: Create 'properMeasurements' starting from this index
        properMeasurements = measurements[start_index:]

        # # Step 3: Adjust the 'dist' values by subtracting the 'dist' of the first valid measurement
        # dist_offset = properMeasurements[0].get("dist", 0)
        # for measurement in properMeasurements:
        #     if "dist" in measurement:
        #         measurement["dist"] -= dist_offset
        return properMeasurements
    
    def waitWhileMovingRobot(cpsclient):
        # keep going?
        keepgoing = True
        result = []
        while keepgoing:
            cpsclient.HRIF_ReadRobotState(0,0, result)  
            print(f"Here! waiting now {result}")
            if result[11] == '1':
                keepgoing = False

    def seventhGoToPos(cc_client, position, velocity, config, wait=True):
        pluginRes = []
        robotRes = []
        PosRes = []
        result = []

        Tpos = position

        cnt = 0
        ecnt = 0
        while True:
            nret = cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorReadCurFSM', [], pluginRes)
            time.sleep(0.2)
            nret = cc_client.HRIF_ReadRobotState(0, 0, robotRes)
            time.sleep(0.2)
            nret = cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorReadPosition', [], PosRes)
            time.sleep(0.2)

            # if(config['settings']['debug']):
            #     print(f'pluginRes: {pluginRes}')
            #     print(f'robotRes: {robotRes}')
            #     print(f'posRes: {PosRes}')

            pos = float(PosRes[0])
            if (len(pluginRes) == 0):
                continue
            else:

                if (abs(Tpos - pos) < 0.1):
                    # reached!
                    cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorPowerOff', '1', result)
                    break
                if (pluginRes[0] != '5'):
                    cnt += 1
                    if(cnt >= 3):
                        cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorPowerOff,1')
                        if(config['settings']['debug']): print('[script]====Motor Power Off!  Still  Response Error]                ', end='\r')
                        return
                elif (robotRes[0] == '1'):
                    ecnt += 1
                    if(ecnt >= 3):
                        if(config['settings']['debug']): print('[MT_Kinco]=============errorState[robot error]                ', end='\r')
                        cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorPowerOff,2')
                        return
                elif (pluginRes[0] == '4'):
                    if(config['settings']['debug']): print('[script]===============================errorState[motor disconnect]                ', end='\r')
                    return
                elif (pluginRes[0] == '5' or pluginRes[0] == '6' ):
                    if(config['settings']['debug']): 
                        # print('[MT_Kinco]=============StandState[script jumpout]                ', end='\r')
                        # print('TPos = ')
                        # print(Tpos)
                        print(f'Tpos={Tpos}; CurPos = {pos}                ', end='\r')
                        # print(pos)
                    cc_client.HRIF_HRApp(0, 'MT_Kinco','MotorPowerOn', ['1', str(velocity), str(Tpos)], result)
                    if not wait:
                        break

                time.sleep(0.2)

    measurements = []
    # setting the speed of the client
    setSpeed(cpsclient, speed)
    time.sleep(0.1)

    if seventh != -1:
        seventhGoToPos(cc_client=cpsclient, position=seventh, velocity=config['UI']['seventhAxisSpeed'] / 100 * config['seventhAxis']['maxSpeed'], config=config, wait=bool(1 - doMeasure))

    
    
    if point: 
        print("[com] came here to move to L")
        customMoveL(cpsclient, point=point, tcp = config['coords']['tcpTool'], ucs=config['coords']['ucsTool'], config=config, wait=bool(1 - doMeasure))

    # the list to collect values in
    if doMeasure: 
        measurements = data_collection(cpsclient, config, stopWhenNan)
    else:
        waitWhileMovingRobot(cpsclient)
    try:
        input("proceed to the next one?")
    except EOFError:
        print("No input available, proceeding without user confirmation.")

    # measurements = remove_leading_nan_entries(measurements)
    return measurements

if __name__ == "__main__":
    # Read the YAML configuration file
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    
    handle_client(config)