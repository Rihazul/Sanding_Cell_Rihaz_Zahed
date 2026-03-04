import sys
import os
import time
import yaml
import traceback
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from smallTable.model4frame import smalldoor1side,smalldoor2side,smalldoor3side,smalldoor4side
from smallTable.model4zigzag import smalldoor1zizag,smalldoor2zizag,smalldoor3zizag,smalldoor4zizag
from smallTable.pocketplane1final import smalldoor1pocket,smalldoor2pocket,smalldoor3pocket,smalldoor4pocket
from smallTable.frame1tool2sidefinal import door1frametool2side,door2frametool2side,door3frametool2side,door4frametool2side
from smallTable.frame1tool2edgefinal import door1frametool2sideedge,door2frametool2sideedge,door3frametool2sideedge,door4frametool2sideedge
from smallTable.model4tool3 import smalldoor1tool3,smalldoor2tool3,smalldoor3tool3,smalldoor4tool3

from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate,waitForBlending,toolValve1,msg_to_frontend,keepToolupdated,getToolUpdated
from modules.CPS import CPSClient
import time

from cycle_data_utils import any_cycles, doors_with_cycles, get_spiral_settings, get_tableA_task_by_door

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config

# def run_side_cycles(count,force):
#     """Execute side function with specified number of cycles"""
#     for i in range(count):
#         print(f"\n=== SIDE CYCLE {i+1}/{count} ===")
#         testmodel4sidesmallfunction(force=force)
#         if i < count-1:
#             print("Pausing 3 seconds before next side cycle...")
#             time.sleep(3)

def run_side_cycles(count, force, door_num,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: smalldoor1side,
        2: smalldoor2side,
        3: smalldoor3side,
        4: smalldoor4side
    }
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_zigzag_cycles(
    count,
    force,
    door_num,
    z,
    cps,
    *,
    orientation="vertical",
    movement="zigzag",
    spiral_settings=None,
):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: smalldoor1zizag,
        2: smalldoor2zizag,
        3: smalldoor3zizag,
        4: smalldoor4zizag
    }
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(
            force=force,
            z=z,
            cps=cps,
            orientation=orientation,
            movement=movement,
            spiral_settings=spiral_settings,
        )
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_pocket_cycles(count, force, door_num, z,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: smalldoor1pocket,
        2: smalldoor2pocket,
        3: smalldoor3pocket,
        4: smalldoor4pocket
    }
    
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,z=z,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_tool2side_cycles(count, force, door_num,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: door1frametool2side,
        2: door2frametool2side,
        3: door3frametool2side,
        4: door4frametool2side
    }
    
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_tool2side_edgecycles(count, force, door_num,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: door1frametool2sideedge,
        2: door2frametool2sideedge,
        3: door3frametool2sideedge,
        4: door4frametool2sideedge
    }
    
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_pocket_cycles(count,door_num, z,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: smalldoor1tool3,
        # 2: smalldoor2tool3,
        # 3: smalldoor3tool3,
        # 4: smalldoor4tool3
    }
    
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(z=z,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)

def run_tool3_cycles(count,door_num, z,cps):
    """Execute door function based on number"""
    if count <= 0:  # Skip if count is 0 or negative
        return
    door_funcs = {
        1: smalldoor1tool3,
        2: smalldoor2tool3,
        3: smalldoor3tool3,
        4: smalldoor4tool3
    }
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(z=z,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds...")
            time.sleep(3)
  
# def keepToolupdated(cps, toolNumber, config): #Tool Postion a rekhe dibe
#         # if (not config['settings']['useTool']):
#         #     return
#         msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Keeping Started...")
#         # come to safe tool picking position
#         communicate(cps=cps, point=config['point']['safePointTool'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=False)
#         # go to tool's home
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}home'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=False)
#         # touch the tool (slowly)
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.25, wait=False)
#         # drop the tool
#         waitForBlending(cps=cps, config=config)
#         toolValve1(cps, valveState="drop", config=config)
#         # come back to tool's home
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}home'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.25, wait=True)
#         # if don't need to pick another tool just after dropping this one, then come to safe picking position
#         # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Kept Successfully")
#         # if goToSafe: 
#         #     communicate(cps=cps, point=config['point']['safePointTool'],tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)

# def getToolUpdated(cps, toolNumber, config, startFromSafe=True): #Tool postion dile nibe
#         """_summary_

#         Args:
#             cps (CPSClient): cobot client
#             toolNumber (int): The position from which to pick tool (out of 1-4)
#             config (config): configuration info
#             startFromSafe (bool, optional): If should go to the safe tool picking position or not. Make False if doing tool drop and pick one after another. Defaults to True.
#         """
#         # if (not config['settings']['useTool']):
#         #     return
#         # msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Collection Started...")
#         # # if didn't drop another tool just before picking this one, then come to safe picking position
#         # if startFromSafe: 
#         #     communicate(cps=cps, point=config['point']['safePointTool'],tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=False)

#         # go to that tool's home position (right above the tool)
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}home'],  tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=False)
#         # drop (for safety, to open the valve)
#         waitForBlending(cps=cps, config=config)
#         toolValve1(cps, valveState="drop", config=config)
#         # touch the tool (slowly)
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.25, wait=False)
#         # pick the tool
#         waitForBlending(cps=cps, config=config)
#         toolValve1(cps, valveState="pick", config=config)
#         # come back to tool's home position
#         communicate(cps=cps, point=config['point'][f'tool{toolNumber}home'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.25, wait=True)
#         # come back to safe tool picking position
#         msg_to_frontend(api_url=config['server']['frontEnd_messaging_url'], message=f"Tool {toolNumber} Collection Successful!")
#         communicate(cps=cps, point=config['point']['safePointTool'],tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)

def is_door_available(doors):
    if any(doors):
        return True
    else:
        return False

def check_tool(cps, config, tool_num, ci0, ci1, ci2):
    # Check Conditions
    if ci0 is None or ci1 is None or ci2 is None:
        print("Failed to read one or more CI bits.")
        return
    else:
        if ci0 == 0 and ci1 == 0 and ci2 == 0:
            print("No tool in hand")
        elif ci0 == 1 and ci1 == 0 and ci2 == 0:
            # if tool_num == 3:
            #     return
            print("Tool 3 detected → executing keepTool11()")
            keepTool11(cps, toolNumber=3, config=config)
        elif ci0 == 0 and ci1 == 1 and ci2 == 0:
            # if tool_num == 2:
            #     return
            print("Tool 2 detected → executing keepTool11()")
            keepTool11(cps, toolNumber=2, config=config)
        elif ci0 == 0 and ci1 == 1 and ci2 == 1:
            # if tool_num == 1:
            #     return
            print("Tool 1 detected → executing keepTool11()")
            keepTool11(cps, toolNumber=1, config=config)
        else:
            print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")

def sandingModelDETableA():
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])
    
    json_config = load_json_config()
    json_config_TableA = json_config['TableA']

    spiral_settings = get_spiral_settings(json_config)

    frame_by_door = get_tableA_task_by_door(json_config_TableA, 'frame')
    zigzag_by_door = get_tableA_task_by_door(json_config_TableA, 'pocketzigzag')
    pocket_by_door = get_tableA_task_by_door(json_config_TableA, 'pocketsquare')
    tool2side_by_door = get_tableA_task_by_door(json_config_TableA, 'side')
    tool2edge_by_door = get_tableA_task_by_door(json_config_TableA, 'edgeOutside')
    tool3_by_door = get_tableA_task_by_door(json_config_TableA, '3D')

    side_cycles_doors = doors_with_cycles(frame_by_door)
    zig_zag_cycle_doors = doors_with_cycles(zigzag_by_door)
    pocket_cycle_doors = doors_with_cycles(pocket_by_door)
    tool2side_cycle_doors = doors_with_cycles(tool2side_by_door)
    tool2sideedge_cycle_doors = doors_with_cycles(tool2edge_by_door)
    tl3sideedge_door = doors_with_cycles(tool3_by_door)
    
    z, z1, z2 = -4, 0, -3

    #Speed
    speeed = float(json_config['robotSpeed'])

    #Tool1 Side Cycle
    # side_cycles   = 1
    # side_cycles1  = 1 # 0 to 10
    # side_cycles2   = 0
    # side_cycles3   = 0
    # side_cycles4   = 0
    # # zigzag_cycles = 1
    # door_number1=1  # 1 to 4 
    # door_number2=2
    # door_number3=3
    # door_number4=4
    # tool2_side_cycle=1
    # tool2_sideoutedge=1
    # force_side_cycles1=5  # 1 to 30
    # force_side_cycles2=2
    # force_side_cycles3=2
    # force_side_cycles4=2
    # force_zigzag_cycles = 1
    # force_tool2_side_cycle=2
    # force_tool2_sideoutedge=2

    #Zigzag Cycle Tool1
    # zig_cycle1= 1
    # zig_cycle2= 0
    # zig_cycle3= 0
    # zig_cycle4= 0
    # force_zigzag1= 20
    # force_zigzag2= 20
    # force_zigzag3= 20
    # force_zigzag4= 20
    # zig_door1=1
    # zig_door2=2
    # zig_door3=3
    # zig_door4=4
    # z1=0
    # z2=-3
    # z= -6.5

    #PocketTool1
    # pocket_cycle1= 0
    # pocket_cycle2= 0
    # pocket_cycle3= 0
    # pocket_cycle4= 0
    # force_pocket1 = 2
    # force_pocket2 = 2
    # force_pocket3 = 2
    # force_pocket4 = 2
    # pocket_door1= 1
    # pocket_door2= 2
    # pocket_door3= 3
    # pocket_door4= 4

    #Tool2Side Cycle
    # tl2side_cycle1= 1
    # tl2side_cycle2= 0
    # tl2side_cycle3= 0
    # tl2side_cycle4= 0
    # tl2side_force1= 2
    # tl2side_force2= 2
    # tl2side_force3= 2
    # tl2side_force4= 2
    # tl2side_door1= 1
    # tl2side_door2= 2
    # tl2side_door3= 3
    # tl2side_door4= 4

    #Tool2Side Cycle Edge
    # tl2sideedge_cycle1= 1
    # tl2sideedge_cycle2= 0
    # tl2sideedge_cycle3= 0
    # tl2sideedge_cycle4= 0
    # tl2sideedge_force1= 2
    # tl2sideedge_force2= 2
    # tl2sideedge_force3= 2
    # tl2sideedge_force4= 2
    # tl2sideedge_door1= 1
    # tl2sideedge_door2= 2
    # tl2sideedge_door3= 3
    # tl2sideedge_door4= 4

    #Tool3Cycle
    # tl3sideedge_cycle1= 1
    # tl3sideedge_cycle2= 0
    # tl3sideedge_cycle3= 0
    # tl3sideedge_cycle4= 0
    # tl3sideedge_door1= 1
    # tl3sideedge_door2= 2
    # tl3sideedge_door3= 3
    # tl3sideedge_door4= 4

    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    # --- Adding tool detection code here ---
    if ret != 0:
        print(f"Failed to connect, error code: {ret}")
        exit()

    def read_ci_bit(cps, bit_index):
        """Reads CI bit and returns its value (0 or 1)."""
        result = []
        nRet = cps.HRIF_ReadBoxCI(0, bit_index, result)
        if nRet != 0:
            print(f"Error reading CI[{bit_index}], error code: {nRet}")
            return None
        return int(result[0])

    # Read CI values
    ci0 = read_ci_bit(cps, 0)
    ci1 = read_ci_bit(cps, 1)
    ci2 = read_ci_bit(cps, 2)

    # # Check Conditions
    # if ci0 is None or ci1 is None or ci2 is None:
    #     print("Failed to read one or more CI bits.")
    # else:
    #     if ci0 == 0 and ci1 == 0 and ci2 == 0:
    #         print("No tool in hand")
    #     elif ci0 == 1 and ci1 == 0 and ci2 == 0:
    #         print("Tool 3 detected → executing keepTool11()")
    #         keepTool11(cps, toolNumber=3, config=config)
    #     elif ci0 == 0 and ci1 == 1 and ci2 == 0:
    #         print("Tool 2 detected → executing keepTool11()")
    #         keepTool11(cps, toolNumber=2, config=config)
    #     elif ci0 == 0 and ci1 == 1 and ci2 == 1:
    #         print("Tool 1 detected → executing keepTool11()")
    #         keepTool11(cps, toolNumber=1, config=config)
    #     else:
    #         print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")

    """Main control function"""
    try:
        if (is_door_available(side_cycles_doors) or is_door_available(zig_zag_cycle_doors) or is_door_available(pocket_cycle_doors)) and (any_cycles(frame_by_door) or any_cycles(zigzag_by_door) or any_cycles(pocket_by_door)):
            check_tool(cps=cps,config=config,tool_num=3,ci0=ci0,ci1=ci1,ci2=ci2)

            #Intitial Position
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #Pick Tool 3
            getTool11(cps, toolNumber=3, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            
            #Run side cycles
            for door_number in side_cycles_doors:
                cfg = frame_by_door.get(int(door_number), {})
                run_side_cycles(int(cfg.get('cycle', 0)), int(cfg.get('force', 0)), int(door_number), cps)
            
            # run_side_cycles(side_cycles1,force_side_cycles1,door_number1,cps)
            #run_side_cycles(side_cycles2,force_side_cycles2,door_number2)
            #run_side_cycles(side_cycles3,force_side_cycles3,door_number3)
            #run_side_cycles(side_cycles4,force_side_cycles4,door_number4)
            
            #Run Zigzag cycles
            for door_number in zig_zag_cycle_doors:
                cfg = zigzag_by_door.get(int(door_number), {})
                # Map UI toggles/config to orientation/movement
                orientation = str(cfg.get("orientation") or "vertical").lower()
                edge_flag = cfg.get("edge")
                if edge_flag is None:
                    edge_flag = cfg.get("edgeCoverage")
                movement = "rect" if edge_flag else "zigzag"
                run_zigzag_cycles(
                    int(cfg.get("cycle", 0)),
                    int(cfg.get("force", 0)),
                    int(door_number),
                    z,
                    cps,
                    orientation=orientation,
                    movement=movement,
                    spiral_settings=spiral_settings,
                )
            
            #run_zigzag_cycles(zig_cycle2, force_zigzag2, zig_door2, z)
            #run_zigzag_cycles(zig_cycle3, force_zigzag3, zig_door3, z)
            #run_zigzag_cycles(zig_cycle4, force_zigzag4, zig_door4, z)

            #Run Pocket cycles
            for door_number in pocket_cycle_doors:
                cfg = pocket_by_door.get(int(door_number), {})
                run_pocket_cycles(int(cfg.get('cycle', 0)), int(cfg.get('force', 0)), int(door_number), z, cps)
            # run_pocket_cycles(pocket_cycle2, force_pocket2, pocket_door2, z)
            # run_pocket_cycles(pocket_cycle3, force_pocket3, pocket_door3, z)
            # run_pocket_cycles(pocket_cycle4, force_pocket4, pocket_door4, z)

            #Keep Tool 3
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps ,config=config, seventh=0, tcp=config['coords']['tcptool1plane1'], ucs=config['coords']['ucsTable1'], speed=0.3, wait=True)
            #keepTool11(cps, toolNumber=3, config=config)
            if any_cycles(tool2edge_by_door) or any_cycles(tool2side_by_door) or any_cycles(tool3_by_door):
                keepToolupdated(cps, toolNumber=3, config=config)
            else:
                keepTool11(cps, toolNumber=3, config=config)
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)

            print("\nAll operations completed for Tool1 successfully!")
        
        if (is_door_available(tool2side_cycle_doors) or is_door_available(tool2sideedge_cycle_doors)) and (any_cycles(tool2edge_by_door) or any_cycles(tool2side_by_door)):
            check_tool(cps=cps,config=config,tool_num=2,ci0=ci0,ci1=ci1,ci2=ci2)
            #Pick Tool 2
            #getTool11(cps, toolNumber=2, config=config)
            if any_cycles(frame_by_door) or any_cycles(zigzag_by_door) or any_cycles(pocket_by_door):
                print("At least one cycle > 0 → running getToolUpdated()")
                getToolUpdated(cps, toolNumber=2, config=config)
            else:
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                getTool11(cps, toolNumber=2, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            # time.sleep(2)
            
            #Tool2 Outside cycle
            for door_number in tool2side_cycle_doors:
                cfg = tool2side_by_door.get(int(door_number), {})
                print('run_tool2side_cycles', int(cfg.get('cycle', 0)), int(cfg.get('force', 0)), int(door_number), cps)
                run_tool2side_cycles(int(cfg.get('cycle', 0)), int(cfg.get('force', 0)), int(door_number), cps)
            
            #Tool2 Outside Edge Cycle
            for door_number in tool2sideedge_cycle_doors:
                cfg = tool2edge_by_door.get(int(door_number), {})
                run_tool2side_edgecycles(int(cfg.get('cycle', 0)), int(cfg.get('force', 0)), int(door_number), cps)
            
            #Drop Tool 2
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
            #keepTool11(cps, toolNumber=2, config=config)
            #keepToolupdated(cps, toolNumber=2, config=config)
            if any_cycles(tool3_by_door):
                keepToolupdated(cps, toolNumber=2, config=config)
            else:
                keepTool11(cps, toolNumber=2, config=config)
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            
            #communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.2, wait=True)

        if is_door_available(tl3sideedge_door) and any_cycles(tool3_by_door):
            check_tool(cps=cps,config=config,tool_num=1,ci0=ci0,ci1=ci1,ci2=ci2)
            #Pick Tool 3
            #print("taking tool 1")
            #getTool11(cps, toolNumber=1, config=config)
            if any_cycles(tool2edge_by_door) or any_cycles(tool2side_by_door) or any_cycles(frame_by_door) or any_cycles(zigzag_by_door) or any_cycles(pocket_by_door):
                print("At least one cycle > 0 → running getToolUpdated()")
                getToolUpdated(cps, toolNumber=1, config=config)
            else:
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                getTool11(cps, toolNumber=1, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
           
            # # #Tool 3 3D Cycle
            # # run_tool3_cycles(tool3_3dcycle,force_tool3_3d)  # <-- Fixed indentation
            # for door_number in tl3sideedge_door:
            #     run_tool3_cycles(tl3sideedge_cycle, int(door_number), z2)
            for door_number in tl3sideedge_door:
                cfg = tool3_by_door.get(int(door_number), {})
                run_tool3_cycles(int(cfg.get('cycle', 0)), int(door_number), z2, cps)
            
            #Drop Tool 2
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
            keepTool11(cps, toolNumber=1, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
           
    except Exception as e:
        print(f"\nExecution error: {str(e)}")
        traceback.print_exc()
    finally:
        print("\nSequence terminated")

if __name__ == "__main__":
    sandingModelDETableA() 
