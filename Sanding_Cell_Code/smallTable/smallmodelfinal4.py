import sys
import os
import time
import yaml
import traceback
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from smallTable.model4frame import smalldoor1side,smalldoor2side,smalldoor3side,smalldoor4side
from smallTable.zigzagplane1final import smalldoor1zizag,smalldoor2zizag,smalldoor3zizag,smalldoor4zizag
from smallTable.edge_coverage_tool3 import run_tool3_pocket_edge_cycles
from smallTable.pocketplane1final import smalldoor1pocket,smalldoor2pocket,smalldoor3pocket,smalldoor4pocket
from smallTable.frame1tool2sidefinal import door1frametool2side,door2frametool2side,door3frametool2side,door4frametool2side
from smallTable.frame1tool2edgefinal import door1frametool2sideedge,door2frametool2sideedge,door3frametool2sideedge,door4frametool2sideedge
from smallTable.tool1_modelD_internal import smalldoor1tool3,smalldoor2tool3,smalldoor3tool3,smalldoor4tool3

from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate,stop_requested
from modules.CPS import CPSClient
import time

from cycle_data_utils import any_cycles, doors_with_cycles, get_spiral_settings, get_tableA_task_by_door

INTER_PASS_DELAY_SECONDS = 0.0

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
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)

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
        if stop_requested():
            raise RuntimeError("[Spiral] Stop requested.")
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(
            force=force,
            z=z,
            cps=cps,
            orientation=orientation,
            movement=movement,
            spiral_settings=spiral_settings,
        )
        if i < count - 1:
            if stop_requested():
                raise RuntimeError("[Spiral] Stop requested.")
            if INTER_PASS_DELAY_SECONDS > 0:
                print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
                time.sleep(INTER_PASS_DELAY_SECONDS)

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
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)

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
        if stop_requested():
            raise RuntimeError("[Tool 2 Side] Stop requested.")
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,cps=cps)
        if stop_requested():
            raise RuntimeError("[Tool 2 Side] Stop requested.")
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)

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
        if stop_requested():
            raise RuntimeError("[Tool 2 Edge] Stop requested.")
        print(f"\n=== SIDE CYCLE {i+1}/{count} (Door {door_num}) ===")
        door_func(force=force,cps=cps)
        if stop_requested():
            raise RuntimeError("[Tool 2 Edge] Stop requested.")
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)

def run_tool3_cycles(count, door_num, z, cps, force):
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
        door_func(z=z, cps=cps, force=force)
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)
  
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

def unique_sorted_doors(*door_groups):
    """Return unique valid door ids as sorted ints."""
    doors = set()
    for group in door_groups:
        for door in group:
            try:
                doors.add(int(door))
            except (TypeError, ValueError):
                continue
    return sorted(doors)

def check_tool(cps, config, tool_num, ci0, ci1, ci2):
    if ci0 is None or ci1 is None or ci2 is None:
        print("Failed to read one or more CI bits.")
        return False

    tool_in_hand = None
    if ci0 == 1 and ci1 == 0 and ci2 == 0:
        tool_in_hand = 3
    elif ci0 == 0 and ci1 == 1 and ci2 == 0:
        tool_in_hand = 2
    elif ci0 == 0 and ci1 == 1 and ci2 == 1:
        tool_in_hand = 1
    elif ci0 == 0 and ci1 == 0 and ci2 == 1:
        tool_in_hand = 4
    elif ci0 == 0 and ci1 == 0 and ci2 == 0:
        tool_in_hand = None
    else:
        print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")
        return False

    if tool_in_hand is None:
        print("No tool in hand")
        return False

    if tool_in_hand == tool_num:
        print(f"Tool {tool_in_hand} already in hand; skipping drop/pick.")
        return True

    print(f"Tool {tool_in_hand} detected; dropping before picking tool {tool_num}.")
    communicate(
        cps=cps,
        config=config,
        seventh=0,
        tcp=config["coords"]["tcptool1plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=0.3,
        wait=True,
    )
    keepTool11(cps, toolNumber=tool_in_hand, config=config, goToSafe=False, startFromSafe=True)
    return False

def sandingModelDETableA():
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])
    
    json_config = load_json_config()
    json_config_TableA = json_config['TableA']

    spiral_settings = get_spiral_settings(json_config)

    frame_by_door = get_tableA_task_by_door(json_config_TableA, 'frame')
    zigzag_by_door = get_tableA_task_by_door(json_config_TableA, 'pocketzigzag')
    tool2side_by_door = get_tableA_task_by_door(json_config_TableA, 'side')
    tool2edge_by_door = get_tableA_task_by_door(json_config_TableA, 'edgeOutside')
    tool3_by_door = get_tableA_task_by_door(json_config_TableA, '3D')

    side_cycles_doors = doors_with_cycles(frame_by_door)
    zig_zag_cycle_doors = doors_with_cycles(zigzag_by_door)
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

    def read_ci_triplet(cps):
        ci0_local = read_ci_bit(cps, 0)
        ci1_local = read_ci_bit(cps, 1)
        ci2_local = read_ci_bit(cps, 2)
        return ci0_local, ci1_local, ci2_local

    def decode_tool_in_hand(ci0_local, ci1_local, ci2_local):
        if ci0_local is None or ci1_local is None or ci2_local is None:
            raise RuntimeError("Failed to read one or more CI bits.")
        if ci0_local == 1 and ci1_local == 0 and ci2_local == 0:
            return 3
        if ci0_local == 0 and ci1_local == 1 and ci2_local == 0:
            return 2
        if ci0_local == 0 and ci1_local == 1 and ci2_local == 1:
            return 1
        if ci0_local == 0 and ci1_local == 0 and ci2_local == 1:
            return 4
        if ci0_local == 0 and ci1_local == 0 and ci2_local == 0:
            return None
        raise RuntimeError(
            f"Unrecognized CI combination: CI0={ci0_local}, CI1={ci1_local}, CI2={ci2_local}"
        )

    def move_to_safe_point():
        communicate(
            cps=cps,
            point=config["point"]["safePoint"],
            tcp=config["coords"]["tcpDefault"],
            ucs=config["coords"]["ucsDefault"],
            seventh=-1,
            config=config,
            speed=speeed,
            wait=True,
        )

    def move_seventh_to_tool_station():
        communicate(
            cps=cps,
            config=config,
            seventh=0,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.7,
            wait=True,
        )

    def move_to_homing_with_tool():
        move_to_safe_point()
        move_seventh_to_tool_station()
        move_to_safe_point()

    def ensure_tool_in_hand(tool_num):
        ci0_local, ci1_local, ci2_local = read_ci_triplet(cps)
        current_tool_before = decode_tool_in_hand(ci0_local, ci1_local, ci2_local)
        has_requested_tool = check_tool(
            cps=cps,
            config=config,
            tool_num=tool_num,
            ci0=ci0_local,
            ci1=ci1_local,
            ci2=ci2_local,
        )
        if not has_requested_tool:
            switching_from_other_tool = (
                current_tool_before is not None and current_tool_before != tool_num
            )
            picked = getTool11(
                cps,
                toolNumber=tool_num,
                config=config,
                startFromSafe=not switching_from_other_tool,
                exitToSafe=True,
            )
            if picked is False:
                raise RuntimeError(f"Failed to pick tool {tool_num}.")

    """Main control function"""
    try:
        has_any_task = (
            any_cycles(frame_by_door)
            or any_cycles(zigzag_by_door)
            or any_cycles(tool2edge_by_door)
            or any_cycles(tool2side_by_door)
            or any_cycles(tool3_by_door)
        )
        if has_any_task:
            move_to_safe_point()

        work_executed = False
        keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

        tool3_edge_doors = unique_sorted_doors(
            [
                door_number
                for door_number in zig_zag_cycle_doors
                if int(zigzag_by_door.get(int(door_number), {}).get("cycle", 0)) > 0
            ]
        )
        tool4_doors = unique_sorted_doors(side_cycles_doors, zig_zag_cycle_doors)
        tool2_doors = unique_sorted_doors(tool2side_cycle_doors, tool2sideedge_cycle_doors)
        tool1_doors = unique_sorted_doors(tl3sideedge_door)

        has_tool3_batch = len(tool3_edge_doors) > 0
        has_tool4_batch = any_cycles(frame_by_door) or any_cycles(zigzag_by_door)
        has_tool2_batch = any_cycles(tool2edge_by_door) or any_cycles(tool2side_by_door)
        has_tool1_batch = any_cycles(tool3_by_door)

        # Tool 3 edge-coverage pre-pass tied to Pocket ZigZag
        if has_tool3_batch and is_door_available(tool3_edge_doors):
            ensure_tool_in_hand(3)
            for door_number in tool3_edge_doors:
                zigzag_cfg = zigzag_by_door.get(int(door_number), {})
                zigzag_cycle = int(zigzag_cfg.get("cycle", 0))
                if zigzag_cycle <= 0:
                    continue
                pocket_orientation = str(zigzag_cfg.get("orientation") or "vertical").lower()
                edge_force = int(zigzag_cfg.get("force") or 0)
                if edge_force <= 0:
                    raise RuntimeError(
                        f"Tool 3 edge force is not configured for door {door_number}. "
                        "Set Pocket ZigZag force to a value > 0."
                    )
                run_tool3_pocket_edge_cycles(
                    zigzag_cycle,
                    edge_force,
                    int(door_number),
                    z,
                    cps,
                    orientation=pocket_orientation,
                    spiral_settings=spiral_settings,
                )
                work_executed = True

        # Tool 4 batch: frame + pocket zigzag
        if has_tool4_batch and is_door_available(tool4_doors):
            ensure_tool_in_hand(4)
            for door_number in tool4_doors:
                frame_cfg = frame_by_door.get(door_number, {})
                frame_cycle = int(frame_cfg.get("cycle", 0))

                zigzag_cfg = zigzag_by_door.get(door_number, {})
                zigzag_cycle = int(zigzag_cfg.get("cycle", 0))

                if frame_cycle <= 0 and zigzag_cycle <= 0:
                    continue

                if frame_cycle > 0:
                    run_side_cycles(frame_cycle, int(frame_cfg.get("force", 0)), door_number, cps)
                    work_executed = True

                if zigzag_cycle > 0:
                    orientation = str(zigzag_cfg.get("orientation") or "vertical").lower()
                    run_zigzag_cycles(
                        zigzag_cycle,
                        int(zigzag_cfg.get("force", 0)),
                        door_number,
                        z,
                        cps,
                        orientation=orientation,
                        movement="zigzag",
                        spiral_settings=spiral_settings,
                    )
                    work_executed = True

        # Tool 2 batch
        if has_tool2_batch and is_door_available(tool2_doors):
            ensure_tool_in_hand(2)
            # Stabilize the arm orientation before entering the first Tool 2 door path.
            move_to_safe_point()
            previous_tool2_door = None
            for door_number in tool2_doors:
                side_cfg = tool2side_by_door.get(door_number, {})
                side_cycle = int(side_cfg.get("cycle", 0))

                edge_cfg = tool2edge_by_door.get(door_number, {})
                edge_cycle = int(edge_cfg.get("cycle", 0))

                if side_cycle <= 0 and edge_cycle <= 0:
                    continue

                if previous_tool2_door is not None:
                    print(
                        f"\n--- Tool 2 / Door-to-Door Safe Transition / "
                        f"Door {previous_tool2_door} to Door {door_number} ---"
                    )
                    move_to_safe_point()

                if side_cycle > 0:
                    run_tool2side_cycles(side_cycle, int(side_cfg.get("force", 0)), door_number, cps)
                    work_executed = True

                if side_cycle > 0 and edge_cycle > 0:
                    print(f"\n--- Tool 2 / Side-to-Edge Safe Transition / Door {door_number} ---")
                    move_to_safe_point()

                if edge_cycle > 0:
                    run_tool2side_edgecycles(edge_cycle, int(edge_cfg.get("force", 0)), door_number, cps)
                    work_executed = True

                previous_tool2_door = door_number

        # Tool 1 batch (Model D tool1 internal path)
        if has_tool1_batch and is_door_available(tool1_doors):
            ensure_tool_in_hand(1)
            for door_number in tool1_doors:
                cfg = tool3_by_door.get(int(door_number), {})
                cycle_count = int(cfg.get("cycle", 0))
                if cycle_count <= 0:
                    continue
                run_tool3_cycles(
                    cycle_count,
                    int(door_number),
                    z2,
                    cps,
                    int(cfg.get("force", 0)),
                )
                work_executed = True

        if work_executed:
            if keep_tool_after_task:
                move_to_homing_with_tool()
                print("Task completed: moved to homing position while keeping current tool mounted.")
            else:
                ci0_end, ci1_end, ci2_end = read_ci_triplet(cps)
                tool_in_hand = decode_tool_in_hand(ci0_end, ci1_end, ci2_end)
                if tool_in_hand is not None:
                    move_to_safe_point()
                    move_seventh_to_tool_station()
                    keepTool11(cps, toolNumber=tool_in_hand, config=config)
                else:
                    print("Task completed: no mounted tool to drop.")
        elif has_any_task:
            print("No valid door cycles found to execute.")

    except Exception as e:
        print(f"\nExecution error: {str(e)}")
        traceback.print_exc()
        if stop_requested():
            raise
    finally:
        print("\nSequence terminated")

if __name__ == "__main__":
    sandingModelDETableA() 
