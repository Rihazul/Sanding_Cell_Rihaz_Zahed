import sys
import os
import time
import yaml
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from model1cycle.mod1pocketside import mod1tool1siderun, model1pocket1side, model1pocket2side, model1pocket3side
from model1cycle.mod1zigzag import mod1zigzag, testmodel1zigzagsmallfunction, testmodel2zigzagsmallfunction, testmodel3zigzagsmallfunction
from model1cycle.mod1bottoms import model1bottomsmall
from model1cycle.mod1tool2edge import mod1tool2outedge
from model1cycle.mod1tool2sideb import mod1tool2sidesrun
from model1cycle.mod1tool3 import mod1tool1
from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate, stop_requested
from modules.CPS import CPSClient
import time

from cycle_data_utils import get_spiral_settings, get_inverse_overlap_step

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def run_side_cycles(count,force,cps):
    """Execute side function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} ===")
        mod1tool1siderun(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next side cycle...")
            time.sleep(3)

def run_zigzag_cycles(count,force,innerSandingOffset,cps,movement="both",tcp_name="tcpReal"):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1zigzag(
            force=force,
            innerSandingOffset=innerSandingOffset,
            cps=cps,
            movement=movement,
            tcp_name=tcp_name,
        )
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)

def run_tool4_section_cycles(side_cycles, force_side, zigzag_cycles, force_zigzag, innerSandingOffset, cps):
    """
    Execute Tool 4 work section-by-section:
    section frame + section zigzag, then move to next section.
    """
    section_handlers = [
        ("SECTION 1", model1pocket1side, testmodel1zigzagsmallfunction),
        ("SECTION 2", model1pocket2side, testmodel2zigzagsmallfunction),
        ("SECTION 3", model1pocket3side, testmodel3zigzagsmallfunction),
    ]

    for section_name, side_fn, zigzag_fn in section_handlers:
        if stop_requested():
            return

        print(f"\n=== TOOL 4 {section_name} ===")

        if side_cycles > 0:
            for i in range(side_cycles):
                if stop_requested():
                    return
                print(f"Frame cycle {i+1}/{side_cycles}")
                side_fn(force=force_side, cps=cps)
                if i < side_cycles - 1:
                    time.sleep(0.5)

        if zigzag_cycles > 0:
            for i in range(zigzag_cycles):
                if stop_requested():
                    return
                print(f"Zigzag cycle {i+1}/{zigzag_cycles}")
                zigzag_fn(
                    force=force_zigzag,
                    innerSandingOffset=innerSandingOffset,
                    cps=cps,
                    movement="zigzag_only",
                    tcp_name="tcptool4plane2",
                    direct_prepoint=(side_cycles > 0),
                )
                if i < zigzag_cycles - 1:
                    time.sleep(0.5)

    # Include bottom Tool-4 routine whenever frame is selected
    # (applies to both "frame only" and "frame + pocket").
    if side_cycles > 0:
        for i in range(side_cycles):
            if stop_requested():
                return
            print(f"Bottom cycle {i+1}/{side_cycles}")
            model1bottomsmall(force=force_side, cps=cps)
            if i < side_cycles - 1:
                time.sleep(0.5)


def run_tool2side_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1tool2sidesrun(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


def run_tool2sideoutedge_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1tool2outedge(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)

# def run_tool2sideoutedge_cycles(count,force,cps):
#     """Execute zigzag function with specified number of cycles"""
#     for i in range(count):
#         print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
#         mod1tool2outedge(force=force,cps=cps)
#         if i < count-1:
#             print("Pausing 3 seconds before next zigzag cycle...")
#             time.sleep(3)

def run_tool1_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1tool1(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config

def check_tool(cps, config, tool_num, ci0, ci1, ci2):
    # Check Conditions
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
    seventh_result = communicate(
        cps=cps,
        config=config,
        seventh=0,
        tcp=config["coords"]["tcptool1plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=0.3,
        wait=True,
        require_seventh_ok=True,
    )
    if seventh_result is None:
        raise RuntimeError(
            f"Failed to move 7th axis to tool station before dropping tool {tool_in_hand}."
        )
    keepTool11(
        cps,
        toolNumber=tool_in_hand,
        config=config,
        goToSafe=False,
        startFromSafe=True,
    )
    return False


def startingRobotToSandmodel1():
 
    #side_cycles   = 0
    #zigzag_cycles = 1
    #tool2_side_cycle=1
    #tool2_sideoutedge=1
    #tool1_cycles=1
    #force_side_cycles=5
    #force_zigzag_cycles = 5
    #force_tool2_side_cycle=5
    #force_tool2_sideoutedge=5
    #force_tool3=5
    
    
    json_config = load_json_config()
    innerSandingOffset = get_inverse_overlap_step(json_config)
    json_config_TableB = json_config['TableB']

    spiral_settings = get_spiral_settings(json_config)
    pocketzigzag_cfg = json_config_TableB.get('pocketzigzag', {})
    vertical_spiral = bool(pocketzigzag_cfg.get('verticalSpiral', False))
    horizontal_spiral = bool(pocketzigzag_cfg.get('horizontalSpiral', False))
    edge_coverage = bool(pocketzigzag_cfg.get('edgeCoverage', False))
    
    side_cycles = int(json_config_TableB['frame']['cycle'])
    print("the side cycle is", side_cycles)
    zigzag_cycles = int(json_config_TableB['pocketzigzag']['cycle'])
    print("the pocketzigzag cycle is", zigzag_cycles)
    tool2_side_cycle = int(json_config_TableB['side']['cycle'])
    tool2_sideoutedge = int(json_config_TableB['edgeOutside']['cycle'])
    tool1_cycles= int(json_config_TableB['3D']['cycle'])
    
    force_side_cycles = int(json_config_TableB['frame']['force'])
    force_zigzag_cycles = int(json_config_TableB['pocketzigzag']['force'])
    force_tool2_side_cycle = int(json_config_TableB['side']['force'])
    force_tool2_sideoutedge = int(json_config_TableB['edgeOutside']['force']) 
    force_tool3=int(json_config_TableB['3D']['force'])
    print(
        "[mainmodel1] Task config -> "
        f"frame(cycle={side_cycles}, force={force_side_cycles}), "
        f"pocket(cycle={zigzag_cycles}, force={force_zigzag_cycles}), "
        f"side(cycle={tool2_side_cycle}, force={force_tool2_side_cycle}), "
        f"edgeOutside(cycle={tool2_sideoutedge}, force={force_tool2_sideoutedge}), "
        f"3D(cycle={tool1_cycles}, force={force_tool3})"
    )

    speeed = float(json_config['robotSpeed'])
    speed_profile = {
        'travel': speeed,
        'tool_change': speeed,
    }

    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])
    keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

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
    
    def read_ci_triplet():
        return read_ci_bit(cps, 0), read_ci_bit(cps, 1), read_ci_bit(cps, 2)

    def decode_tool_in_hand(ci0, ci1, ci2):
        if ci0 is None or ci1 is None or ci2 is None:
            raise RuntimeError("Failed to read one or more CI bits.")
        if ci0 == 1 and ci1 == 0 and ci2 == 0:
            return 3
        if ci0 == 0 and ci1 == 1 and ci2 == 0:
            return 2
        if ci0 == 0 and ci1 == 1 and ci2 == 1:
            return 1
        if ci0 == 0 and ci1 == 0 and ci2 == 1:
            return 4
        if ci0 == 0 and ci1 == 0 and ci2 == 0:
            return None
        raise RuntimeError(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")

    def move_to_safe_point():
        communicate(
            cps=cps,
            point=config['point']['safePoint'],
            tcp=config['coords']['tcpDefault'],
            ucs=config['coords']['ucsDefault'],
            seventh=-1,
            config=config,
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True,
        )

    def move_seventh_to_tool_station():
        seventh_result = communicate(
            cps=cps,
            config=config,
            seventh=0,
            tcp=config['coords']['tcptool1plane1'],
            ucs=config['coords']['ucsTable1'],
            speed=speed_profile['travel'],
            velocity_profile="robot",
            wait=True,
            require_seventh_ok=True,
        )
        if seventh_result is None:
            raise RuntimeError("Failed to move 7th axis to tool station.")

    def ensure_tool_in_hand(tool_num):
        ci0, ci1, ci2 = read_ci_triplet()
        current_tool_before = decode_tool_in_hand(ci0, ci1, ci2)
        has_requested_tool = check_tool(
            cps=cps, config=config, tool_num=tool_num, ci0=ci0, ci1=ci1, ci2=ci2
        )
        if has_requested_tool:
            return False
        switching_from_other_tool = (
            current_tool_before is not None and current_tool_before != tool_num
        )
        picked = getTool11(
            cps=cps,
            toolNumber=tool_num,
            config=config,
            startFromSafe=not switching_from_other_tool,
            exitToSafe=True,
        )
        if picked is False:
            raise RuntimeError(f"Failed to pick tool {tool_num}.")
        return switching_from_other_tool

    def move_to_homing_with_tool():
        """Return to homing/safe position without dropping the mounted tool."""
        move_to_safe_point()
        move_seventh_to_tool_station()
        move_to_safe_point()
    """Main control function"""
    try:
        has_pocket_work = zigzag_cycles > 0
        has_tool3_edge_pass = has_pocket_work
        has_tool4_pocket_pass = (side_cycles > 0 or has_pocket_work)
        has_any_task = (
            has_tool3_edge_pass
            or has_tool4_pocket_pass
            or tool2_side_cycle > 0
            or tool2_sideoutedge > 0
            or tool1_cycles > 0
        )

        if has_any_task:
            move_to_safe_point()
        work_executed = False

        if has_pocket_work and not edge_coverage:
            print("Pocket zigzag is selected: forcing edge-coverage pass on Tool 3.")

        # Tool 3 batch: pocket edge-coverage only
        if has_tool3_edge_pass:
            ensure_tool_in_hand(3)
            if stop_requested():
                return
            run_zigzag_cycles(
                zigzag_cycles,
                force_zigzag_cycles,
                innerSandingOffset,
                cps,
                movement="edge_only",
                tcp_name="tcpReal",
            )
            if stop_requested():
                return
            work_executed = True
            
            print("\nTool 3 edge-coverage batch completed successfully!")
        
        if has_tool4_pocket_pass:
            ensure_tool_in_hand(4)
            if stop_requested():
                return

            # Tool 4 batch: section-by-section (frame + pocket zigzag per section)
            run_tool4_section_cycles(
                side_cycles=side_cycles,
                force_side=force_side_cycles,
                zigzag_cycles=zigzag_cycles,
                force_zigzag=force_zigzag_cycles,
                innerSandingOffset=innerSandingOffset,
                cps=cps,
            )
            if stop_requested():
                return
            work_executed = True
            
        
        if tool2_side_cycle > 0 or  tool2_sideoutedge > 0:
            ensure_tool_in_hand(2)
            if stop_requested():
                return

            #Tool 2 Side Cycles
            run_tool2side_cycles(tool2_side_cycle,force_tool2_side_cycle,cps)

            #Tool 2 Side Out Edge Cycles
            run_tool2sideoutedge_cycles(tool2_sideoutedge,force_tool2_sideoutedge,cps)
            if stop_requested():
                return
            work_executed = True

        if tool1_cycles>0:
            ensure_tool_in_hand(1)
            if stop_requested():
                return
            #Tool 3Cycle
            run_tool1_cycles(tool1_cycles,force_tool3,cps)
            if stop_requested():
                return
            work_executed = True

        if work_executed:
            if keep_tool_after_task:
                move_to_homing_with_tool()
                print(
                    "Task completed: moved to homing position while keeping current tool mounted."
                )
            else:
                ci0_end, ci1_end, ci2_end = read_ci_triplet()
                tool_in_hand = decode_tool_in_hand(ci0_end, ci1_end, ci2_end)
                if tool_in_hand is not None:
                    move_to_safe_point()
                    move_seventh_to_tool_station()
                    keepTool11(cps, toolNumber=tool_in_hand, config=config)
                else:
                    print("Task completed: no mounted tool to drop.")


    except Exception as e:
        print(f"\nExecution error: {str(e)}")
    finally:
        print("\nSequence terminated")

if __name__ == "__main__":
    startingRobotToSandmodel1() 
