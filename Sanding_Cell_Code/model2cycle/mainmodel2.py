import sys
import os
import time
import yaml
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))




from model2cycle.testmodel2sidesmall import testmodel2siderun
from model2cycle.testmodel2zigzags import model2zigzagrun
from model2cycle.testmodel2tool2sides import testmodel2tool2sidesrun
from model2cycle.testmodeltool2edgesm import testmodel2tool2edgerun
from model2cycle.testmodel2thirdtool import model2thirdtoolrun

from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate, stop_requested
from modules.CPS import CPSClient
import time

from cycle_data_utils import get_spiral_settings, get_inverse_overlap_step

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

def run_side_cycles(count,force,cps):
    """Execute side function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== SIDE CYCLE {i+1}/{count} ===")
        testmodel2siderun(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next side cycle...")
            time.sleep(3)

def run_zigzag_cycles(
    count,
    force,
    innerSandingOffset,
    cps,
    movement="both",
    tcp_name="tcpReal",
    direct_prepoint=False,
):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        model2zigzagrun(
            force=force,
            innerSandingOffset=innerSandingOffset,
            cps=cps,
            movement=movement,
            tcp_name=tcp_name,
            direct_prepoint=direct_prepoint,
        )
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


def run_tool2side_cycles(count,force,cps,section=None,return_home=True):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        testmodel2tool2sidesrun(force=force,cps=cps,section=section,return_home=return_home)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


def run_tool2sideoutedge_cycles(count,force,cps,section=None,return_home=True):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        testmodel2tool2edgerun(force=force,cps=cps,section=section,return_home=return_home)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


def run_tool2_combined_cycles(side_count, side_force, edge_count, edge_force, cps):
    """
    Run Tool 2 side+edge together in one grouped sequence while Tool 2 is mounted.
    This keeps task selection grouped by the same tool and same 7th-axis family.
    """
    sections = ("bottom", "left", "top", "right")
    total_steps = max(side_count, edge_count)
    for step in range(total_steps):
        if stop_requested():
            return
        print(f"\n=== TOOL2 GROUP STEP {step+1}/{total_steps} (section-by-section) ===")
        for idx, section_name in enumerate(sections):
            if stop_requested():
                return
            side_runs = step < side_count
            edge_runs = step < edge_count
            is_final_section = (step == total_steps - 1) and (idx == len(sections) - 1)

            if side_runs:
                side_return_home = is_final_section and not edge_runs
                print(f"Tool2 SIDE section: {section_name}")
                run_tool2side_cycles(
                    1,
                    side_force,
                    cps,
                    section=section_name,
                    return_home=side_return_home,
                )
            if edge_runs:
                edge_return_home = is_final_section
                print(f"Tool2 EDGE section: {section_name}")
                run_tool2sideoutedge_cycles(
                    1,
                    edge_force,
                    cps,
                    section=section_name,
                    return_home=edge_return_home,
                )
        if step < total_steps - 1:
            time.sleep(0.5)

def run_tool3_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        model2thirdtoolrun(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)

def check_tool(cps, config, tool_num, ci0, ci1, ci2):
    # Check Conditions
    if ci0 is None or ci1 is None or ci2 is None:
        print("Failed to read one or more CI bits.")
        return None

    tool_in_hand = None
    if ci0 == 0 and ci1 == 0 and ci2 == 0:
        tool_in_hand = None
    elif ci0 == 1 and ci1 == 0 and ci2 == 0:
        tool_in_hand = 3
    elif ci0 == 0 and ci1 == 1 and ci2 == 0:
        tool_in_hand = 2
    elif ci0 == 0 and ci1 == 1 and ci2 == 1:
        tool_in_hand = 1
    elif ci0 == 0 and ci1 == 0 and ci2 == 1:
        tool_in_hand = 4
    else:
        print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")
        return None

    if tool_in_hand is None:
        print("No tool in hand")
        return False

    if tool_in_hand == tool_num:
        print(f"Tool {tool_in_hand} already in hand; skipping drop/pick.")
        return True

    print(f"Tool {tool_in_hand} detected; dropping before picking Tool {tool_num}.")
    seventh_result = communicate(
        cps=cps,
        config=config,
        seventh=0,
        tcp=config["coords"]["tcptool1plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=0.3,
        velocity_profile="robot",
        wait=False,
        require_seventh_ok=True,
    )
    if seventh_result is None:
        raise RuntimeError(
            f"Failed to move 7th axis to tool station before dropping Tool {tool_in_hand}."
        )
    keepTool11(
        cps=cps,
        toolNumber=tool_in_hand,
        config=config,
        goToSafe=False,
        startFromSafe=True,
    )
    return False

def startingRobotToSandmodel2():

    # side_cycles   = 1
    # zigzag_cycles = 1
    # tool2_side_cycle=1
    # tool2_sideoutedge=1
    # tool3_cycles=1
    # force_side_cycles=5
    # force_zigzag_cycles = 5
    # force_tool2_side_cycle=5
    # force_tool2_sideoutedge=5
    # force_tool3=5

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
    tool3_cycles= int(json_config_TableB['3D']['cycle'])
    
    force_side_cycles = int(json_config_TableB['frame']['force'])
    force_zigzag_cycles = int(json_config_TableB['pocketzigzag']['force'])
    force_tool2_side_cycle = int(json_config_TableB['side']['force'])
    force_tool2_sideoutedge = int(json_config_TableB['edgeOutside']['force']) 
    force_tool3=int(json_config_TableB['3D']['force'])
    
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
    # Model B workflow:
    # Tool 3 => edge coverage only
    # Tool 4 => frame + pocket zigzag

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
            wait=False,
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
        has_tool4_frame_and_zigzag = (side_cycles > 0 or has_pocket_work)
        has_any_task = (
            has_tool3_edge_pass
            or has_tool4_frame_and_zigzag
            or tool2_side_cycle > 0
            or tool2_sideoutedge > 0
            or tool3_cycles > 0
        )

        if has_any_task:
            move_to_safe_point()
        work_executed = False

        if has_pocket_work and not edge_coverage:
            print("Pocket zigzag is selected: forcing edge-coverage pass on Tool 3.")

        if has_tool3_edge_pass:
            ensure_tool_in_hand(3)
            if stop_requested():
                return

            # Edge coverage pass only.
            run_zigzag_cycles(
                count=zigzag_cycles,
                force=force_zigzag_cycles,
                innerSandingOffset=innerSandingOffset,
                cps=cps,
                movement="edge_only",
                tcp_name="tcpReal",
            )
            if stop_requested():
                return
            work_executed = True
            
            print("\nTool 3 edge coverage completed successfully!")

        if has_tool4_frame_and_zigzag:
            ensure_tool_in_hand(4)
            if stop_requested():
                return

            # Frame with Tool 4
            run_side_cycles(side_cycles,force_side_cycles,cps)
            # Zigzag only with Tool 4 (edge is excluded)
            run_zigzag_cycles(
                count=zigzag_cycles,
                force=force_zigzag_cycles,
                innerSandingOffset=innerSandingOffset,
                cps=cps,
                movement="zigzag_only",
                tcp_name="tcptool4plane2",
                direct_prepoint=(side_cycles > 0),
            )
            if stop_requested():
                return
            work_executed = True
            print("\nTool 4 frame+zigzag completed successfully!")
        
        if tool2_side_cycle > 0 or  tool2_sideoutedge > 0:
            ensure_tool_in_hand(2)
            if stop_requested():
                return

            if tool2_side_cycle > 0 and tool2_sideoutedge > 0:
                run_tool2_combined_cycles(
                    tool2_side_cycle,
                    force_tool2_side_cycle,
                    tool2_sideoutedge,
                    force_tool2_sideoutedge,
                    cps,
                )
            else:
                # Tool 2 Side Cycles
                run_tool2side_cycles(tool2_side_cycle,force_tool2_side_cycle,cps)
                # Tool 2 Side Out Edge Cycles
                run_tool2sideoutedge_cycles(tool2_sideoutedge,force_tool2_sideoutedge,cps)
            if stop_requested():
                return
            work_executed = True

        if tool3_cycles>0:
            ensure_tool_in_hand(1)
            if stop_requested():
                return

            #Tool 3Cycle
            run_tool3_cycles(tool3_cycles,force_tool3,cps)
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
    startingRobotToSandmodel2() 
    

