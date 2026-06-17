import sys
import os
import time
import yaml
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))



from model5cycle.outpocketsmall import model5outpocketsrun
from model5cycle.zigzagsmall import mod5zigzagrun
from model5cycle.tool2sidesmall import mod5tool2sidesrun
from model5cycle.tool2sideedgesmall import testmodel5tooloutedgerun
from model5cycle.tool3cyclesmall import tool3run

from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate,keepToolupdated,getToolUpdated, stop_requested
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
        model5outpocketsrun(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next side cycle...")
            time.sleep(3)

def run_zigzag_cycles(count,force,innerSandingOffset,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod5zigzagrun(force=force,innerSandingOffset=innerSandingOffset,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


def run_tool2side_cycles(count,force,cps,section=None,return_home=True,pass_index=None):
    """Execute Tool 2 side cycles and return whether motion was executed."""
    result = False
    for i in range(count):
        print(f"\n=== TOOL 2 SIDE CYCLE {i+1}/{count} ===")
        result = bool(mod5tool2sidesrun(
            force=force,
            cps=cps,
            section=section,
            return_home=return_home,
            pass_index=pass_index,
        ))
        if i < count-1:
            time.sleep(0.5)
    return result


def run_tool2sideoutedge_cycles(count,force,cps,section=None,return_home=True,pass_index=None):
    """Execute Tool 2 outside-edge cycles and return whether motion was executed."""
    result = False
    for i in range(count):
        print(f"\n=== TOOL 2 EDGE-OUTSIDE CYCLE {i+1}/{count} ===")
        result = bool(testmodel5tooloutedgerun(
            force=force,
            cps=cps,
            section=section,
            return_home=return_home,
            pass_index=pass_index,
        ))
        if i < count-1:
            time.sleep(0.5)
    return result


def run_tool2_combined_cycles(side_count, side_force, edge_count, edge_force, cps):
    """Run Tool 2 side and outside-edge on each 7th-axis pass before advancing."""
    sections = ("bottom", "left", "top", "right")
    max_passes_by_section = {"bottom": 8, "left": 1, "top": 8, "right": 1}
    total_steps = max(side_count, edge_count)
    for step in range(total_steps):
        if stop_requested():
            return
        print(f"\n=== TOOL2 GROUP STEP {step+1}/{total_steps} (pass-by-pass) ===")
        side_runs = step < side_count
        edge_runs = step < edge_count
        for section_name in sections:
            if stop_requested():
                return
            max_passes = max_passes_by_section[section_name]
            for section_pass_index in range(max_passes):
                if stop_requested():
                    return
                side_executed = False
                edge_executed = False
                final_right_pass = (
                    step == total_steps - 1
                    and section_name == "right"
                    and section_pass_index == 0
                )
                if side_runs:
                    side_return_home = final_right_pass and not edge_runs
                    print(
                        f"Tool2 SIDE section={section_name} pass={section_pass_index}"
                    )
                    side_executed = run_tool2side_cycles(
                        1,
                        side_force,
                        cps,
                        section=section_name,
                        return_home=side_return_home,
                        pass_index=section_pass_index,
                    )
                if edge_runs:
                    edge_return_home = final_right_pass
                    print(
                        f"Tool2 EDGE section={section_name} pass={section_pass_index}"
                    )
                    edge_executed = run_tool2sideoutedge_cycles(
                        1,
                        edge_force,
                        cps,
                        section=section_name,
                        return_home=edge_return_home,
                        pass_index=section_pass_index,
                    )
                if not side_executed and not edge_executed:
                    break
        if step < total_steps - 1:
            time.sleep(0.5)

def run_tool3_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        tool3run(force=force,cps=cps)
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
    else:
        print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")
        return None

    if tool_in_hand is None:
        print("No tool in hand")
        return None

    if tool_in_hand == tool_num:
        print(f"Tool {tool_in_hand} already in hand; skipping drop/pick.")
        return tool_in_hand

    print(f"Tool {tool_in_hand} detected → executing keepTool11()")
    keepTool11(cps, toolNumber=tool_in_hand, config=config)
    return None

def startingRobotToSandmodel5():

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
    
    # speeed=0.7
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

    # Read CI values
    ci0 = read_ci_bit(cps, 0)
    ci1 = read_ci_bit(cps, 1)
    ci2 = read_ci_bit(cps, 2)
    """Main control function"""
    try:
        if side_cycles > 0 or zigzag_cycles > 0:
            tool_in_hand = check_tool(cps=cps,config=config,tool_num=3,ci0=ci0,ci1=ci1,ci2=ci2)
            if stop_requested():
                return
        #Intitial Position
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #Pick Tool 3
            if tool_in_hand != 3:
                getTool11(cps, toolNumber=3, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            # Run side cycles
            run_side_cycles(side_cycles,force_side_cycles,cps)
            
            # Run zigzag cycles
            run_zigzag_cycles(zigzag_cycles,force_zigzag_cycles,innerSandingOffset,cps) 
            if stop_requested():
                return

            #Keep Tool 3
            has_followup_after_tool3 = (tool2_side_cycle > 0 or tool2_sideoutedge > 0 or tool3_cycles > 0)
            if not stop_requested():
                if has_followup_after_tool3 or not keep_tool_after_task:
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                    communicate(cps=cps, config=config, seventh=0, tcp=config['coords']['tcptool1plane1'], ucs=config['coords']['ucsTable1'], speed=0.3, wait=True)
                    time.sleep(0.2)
                    cycles = [tool2_side_cycle, tool2_sideoutedge,tool3_cycles]
                    if any(cycle > 0 for cycle in cycles):
                        keepToolupdated(cps, toolNumber=3, config=config)
                    else:
                        keepTool11(cps, toolNumber=3, config=config)
                        communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            
            print("\nAll operations completed for Tool1 successfully!")
            
        
        if tool2_side_cycle > 0 or  tool2_sideoutedge > 0:
            tool_in_hand = check_tool(cps=cps,config=config,tool_num=2,ci0=ci0,ci1=ci1,ci2=ci2)
            if stop_requested():
                return

            #Pick Tool 2
            # getTool11(cps, toolNumber=2, config=config)
            # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            cycles = [side_cycles,zigzag_cycles]
            
            if tool_in_hand != 2:
                if any(cycle > 0 for cycle in cycles):
                    print("At least one cycle > 0 → running getToolUpdated()")
                    getToolUpdated(cps, toolNumber=2, config=config)
                else:
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                    getTool11(cps, toolNumber=2, config=config)
            
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            if tool2_side_cycle > 0 and tool2_sideoutedge > 0:
                run_tool2_combined_cycles(
                    tool2_side_cycle,
                    force_tool2_side_cycle,
                    tool2_sideoutedge,
                    force_tool2_sideoutedge,
                    cps,
                )
            else:
                run_tool2side_cycles(tool2_side_cycle,force_tool2_side_cycle,cps)
                run_tool2sideoutedge_cycles(tool2_sideoutedge,force_tool2_sideoutedge,cps)
            if stop_requested():
                return

            #Drop Tool 2
            has_followup_after_tool2 = tool3_cycles > 0
            if not stop_requested():
                if has_followup_after_tool2 or not keep_tool_after_task:
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
                    time.sleep(0.2)
                    cycles = [tool3_cycles]
                    if any(cycle > 0 for cycle in cycles):
                        keepToolupdated(cps, toolNumber=2, config=config)
                    else:
                        keepTool11(cps, toolNumber=2, config=config)
                        communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)

        if tool3_cycles>0:
            tool_in_hand = check_tool(cps=cps,config=config,tool_num=1,ci0=ci0,ci1=ci1,ci2=ci2)
            if stop_requested():
                return

            #Pick Tool 2
            # getTool11(cps, toolNumber=1, config=config)
            # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            cycles = [tool2_side_cycle, tool2_sideoutedge, side_cycles,zigzag_cycles]

            if tool_in_hand != 1:
                if any(cycle > 0 for cycle in cycles):
                    print("At least one cycle > 0 → running getToolUpdated()")
                    getToolUpdated(cps, toolNumber=1, config=config)
                else:
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                    getTool11(cps, toolNumber=1, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #Tool 3Cycle
            run_tool3_cycles(tool3_cycles,force_tool3,cps)
            if stop_requested():
                return
            if not keep_tool_after_task:
                #Drop Tool 1
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
                time.sleep(0.2)
                keepTool11(cps, toolNumber=1, config=config)
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)


    except Exception as e:
        print(f"\nExecution error: {str(e)}")
    finally:
        print("\nSequence terminated")

if __name__ == "__main__":
    startingRobotToSandmodel5() 
    
