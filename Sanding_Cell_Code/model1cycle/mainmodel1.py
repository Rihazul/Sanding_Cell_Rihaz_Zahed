import sys
import os
import time
import yaml
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from model1cycle.mod1pocketside import mod1tool1siderun
from model1cycle.mod1zigzag import mod1zigzag
from model1cycle.mod1tool2edge import mod1tool2outedge
from model1cycle.mod1tool2sideb import mod1tool2sidesrun
from model1cycle.mod1tool3 import mod1tool3
from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate,keepToolupdated,getToolUpdated
from modules.CPS import CPSClient
import time

from cycle_data_utils import get_spiral_settings

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

def run_zigzag_cycles(count,force,innerSandingOffset,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1zigzag(force=force,innerSandingOffset=innerSandingOffset,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)


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

def run_tool3_cycles(count,force,cps):
    """Execute zigzag function with specified number of cycles"""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ===")
        mod1tool3(force=force,cps=cps)
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


def startingRobotToSandmodel1():
 
    #side_cycles   = 0
    #zigzag_cycles = 1
    #tool2_side_cycle=1
    #tool2_sideoutedge=1
    #tool3_cycles=1
    #force_side_cycles=5
    #force_zigzag_cycles = 5
    #force_tool2_side_cycle=5
    #force_tool2_sideoutedge=5
    #force_tool3=5
    
    
    json_config = load_json_config()
    innerSandingOffset=int(json_config['inverseOverlapping'])
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
            check_tool(cps=cps,config=config,tool_num=3,ci0=ci0,ci1=ci1,ci2=ci2)
        #Intitial Position
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #Pick Tool 3
            getTool11(cps, toolNumber=3, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            # Run side cycles
            run_side_cycles(side_cycles,force_side_cycles,cps)
            
            # Run zigzag cycles
            run_zigzag_cycles(zigzag_cycles,force_zigzag_cycles,innerSandingOffset,cps) 

            #Keep Tool 3
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps, config=config, seventh=0, tcp=config['coords']['tcptool1plane1'], ucs=config['coords']['ucsTable1'], speed=0.3, wait=True)
            #keepTool11(cps, toolNumber=3, config=config)
            cycles = [tool2_side_cycle, tool2_sideoutedge,tool3_cycles]
            if any(cycle > 0 for cycle in cycles):
                keepToolupdated(cps, toolNumber=3, config=config)
            else:
                keepTool11(cps, toolNumber=3, config=config)
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            
            print("\nAll operations completed for Tool1 successfully!")
            
        
        if tool2_side_cycle > 0 or  tool2_sideoutedge > 0:
            check_tool(cps=cps,config=config,tool_num=2,ci0=ci0,ci1=ci1,ci2=ci2)

            #Pick Tool 2
            # getTool11(cps, toolNumber=2, config=config)
            # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)
            cycles = [side_cycles,zigzag_cycles]
            
            if any(cycle > 0 for cycle in cycles):
                print("At least one cycle > 0 → running getToolUpdated()")
                getToolUpdated(cps, toolNumber=2, config=config)
            else:
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                getTool11(cps, toolNumber=2, config=config)
            
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)

            #Tool 2 Side Cycles
            run_tool2side_cycles(tool2_side_cycle,force_tool2_side_cycle,cps)

            #Tool 2 Side Out Edge Cycles
            run_tool2sideoutedge_cycles(tool2_sideoutedge,force_tool2_sideoutedge,cps)

            #Drop Tool 2
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
            #keepTool11(cps, toolNumber=2, config=config)
            cycles = [tool3_cycles]
            if any(cycle > 0 for cycle in cycles):
                keepToolupdated(cps, toolNumber=2, config=config)
            else:
                keepTool11(cps, toolNumber=2, config=config)
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)

        if tool3_cycles>0:
            check_tool(cps=cps,config=config,tool_num=1,ci0=ci0,ci1=ci1,ci2=ci2)

            #Pick Tool 2
            # getTool11(cps, toolNumber=1, config=config)
            # communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)
            cycles = [tool2_side_cycle, tool2_sideoutedge, side_cycles,zigzag_cycles]

            if any(cycle > 0 for cycle in cycles):
                print("At least one cycle > 0 → running getToolUpdated()")
                getToolUpdated(cps, toolNumber=1, config=config)
            else:
                communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
                getTool11(cps, toolNumber=1, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            #Tool 3Cycle
            run_tool3_cycles(tool3_cycles,force_tool3,cps)
            #Drop Tool 2
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)
            communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcptool1plane1'],ucs=config['coords']['ucsTable1'],speed=0.3,wait=True)
            keepTool11(cps, toolNumber=1, config=config)
            communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=speeed, wait=True)


    except Exception as e:
        print(f"\nExecution error: {str(e)}")
    finally:
        print("\nSequence terminated")

if __name__ == "__main__":
    startingRobotToSandmodel1() 