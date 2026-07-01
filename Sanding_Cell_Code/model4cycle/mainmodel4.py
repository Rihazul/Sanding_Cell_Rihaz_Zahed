import sys
import os
import time
import yaml
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))




from model4cycle.testmodel4sidesmall import testmodel4sidesmallfunction
from model4cycle.testmodel4zigzagsmall import testmodel4zigzagsmallfunction
from model4cycle.testmodel4tool2sidesmall import testmodel4tool2sidesmallfunction
from model4cycle.testmodel4tooloutedgesmall import testmodel4tooloutedgesmallfunction
from model4cycle.testmodel4tool3s import testmodel4tool3s
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
        testmodel4sidesmallfunction(force=force,cps=cps)
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
    edge_coverage_override=None,
):
    """Execute zigzag function with specified number of cycles."""
    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i+1}/{count} ({movement}) ===")
        testmodel4zigzagsmallfunction(
            force=force,
            innerSandingOffset=innerSandingOffset,
            cps=cps,
            movement=movement,
            tcp_name=tcp_name,
            direct_prepoint=direct_prepoint,
            edge_coverage_override=edge_coverage_override,
        )
        if i < count-1:
            time.sleep(0.5)


def run_tool2side_cycles(count,force,cps,section=None,return_home=True,pass_index=None):
    """Execute Tool 2 side cycles and return whether motion was executed."""
    result = False
    for i in range(count):
        print(f"\n=== TOOL 2 SIDE CYCLE {i+1}/{count} ===")
        result = bool(testmodel4tool2sidesmallfunction(
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
        result = bool(testmodel4tooloutedgesmallfunction(
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
        testmodel4tool3s(force=force,cps=cps)
        if i < count-1:
            print("Pausing 3 seconds before next zigzag cycle...")
            time.sleep(3)

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
    seventh_result = communicate(
        cps=cps,
        config=config,
        seventh=0,
        tcp=config["coords"]["tcptool1plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=0.3,
        velocity_profile="robot",
        wait=True,
        require_seventh_ok=True,
    )
    if seventh_result is None:
        raise RuntimeError(
            f"Failed to move 7th axis to tool station before dropping tool {tool_in_hand}."
        )
    keepTool11(
        cps=cps,
        toolNumber=tool_in_hand,
        config=config,
        goToSafe=False,
        startFromSafe=True,
    )
    return False

def startingRobotToSandmodel4():
    json_config = load_json_config()
    json_config_TableB = json_config['TableB']
    innerSandingOffsetOverlapping = get_inverse_overlap_step(json_config)

    pocketzigzag_cfg = json_config_TableB.get('pocketzigzag', {})
    edge_coverage = bool(pocketzigzag_cfg.get('edgeCoverage', False))

    side_cycles = int(json_config_TableB['frame']['cycle'])
    zigzag_cycles = int(json_config_TableB['pocketzigzag']['cycle'])
    tool2_side_cycle = int(json_config_TableB['side']['cycle'])
    tool2_sideoutedge = int(json_config_TableB['edgeOutside']['cycle'])
    tool1_3dcycle = int(json_config_TableB['3D']['cycle'])

    force_side_cycles = int(json_config_TableB['frame']['force'])
    force_zigzag_cycles = int(json_config_TableB['pocketzigzag']['force'])
    force_tool2_side_cycle = int(json_config_TableB['side']['force'])
    force_tool2_sideoutedge = int(json_config_TableB['edgeOutside']['force'])
    force_tool1_3d = int(json_config_TableB['3D']['force'])

    speeed = float(json_config['robotSpeed'])
    speed_profile = {
        "travel": speeed,
        "tool_change": speeed,
    }

    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])
    keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

    print(
        "[mainmodel4] Task config -> "
        f"frame(cycle={side_cycles}, force={force_side_cycles}), "
        f"pocket(cycle={zigzag_cycles}, force={force_zigzag_cycles}), "
        f"side(cycle={tool2_side_cycle}, force={force_tool2_side_cycle}), "
        f"edgeOutside(cycle={tool2_sideoutedge}, force={force_tool2_sideoutedge}), "
        f"3D(cycle={tool1_3dcycle}, force={force_tool1_3d})"
    )

    cps = None
    try:
        cps = CPSClient()
        ret = cps.HRIF_Connect(0, config['server']['cpip'], config['server']['cps'])
        if ret != 0:
            print(f"Failed to connect, error code: {ret}")
            return

        def read_ci_bit(bit_index):
            result = []
            nRet = cps.HRIF_ReadBoxCI(0, bit_index, result)
            if nRet != 0:
                print(f"Error reading CI[{bit_index}], error code: {nRet}")
                return None
            return int(result[0])

        def read_ci_triplet():
            return read_ci_bit(0), read_ci_bit(1), read_ci_bit(2)

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
            move_to_safe_point()
            move_seventh_to_tool_station()
            move_to_safe_point()

        has_pocket_work = zigzag_cycles > 0
        has_tool3_edge_pass = has_pocket_work
        has_tool4_work = side_cycles > 0 or has_pocket_work
        has_tool2_work = tool2_side_cycle > 0 or tool2_sideoutedge > 0
        has_tool1_work = tool1_3dcycle > 0
        has_any_task = has_tool3_edge_pass or has_tool4_work or has_tool2_work or has_tool1_work

        if has_any_task:
            move_to_safe_point()

        work_executed = False
        if has_pocket_work and not edge_coverage:
            print("Pocket zigzag selected: forcing Tool 3 edge-coverage first pass.")

        # Tool 3: pocket edge coverage first.
        if has_tool3_edge_pass:
            ensure_tool_in_hand(3)
            if stop_requested():
                return
            run_zigzag_cycles(
                count=zigzag_cycles,
                force=force_zigzag_cycles,
                innerSandingOffset=innerSandingOffsetOverlapping,
                cps=cps,
                movement="edge_only",
                tcp_name="tcpReal",
                edge_coverage_override=True,
            )
            if stop_requested():
                return
            work_executed = True
            print("\nTool 3 pocket edge coverage completed successfully.")

        # Tool 4: frame and pocket zigzag.
        if has_tool4_work:
            ensure_tool_in_hand(4)
            if stop_requested():
                return
            if side_cycles > 0:
                run_side_cycles(side_cycles, force_side_cycles, cps)
                if stop_requested():
                    return
                work_executed = True
            if zigzag_cycles > 0:
                run_zigzag_cycles(
                    count=zigzag_cycles,
                    force=force_zigzag_cycles,
                    innerSandingOffset=innerSandingOffsetOverlapping,
                    cps=cps,
                    movement="zigzag_only",
                    tcp_name="tcptool4plane2",
                    direct_prepoint=(side_cycles > 0),
                    edge_coverage_override=False,
                )
                if stop_requested():
                    return
                work_executed = True
            print("\nTool 4 frame/pocket work completed successfully.")

        # Tool 2: side and outside-edge operations.
        if has_tool2_work:
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
                run_tool2side_cycles(tool2_side_cycle, force_tool2_side_cycle, cps)
                run_tool2sideoutedge_cycles(tool2_sideoutedge, force_tool2_sideoutedge, cps)
            if stop_requested():
                return
            work_executed = True

        # Tool 1: 3D operation.
        if has_tool1_work:
            ensure_tool_in_hand(1)
            if stop_requested():
                return
            run_tool3_cycles(tool1_3dcycle, force_tool1_3d, cps)
            if stop_requested():
                return
            work_executed = True

        if work_executed:
            if keep_tool_after_task:
                move_to_homing_with_tool()
                print("Task completed: moved to homing while keeping current tool mounted.")
            else:
                ci0_end, ci1_end, ci2_end = read_ci_triplet()
                tool_in_hand = decode_tool_in_hand(ci0_end, ci1_end, ci2_end)
                if tool_in_hand is not None:
                    move_to_safe_point()
                    move_seventh_to_tool_station()
                    keepTool11(cps, toolNumber=tool_in_hand, config=config)
                else:
                    print("Task completed: no mounted tool to drop.")
        else:
            print("No cycles selected. Nothing to execute.")

    except Exception as e:
        print(f"\nExecution error: {str(e)}")
    finally:
        print("\nSequence terminated")
        if cps is not None:
            try:
                cps.HRIF_DisConnect(0)
            except Exception:
                pass

if __name__ == "__main__":
    startingRobotToSandmodel4() 
