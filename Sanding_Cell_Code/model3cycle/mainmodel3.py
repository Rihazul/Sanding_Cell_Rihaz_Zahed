import json
import os
import sys
import time

import yaml

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cycle_data_utils import get_inverse_overlap_step
from model3cycle.model3zigb import mod3zigbig
from model3cycle.mod3sideb import mod3sidebig
from model3cycle.mod3tool2b import model3tool2big
from model3cycle.mod3tool2edgeb import mod3tool2edgesbig
from model3cycle.mod3tool3b import mod3tool3bigrun
from modules.CPS import CPSClient
from Server_Better_V2 import communicate, getTool11, keepTool11, move_to_task_safe_point, setup_logger, stop_requested


def load_config():
    with open("./configs/config.yaml", "r") as file:
        return yaml.safe_load(file)


def load_json_config():
    with open("./configs/cycleData.json", "r") as file:
        return json.load(file)


def run_side_cycles(count, force, cps):
    for i in range(count):
        print(f"\n=== FRAME CYCLE {i+1}/{count} ===")
        mod3sidebig(force=force, cps=cps)
        if i < count - 1:
            time.sleep(0.5)


def run_zigzag_cycles(count, force, inner_sanding_offset, cps, edge_coverage_override=None):
    for i in range(count):
        print(f"\n=== POCKET ZIGZAG CYCLE {i+1}/{count} ===")
        mod3zigbig(
            force=force,
            innerSandingOffset=inner_sanding_offset,
            cps=cps,
            edge_coverage_override=edge_coverage_override,
        )
        if i < count - 1:
            time.sleep(0.5)


def run_tool4_interleaved_cycles(
    frame_cycles,
    frame_force,
    pocket_cycles,
    pocket_force,
    inner_sanding_offset,
    cps,
    edge_coverage_override=None,
):
    """
    Interleave Tool-4 frame and pocket work cycle-by-cycle to reduce
    unnecessary long travel between large task blocks.
    """
    frame_done = 0
    pocket_done = 0
    total_steps = max(frame_cycles, pocket_cycles)

    for step in range(total_steps):
        if frame_done < frame_cycles:
            print(f"\n=== TOOL4 INTERLEAVED STEP {step+1}: FRAME ===")
            run_side_cycles(1, frame_force, cps)
            frame_done += 1
            if stop_requested():
                return

        if pocket_done < pocket_cycles:
            print(f"\n=== TOOL4 INTERLEAVED STEP {step+1}: POCKET ZIGZAG ===")
            run_zigzag_cycles(
                1,
                pocket_force,
                inner_sanding_offset,
                cps,
                edge_coverage_override=edge_coverage_override,
            )
            pocket_done += 1
            if stop_requested():
                return


def run_tool2side_cycles(count, force, cps, section=None, return_home=True, pass_index=None):
    result = False
    for i in range(count):
        print(f"\n=== TOOL 2 SIDE CYCLE {i+1}/{count} ===")
        result = bool(model3tool2big(
            force=force,
            cps=cps,
            section=section,
            return_home=return_home,
            pass_index=pass_index,
        ))
        if i < count - 1:
            time.sleep(0.5)
    return result


def run_tool2sideoutedge_cycles(count, force, cps, section=None, return_home=True, pass_index=None):
    result = False
    for i in range(count):
        print(f"\n=== TOOL 2 EDGE-OUTSIDE CYCLE {i+1}/{count} ===")
        result = bool(mod3tool2edgesbig(
            force=force,
            cps=cps,
            section=section,
            return_home=return_home,
            pass_index=pass_index,
        ))
        if i < count - 1:
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
            for section_pass_index in range(max_passes_by_section[section_name]):
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

def run_tool3_cycles(count, force, cps):
    for i in range(count):
        print(f"\n=== TOOL 1 (3D) CYCLE {i+1}/{count} ===")
        mod3tool3bigrun(force=force, cps=cps)
        if i < count - 1:
            time.sleep(0.5)


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
        velocity_profile="robotspeed",
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


def startingRobotToSandmodel3():
    json_config = load_json_config()
    json_config_table_b = json_config["TableB"]
    inner_sanding_offset = get_inverse_overlap_step(json_config)

    pocketzigzag_cfg = json_config_table_b.get("pocketzigzag", {})
    edge_coverage = bool(pocketzigzag_cfg.get("edgeCoverage", False))

    side_cycles = int(json_config_table_b["frame"]["cycle"])
    zigzag_cycles = int(json_config_table_b["pocketzigzag"]["cycle"])
    tool2_side_cycle = int(json_config_table_b["side"]["cycle"])
    tool2_sideoutedge = int(json_config_table_b["edgeOutside"]["cycle"])
    tool1_3dcycle = int(json_config_table_b["3D"]["cycle"])

    force_side_cycles = int(json_config_table_b["frame"]["force"])
    force_zigzag_cycles = int(json_config_table_b["pocketzigzag"]["force"])
    force_tool2_side_cycle = int(json_config_table_b["side"]["force"])
    force_tool2_sideoutedge = int(json_config_table_b["edgeOutside"]["force"])
    force_tool1_3d = int(json_config_table_b["3D"]["force"])

    speed = float(json_config["robotSpeed"])
    speed_profile = {"travel": speed}

    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

    print(
        "[mainmodel3] Task config -> "
        f"frame(cycle={side_cycles}, force={force_side_cycles}), "
        f"pocket(cycle={zigzag_cycles}, force={force_zigzag_cycles}), "
        f"side(cycle={tool2_side_cycle}, force={force_tool2_side_cycle}), "
        f"edgeOutside(cycle={tool2_sideoutedge}, force={force_tool2_sideoutedge}), "
        f"3D(cycle={tool1_3dcycle}, force={force_tool1_3d})"
    )

    cps = None
    try:
        cps = CPSClient()
        ret = cps.HRIF_Connect(0, config["server"]["cpip"], config["server"]["cps"])
        if ret != 0:
            print(f"Failed to connect, error code: {ret}")
            return

        def read_ci_bit(bit_index):
            result = []
            nret = cps.HRIF_ReadBoxCI(0, bit_index, result)
            if nret != 0:
                print(f"Error reading CI[{bit_index}], error code: {nret}")
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
            move_to_task_safe_point(
                cps=cps,
                config=config,
                speed=speed_profile["travel"],
                velocity_profile="robotspeed",
            )

        def move_seventh_to_tool_station():
            seventh_result = communicate(
                cps=cps,
                config=config,
                seventh=0,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                speed=speed_profile["travel"],
                velocity_profile="robotspeed",
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
        has_tool4_work = side_cycles > 0 or has_pocket_work
        has_tool2_work = tool2_side_cycle > 0 or tool2_sideoutedge > 0
        has_tool1_work = tool1_3dcycle > 0
        has_any_task = has_tool4_work or has_tool2_work or has_tool1_work

        if has_any_task:
            move_to_safe_point()

        work_executed = False
        force_edge_first = has_pocket_work
        if has_pocket_work and not edge_coverage:
            print("Pocket zigzag selected: forcing edge-coverage first pass.")

        # Tool 4 batch: frame + pocket zigzag
        if has_tool4_work:
            ensure_tool_in_hand(4)
            if stop_requested():
                return

            if side_cycles > 0 and zigzag_cycles > 0:
                run_tool4_interleaved_cycles(
                    frame_cycles=side_cycles,
                    frame_force=force_side_cycles,
                    pocket_cycles=zigzag_cycles,
                    pocket_force=force_zigzag_cycles,
                    inner_sanding_offset=inner_sanding_offset,
                    cps=cps,
                    edge_coverage_override=True if force_edge_first else None,
                )
                if stop_requested():
                    return
                work_executed = True
            else:
                if side_cycles > 0:
                    run_side_cycles(side_cycles, force_side_cycles, cps)
                    if stop_requested():
                        return
                    work_executed = True

                if zigzag_cycles > 0:
                    run_zigzag_cycles(
                        zigzag_cycles,
                        force_zigzag_cycles,
                        inner_sanding_offset,
                        cps,
                        edge_coverage_override=True if force_edge_first else None,
                    )
                    if stop_requested():
                        return
                    work_executed = True

        # Tool 2 batch: side + outside edge
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

        # Tool 1 batch: 3D
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
    startingRobotToSandmodel3()
