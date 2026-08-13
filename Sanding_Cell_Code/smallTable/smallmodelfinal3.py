import sys
import os
import time
import yaml
import traceback
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from smallTable.frameplane1final import smalldoor1side,smalldoor2side,smalldoor3side,smalldoor4side
from smallTable.zigzagplane1final import smalldoor1zizag,smalldoor2zizag,smalldoor3zizag,smalldoor4zizag
from smallTable.edge_coverage_tool3 import run_tool3_pocket_edge_cycles
from smallTable.frame1tool2sidefinal import door1frametool2side,door2frametool2side,door3frametool2side,door4frametool2side
from smallTable.frame1tool2edgefinal import door1frametool2sideedge,door2frametool2sideedge,door3frametool2sideedge,door4frametool2sideedge
from smallTable.frame1tool2sideedge_combined import run_tool2_side_edge_combined
from smallTable.frame1tool3 import smalldoor1tool3,smalldoor2tool3,smalldoor3tool3,smalldoor4tool3
from Server_Better_V2 import keepTool11,setup_logger,getTool11,communicate,stop_requested,moveOnlyJ6r,move_to_task_safe_point,move_to_robot_homing_point
from modules.CPS import CPSClient
import time

from cycle_data_utils import any_cycles, doors_with_cycles, get_spiral_settings, get_tableA_task_by_door

INTER_PASS_DELAY_SECONDS = 0.0
J7_IDLE_TIMEOUT_S = 45.0
J7_IDLE_POLL_S = 0.02


def wait_for_j7_idle(
    cps: CPSClient,
    timeout_s: float = J7_IDLE_TIMEOUT_S,
    poll_s: float = J7_IDLE_POLL_S,
) -> bool:
    start_time = time.time()
    result = []
    while True:
        result.clear()
        nret = cps.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], result)
        if (
            (nret == 0 or nret is None)
            and len(result) > 2
            and str(result[2]).strip() == "0"
        ):
            return True
        if (time.time() - start_time) >= float(timeout_s):
            print(
                f"[J7] Timeout waiting for idle after {timeout_s:.1f}s. "
                f"Last state={result}"
            )
            return False
        time.sleep(max(0.005, float(poll_s)))

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

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

    print(f"\n=== FRAME CONTINUOUS CYCLES: {count} (Door {door_num}) ===")
    door_func(force=force,cps=cps, cycles=count)



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

    print(f"\n=== ZIGZAG CONTINUOUS CYCLES: {count} (Door {door_num}) ===")
    door_func(
        force=force,
        z=z,
        cps=cps,
        orientation=orientation,
        movement=movement,
        spiral_settings=spiral_settings,
        cycles=count,
    )



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

    if stop_requested():
        raise RuntimeError("[Tool 2 Side] Stop requested.")
    print(f"\n=== SIDE CYCLES 1/{count} to {count}/{count} (Door {door_num}) ===")
    door_func(force=force, cps=cps, cycles=count)
    if stop_requested():
        raise RuntimeError("[Tool 2 Side] Stop requested.")


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

    if stop_requested():
        raise RuntimeError("[Tool 2 Edge] Stop requested.")
    print(f"\n=== EDGE CYCLES 1/{count} to {count}/{count} (Door {door_num}) ===")
    door_func(force=force, cps=cps, cycles=count)
    if stop_requested():
        raise RuntimeError("[Tool 2 Edge] Stop requested.")


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

    print(f"\n=== TOOL 1 CONTINUOUS CYCLES: {count} (Door {door_num}) ===")
    door_func(z=z, cps=cps, force=force, cycles=count)


def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config


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
    move_to_robot_homing_point(
        cps=cps,
        config=config,
        speed=float(config.get("UI", {}).get("robotSpeed", 0.7)),
        velocity_profile="robotspeed",
    )
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
            f"Failed to move J7 to the tool station before dropping tool {tool_in_hand}."
        )
    if not wait_for_j7_idle(cps):
        raise RuntimeError(
            f"J7 did not become idle before dropping tool {tool_in_hand}."
        )
    keepTool11(
        cps,
        toolNumber=tool_in_hand,
        config=config,
        goToSafe=False,
        startFromSafe=True,
    )
    if tool_in_hand == 1 and tool_num == 3:
        print("Tool 1 -> Tool 3 transition: applying joint-only correction J1=+30, J6=-90.")
        moveOnlyJ6r(cps, -90, config, J1=30, wait=True)
    return False

def sandingModelCTableA(cps=None):
    #Tool1 Side Cycle
    # side_cycles   = 1
    config = load_config()

    config['logger'] = setup_logger(config['settings']['debug'])

    json_config = load_json_config()
    json_config_TableA = json_config['TableA']

    spiral_settings = get_spiral_settings(json_config)

    frame_by_door = get_tableA_task_by_door(json_config_TableA, 'frame')
    zigzag_by_door = get_tableA_task_by_door(json_config_TableA, 'pocketzigzag')

    pocketedge_by_door = get_tableA_task_by_door(json_config_TableA, 'pocketedge')
    tool2side_by_door = get_tableA_task_by_door(json_config_TableA, 'side')
    tool2edge_by_door = get_tableA_task_by_door(json_config_TableA, 'edgeOutside')
    tool3_by_door = get_tableA_task_by_door(json_config_TableA, '3D')

    side_cycles_doors = doors_with_cycles(frame_by_door)
    zig_zag_cycle_doors = doors_with_cycles(zigzag_by_door)

    pocket_edge_cycle_doors = doors_with_cycles(pocketedge_by_door)
    tool2side_cycle_doors = doors_with_cycles(tool2side_by_door)
    tool2sideedge_cycle_doors = doors_with_cycles(tool2edge_by_door)
    tl3sideedge_door = doors_with_cycles(tool3_by_door)

    z, z1, z2 = 10, 0, 10

    #Speed
    speeed = float(json_config['robotSpeed'])

    # Load configuration from YAML
    config = load_config()

    #Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    #Establish connection with robot
    # Establish connection with robot unless caller supplied one.
    owns_cps = cps is None
    if owns_cps:
        cps = CPSClient()
        IP = config["server"]["cpip"]
        port = config["server"]["cps"]
        ret = cps.HRIF_Connect(0, IP, port)
        if ret != 0:
            raise RuntimeError(f"Failed to connect, error code: {ret}")
    else:
        ret = 0

    # --- Adding tool detection code here ---

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
        move_to_task_safe_point(
            cps=cps,
            config=config,
            speed=speeed,
            velocity_profile="robotspeed",
        )

    def move_seventh_to_tool_station():
        seventh_result = communicate(
            cps=cps,
            config=config,
            seventh=0,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.7,
            wait=True,
            require_seventh_ok=True,
        )
        if seventh_result is None:
            raise RuntimeError("Failed to move J7 to the tool station.")
        if not wait_for_j7_idle(cps):
            raise RuntimeError("J7 did not become idle at the tool station.")

    def move_to_homing_with_tool():
        move_to_robot_homing_point(
            cps=cps,
            config=config,
            speed=speeed,
            velocity_profile="robotspeed",
        )

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
            or any_cycles(pocketedge_by_door)
            or any_cycles(tool2edge_by_door)
            or any_cycles(tool2side_by_door)
            or any_cycles(tool3_by_door)
        )
        if has_any_task:
            move_to_safe_point()

        work_executed = False
        keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

        tool4_doors = unique_sorted_doors(side_cycles_doors, zig_zag_cycle_doors)
        tool2_doors = unique_sorted_doors(tool2side_cycle_doors, tool2sideedge_cycle_doors)
        tool1_doors = unique_sorted_doors(tl3sideedge_door)

        has_tool4_batch = any_cycles(frame_by_door) or any_cycles(zigzag_by_door)
        has_tool2_batch = any_cycles(tool2edge_by_door) or any_cycles(tool2side_by_door)
        has_tool1_batch = any_cycles(tool3_by_door)

        tool3_edge_doors = unique_sorted_doors(pocket_edge_cycle_doors)
        has_tool3_batch = len(tool3_edge_doors) > 0

        # Tool 3 batch: pocket edge operation selected from the UI.
        if has_tool3_batch and is_door_available(tool3_edge_doors):
            print("\n=== TOOL 3 POCKET EDGE BATCH START ===")
            ensure_tool_in_hand(3)

            for door_number in tool3_edge_doors:
                edge_cfg = pocketedge_by_door.get(int(door_number), {})
                edge_cycle = int(edge_cfg.get("cycle", 0))
                if edge_cycle <= 0:
                    continue

                edge_force = int(edge_cfg.get("force") or 0)
                if edge_force <= 0:
                    raise RuntimeError(
                        f"Tool 3 pocket edge force is not configured for door {door_number}. "
                        "Set Pocket Edge force to a value > 0."
                    )
                print(f"\n--- Tool 3 / Pocket Edge / Door {door_number} ---")
                run_tool3_pocket_edge_cycles(
                    edge_cycle,
                    edge_force,
                    int(door_number),
                    z,
                    cps,
                    orientation="horizontal",
                    spiral_settings=spiral_settings,
                )
                work_executed = True

            print("\n=== TOOL 3 POCKET EDGE BATCH COMPLETE ===")

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
                    movement = "rectangular_spiral" if bool(zigzag_cfg.get("rectangularSpiral")) else "zigzag"
                    run_zigzag_cycles(
                        zigzag_cycle,
                        int(zigzag_cfg.get("force", 0)),
                        door_number,
                        z,
                        cps,
                        orientation=orientation,
                        movement=movement,
                        spiral_settings=spiral_settings,
                    )
                    work_executed = True
        # Tool 2 batch: side + outside edge
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

                if side_cycle > 0 and edge_cycle > 0:
                    print(f"\n--- Tool 2 / Side+Edge Combined Passes / Door {door_number} ---")
                    run_tool2_side_edge_combined(
                        door_number=door_number,
                        side_force=int(side_cfg.get("force", 0)),
                        side_cycles=side_cycle,
                        edge_force=int(edge_cfg.get("force", 0)),
                        edge_cycles=edge_cycle,
                        cps=cps,
                    )
                    work_executed = True
                else:
                    if side_cycle > 0:
                        print(f"\n--- Tool 2 / Side / Door {door_number} ---")
                        run_tool2side_cycles(
                            side_cycle,
                            int(side_cfg.get("force", 0)),
                            door_number,
                            cps,
                        )
                        work_executed = True

                    if edge_cycle > 0:
                        print(f"\n--- Tool 2 / EdgeOutside / Door {door_number} ---")
                        run_tool2side_edgecycles(
                            edge_cycle,
                            int(edge_cfg.get("force", 0)),
                            door_number,
                            cps,
                        )
                        work_executed = True

                previous_tool2_door = door_number

        # Tool 1 batch: 3D
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
                    move_to_homing_with_tool()
                    move_seventh_to_tool_station()
                    keepTool11(cps, toolNumber=tool_in_hand, config=config)
                    move_to_homing_with_tool()
                else:
                    print("Task completed: no mounted tool to drop.")
        elif has_any_task:
            print("No valid door cycles found to execute.")

    except Exception as e:
        print(f"\nExecution error: {str(e)}")
        traceback.print_exc()
        raise
    finally:
        if owns_cps and cps is not None:
            try:
                cps.HRIF_DisConnect(0)
            except Exception:
                pass
        print("\nSequence terminated")

if __name__ == "__main__":
    sandingModelCTableA()
