import sys
import os
import time
import yaml
import traceback
import json

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from smallTable.modelFzigzag import (
    smalldoor1zizag,
    smalldoor2zizag,
    smalldoor3zizag,
    smalldoor4zizag,
)

from Server_Better_V2 import keepTool11, setup_logger, getTool11, communicate
from modules.CPS import CPSClient

from cycle_data_utils import any_cycles, doors_with_cycles, get_spiral_settings, get_tableA_task_by_door

INTER_PASS_DELAY_SECONDS = 0.0


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    return config


def load_json_config():
    """Loads configuration from config.json."""
    with open("./configs/cycleData.json", "r") as file:
        config = json.load(file)
    return config


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
    if count <= 0:
        return
    door_funcs = {
        1: smalldoor1zizag,
        2: smalldoor2zizag,
        3: smalldoor3zizag,
        4: smalldoor4zizag,
    }
    try:
        door_func = door_funcs[door_num]
    except KeyError:
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== ZIGZAG CYCLE {i + 1}/{count} (Door {door_num}) ===")
        door_func(
            force=force,
            z=z,
            cps=cps,
            orientation=orientation,
            movement=movement,
            spiral_settings=spiral_settings,
        )
        if i < count - 1 and INTER_PASS_DELAY_SECONDS > 0:
            print(f"Pausing {INTER_PASS_DELAY_SECONDS:.2f} seconds...")
            time.sleep(INTER_PASS_DELAY_SECONDS)


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
    keepTool11(cps, toolNumber=tool_in_hand, config=config)
    return False


def sandingModelFTableA():
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    json_config = load_json_config()
    json_config_TableA = json_config["TableA"]

    spiral_settings = get_spiral_settings(json_config)

    zigzag_by_door = get_tableA_task_by_door(json_config_TableA, "pocketzigzag")
    zig_zag_cycle_doors = doors_with_cycles(zigzag_by_door)
    ignored_tasks = {
        "frame": get_tableA_task_by_door(json_config_TableA, "frame"),
        "pocketsquare": get_tableA_task_by_door(json_config_TableA, "pocketsquare"),
        "3D": get_tableA_task_by_door(json_config_TableA, "3D"),
        "edgeInside": get_tableA_task_by_door(json_config_TableA, "edgeInside"),
        "edgeOutside": get_tableA_task_by_door(json_config_TableA, "edgeOutside"),
        "side": get_tableA_task_by_door(json_config_TableA, "side"),
    }

    z = -4
    speeed = float(json_config["robotSpeed"])
    keep_tool_after_task = bool(config.get("settings", {}).get("keepToolAfterTask", True))

    cps = CPSClient()
    IP = config["server"]["cpip"]
    port = config["server"]["cps"]
    ret = cps.HRIF_Connect(0, IP, port)

    if ret != 0:
        print(f"Failed to connect, error code: {ret}")
        exit()

    def read_ci_bit(cps, bit_index):
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

    try:
        for task_name, task_by_door in ignored_tasks.items():
            if any_cycles(task_by_door):
                print(
                    f"[ModelF] Task '{task_name}' is configured but ignored. "
                    "Model F runs Tool4 zigzag only."
                )

        if not any_cycles(zigzag_by_door):
            print("No zigzag cycles configured for Model F.")
            return

        if zig_zag_cycle_doors:
            ci0, ci1, ci2 = read_ci_triplet(cps)
            has_tool4 = check_tool(
                cps=cps, config=config, tool_num=4, ci0=ci0, ci1=ci1, ci2=ci2
            )

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

            if not has_tool4:
                getTool11(cps, toolNumber=4, config=config)
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

            for door_number in zig_zag_cycle_doors:
                cfg = zigzag_by_door.get(int(door_number), {})
                orientation = str(cfg.get("orientation") or "vertical").lower()
                run_zigzag_cycles(
                    int(cfg.get("cycle", 0)),
                    int(cfg.get("force", 0)),
                    int(door_number),
                    z,
                    cps,
                    orientation=orientation,
                    movement="zigzag",
                    spiral_settings=spiral_settings,
                )

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

            if keep_tool_after_task:
                print("Task completed: keeping Tool 4 mounted.")
            else:
                communicate(
                    cps=cps,
                    config=config,
                    seventh=0,
                    tcp=config["coords"]["tcptool4plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    speed=0.3,
                    wait=True,
                )
                keepTool11(cps, toolNumber=4, config=config)
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

            print("\nModel F zigzag operations completed successfully!")

    except Exception as e:
        print(f"\nExecution error: {str(e)}")
        traceback.print_exc()
    finally:
        print("\nSequence terminated")


if __name__ == "__main__":
    sandingModelFTableA()
