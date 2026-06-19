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


def _decode_tool_from_ci(ci0, ci1, ci2):
    if any(val is None for val in (ci0, ci1, ci2)):
        return None
    if ci0 == 0 and ci1 == 0 and ci2 == 0:
        return 0
    if ci0 == 0 and ci1 == 1 and ci2 == 1:
        return 1
    if ci0 == 0 and ci1 == 1 and ci2 == 0:
        return 2
    if ci0 == 1 and ci1 == 0 and ci2 == 0:
        return 3
    if ci0 == 0 and ci1 == 0 and ci2 == 1:
        return 4
    return -1


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


def sandingModelFTableA(cps=None):
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

    def wait_tool4_in_hand(timeout_s=3.0, poll_s=0.1):
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while time.monotonic() < deadline:
            ci0_local, ci1_local, ci2_local = read_ci_triplet(cps)
            tool = _decode_tool_from_ci(ci0_local, ci1_local, ci2_local)
            if tool == 4:
                return True
            time.sleep(max(0.02, float(poll_s)))
        return False

    try:
        executed_doors = 0
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
                if not wait_tool4_in_hand(timeout_s=3.0, poll_s=0.1):
                    raise RuntimeError("Tool 4 was not confirmed in hand after pick.")
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
                orientation = str(cfg.get("orientation") or "vertical").strip().lower()
                cycle_count = int(cfg.get("cycle", 0))
                force_value = int(cfg.get("force", 0))
                print(
                    f"[ModelF] Door {door_number}: orientation='{orientation}', "
                    f"cycle={cycle_count}, force={force_value}"
                )
                if cycle_count <= 0:
                    continue
                try:
                    run_zigzag_cycles(
                        cycle_count,
                        force_value,
                        int(door_number),
                        z,
                        cps,
                        orientation=orientation,
                        movement="zigzag",
                        spiral_settings=spiral_settings,
                    )
                except Exception as exc:
                    # First pass can fail transiently if force seek starts before
                    # pneumatics/tool settle. Reposition and retry once.
                    print(
                        f"[ModelF] Door {door_number} first attempt failed: {exc}. Retrying once..."
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
                    time.sleep(0.35)
                    run_zigzag_cycles(
                        cycle_count,
                        force_value,
                        int(door_number),
                        z,
                        cps,
                        orientation=orientation,
                        movement="zigzag",
                        spiral_settings=spiral_settings,
                    )
                executed_doors += 1

            if executed_doors <= 0:
                raise RuntimeError(
                    "Model F task finished with zero executed doors. "
                    "Check cycle/door selection payload."
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
        # Propagate failure so parent process does not look like a clean completion.
        raise
    finally:
        if owns_cps and cps is not None:
            try:
                cps.HRIF_DisConnect(0)
            except Exception:
                pass
        print("\nSequence terminated")


if __name__ == "__main__":
    sandingModelFTableA()
