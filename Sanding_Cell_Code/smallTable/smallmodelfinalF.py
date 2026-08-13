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
# Tool 2 side / edge-outside passes (same functions Model E / smallmodelfinal5 uses). Model F
# (UI "Model E - Flat") now also runs Tool 2 on the door sides in addition to its Tool 4 zigzag.
from smallTable.frame1tool2sidefinal import (
    door1frametool2side,
    door2frametool2side,
    door3frametool2side,
    door4frametool2side,
)
from smallTable.frame1tool2edgefinal import (
    door1frametool2sideedge,
    door2frametool2sideedge,
    door3frametool2sideedge,
    door4frametool2sideedge,
)
from smallTable.frame1tool2sideedge_combined import run_tool2_side_edge_combined

from Server_Better_V2 import keepTool11, setup_logger, getTool11, communicate, move_to_task_safe_point, move_to_robot_homing_point, stop_requested
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


def run_tool2side_cycles(count, force, door_num, cps):
    """Run Tool 2 SIDE passes for one door (same functions as Model E)."""
    if count <= 0:
        return
    door_funcs = {
        1: door1frametool2side,
        2: door2frametool2side,
        3: door3frametool2side,
        4: door4frametool2side,
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


def run_tool2side_edgecycles(count, force, door_num, cps):
    """Run Tool 2 EDGE-OUTSIDE passes for one door (same functions as Model E)."""
    if count <= 0:
        return
    door_funcs = {
        1: door1frametool2sideedge,
        2: door2frametool2sideedge,
        3: door3frametool2sideedge,
        4: door4frametool2sideedge,
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


def is_door_available(doors):
    return bool(any(doors))


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
    # Tool 2 side / edge-outside are RUN for Model F (in addition to the Tool 4 zigzag).
    tool2side_by_door = get_tableA_task_by_door(json_config_TableA, "side")
    tool2edge_by_door = get_tableA_task_by_door(json_config_TableA, "edgeOutside")
    tool2side_cycle_doors = doors_with_cycles(tool2side_by_door)
    tool2edge_cycle_doors = doors_with_cycles(tool2edge_by_door)
    tool2_doors = unique_sorted_doors(tool2side_cycle_doors, tool2edge_cycle_doors)
    has_tool2_batch = any_cycles(tool2side_by_door) or any_cycles(tool2edge_by_door)
    # These operations still have no geometry on a flat (Model F) door and are ignored.
    ignored_tasks = {
        "frame": get_tableA_task_by_door(json_config_TableA, "frame"),
        "pocketsquare": get_tableA_task_by_door(json_config_TableA, "pocketsquare"),
        "3D": get_tableA_task_by_door(json_config_TableA, "3D"),
        "edgeInside": get_tableA_task_by_door(json_config_TableA, "edgeInside"),
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

    def wait_tool_in_hand(tool_num, timeout_s=3.0, poll_s=0.1):
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while time.monotonic() < deadline:
            ci0_local, ci1_local, ci2_local = read_ci_triplet(cps)
            if _decode_tool_from_ci(ci0_local, ci1_local, ci2_local) == tool_num:
                return True
            time.sleep(max(0.02, float(poll_s)))
        return False

    _TOOL_PLANE1_TCP = {
        1: "tcptool1plane1",
        2: "tcptool2plane1",
        3: "tcptool3plane1",
        4: "tcptool4plane1",
    }

    def move_to_homing_position():
        move_to_robot_homing_point(
            cps=cps,
            config=config,
            speed=speeed,
            velocity_profile="robotspeed",
        )

    def drop_current_tool_at_station():
        """Drop whatever tool is currently in hand, at its tool station.

        getTool11 refuses to pick when a tool is already held, so the hand must be empty
        before a pick. Dropping must happen at the tool station with J7 there first (not
        wherever a sanding pass ended), which is why this moves J7 to the held tool's plane1
        station before keepTool11 — the same safe sequence the Tool 4 drop uses.
        """
        ci0_local, ci1_local, ci2_local = read_ci_triplet(cps)
        held = _decode_tool_from_ci(ci0_local, ci1_local, ci2_local)
        if not held or held <= 0:
            return
        tcp_key = _TOOL_PLANE1_TCP.get(held)
        if tcp_key is None:
            return
        move_to_homing_position()
        communicate(
            cps=cps,
            config=config,
            seventh=0,
            tcp=config["coords"][tcp_key],
            ucs=config["coords"]["ucsTable1"],
            speed=0.3,
            wait=True,
        )
        keepTool11(cps, toolNumber=held, config=config)
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

    def ensure_tool_ready(tool_num):
        """Guarantee `tool_num` is in hand via a safe drop-at-station then pick.

        Mirrors smallmodelfinal5's ensure_tool_in_hand: if the requested tool is already
        held, do nothing; otherwise drop the currently-held tool at its station (empty hand)
        and pick the requested tool from the safe point.
        """
        ci0_local, ci1_local, ci2_local = read_ci_triplet(cps)
        held = _decode_tool_from_ci(ci0_local, ci1_local, ci2_local)
        if held == tool_num:
            return
        if held and held > 0:
            drop_current_tool_at_station()
        move_to_homing_position()
        getTool11(cps, toolNumber=tool_num, config=config)
        if not wait_tool_in_hand(tool_num, timeout_s=3.0, poll_s=0.1):
            raise RuntimeError(f"Tool {tool_num} was not confirmed in hand after pick.")
        move_to_task_safe_point(
            cps=cps, config=config, speed=speeed, velocity_profile="robotspeed",
        )

    try:
        executed_doors = 0
        for task_name, task_by_door in ignored_tasks.items():
            if any_cycles(task_by_door):
                print(
                    f"[ModelF] Task '{task_name}' is configured but ignored on a flat door."
                )

        if not any_cycles(zigzag_by_door) and not has_tool2_batch:
            print("No zigzag or Tool 2 cycles configured for Model F.")
            return

        if zig_zag_cycle_doors:
            # Safe drop-at-station then pick (handles a leftover Tool 1/2/3 from a prior run).
            ensure_tool_ready(4)

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

            move_to_homing_position()

            # Tool 4 disposition after zigzag:
            #  • If the Tool 2 batch follows, leave Tool 4 mounted here — ensure_tool_ready(2)
            #    performs the safe drop-at-station of Tool 4 and the Tool 2 pick as one flow,
            #    so we must NOT drop it twice.
            #  • Otherwise honour keepToolAfterTask (keep mounted, or drop Tool 4 at station).
            if has_tool2_batch:
                print("Tool 2 batch follows: Tool 4 will be dropped during the tool change.")
            elif keep_tool_after_task:
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

        # --- Tool 2 batch (side / edge outside) ------------------------------------------
        # Runs after the Tool 4 zigzag. Picks Tool 2, then for each configured door runs the
        # side and/or edge passes (combined when both are set), with a safe transition between
        # doors. Same functions and ordering Model E (smallmodelfinal5) uses.
        if has_tool2_batch and is_door_available(tool2_doors):
            # Safe drop-at-station then pick — never drops a tool wherever a pass ended.
            ensure_tool_ready(2)

            previous_tool2_door = None
            for door_number in tool2_doors:
                side_cfg = tool2side_by_door.get(int(door_number), {})
                side_cycle = int(side_cfg.get("cycle", 0))
                edge_cfg = tool2edge_by_door.get(int(door_number), {})
                edge_cycle = int(edge_cfg.get("cycle", 0))
                if side_cycle <= 0 and edge_cycle <= 0:
                    continue

                if previous_tool2_door is not None:
                    print(
                        f"\n--- Tool 2 / Door-to-Door Safe Transition / "
                        f"Door {previous_tool2_door} to Door {door_number} ---"
                    )
                    move_to_task_safe_point(
                        cps=cps, config=config, speed=speeed, velocity_profile="robotspeed",
                    )

                if side_cycle > 0 and edge_cycle > 0:
                    print(f"\n--- Tool 2 / Side+Edge Combined / Door {door_number} ---")
                    run_tool2_side_edge_combined(
                        door_number=int(door_number),
                        side_force=int(side_cfg.get("force", 0)),
                        side_cycles=side_cycle,
                        edge_force=int(edge_cfg.get("force", 0)),
                        edge_cycles=edge_cycle,
                        cps=cps,
                    )
                    executed_doors += 1
                else:
                    if side_cycle > 0:
                        print(f"\n--- Tool 2 / Side / Door {door_number} ---")
                        run_tool2side_cycles(
                            side_cycle, int(side_cfg.get("force", 0)), int(door_number), cps,
                        )
                        executed_doors += 1
                    if edge_cycle > 0:
                        print(f"\n--- Tool 2 / EdgeOutside / Door {door_number} ---")
                        run_tool2side_edgecycles(
                            edge_cycle, int(edge_cfg.get("force", 0)), int(door_number), cps,
                        )
                        executed_doors += 1

                previous_tool2_door = door_number

            move_to_homing_position()
            if not keep_tool_after_task:
                # Drop Tool 2 at its station (J7 to station first), same safe sequence as the
                # tool change, rather than dropping wherever the last pass ended.
                drop_current_tool_at_station()
            else:
                print("Task completed: keeping Tool 2 mounted.")
            print("\nModel F Tool 2 side/edge operations completed successfully!")

        if executed_doors <= 0:
            raise RuntimeError(
                "Model F task finished with zero executed doors. "
                "Check cycle/door selection payload."
            )

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
