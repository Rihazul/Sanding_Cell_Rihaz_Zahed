import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Server_Better_V2 import (
    communicate,
    setup_logger,
    moveOnlyJ6r,
    putForceYplus1,
    putForceXminus,
    putForceYminus1,
    putForceXplus,
    stop_requested,
)
from smallTable.frame1tool2sidefinal import (
    _run_tool2_side_process,
    load_config,
    load_json_config,
)
from smallTable.frame1tool2edgefinal import _run_tool2_edge_process
from smallTable.scancord import get_door_position, get_outer_corner_point, get_y_values


def _move_robot(cps, config, point, robot_speed):
    communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config["coords"]["tcptool2plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )


def _move_j7(cps, config, seventh, robot_speed, *, transition=False):
    communicate(
        cps=cps,
        config=config,
        seventh=seventh,
        tcp=config["coords"]["tcptool2plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=False,
        require_seventh_ok=True,
        require_seventh_run_transition=transition,
    )


def _section(points, force_point, force_func, vibration_point):
    return {
        "points": points,
        "force_point": force_point,
        "force_func": force_func,
        "vibration_point": vibration_point,
    }


def _run_side_section(cps, config, section, force, cycles, sanding_speed):
    if cycles <= 0:
        return
    _run_tool2_side_process(
        cps,
        config,
        section["points"],
        force,
        sanding_speed,
        tcp=config["coords"]["tcptool2plane1"],
        ucs=config["coords"]["ucsTable1"],
        force_point=section["force_point"],
        force_func=section["force_func"],
        vibration_point=section["vibration_point"],
        cycles=cycles,
    )


def _run_edge_section(cps, config, section, force, cycles, sanding_speed):
    if cycles <= 0:
        return
    _run_tool2_edge_process(
        cps,
        config,
        section["points"],
        force,
        sanding_speed,
        tcp=config["coords"]["tcptool2plane1"],
        ucs=config["coords"]["ucsTable1"],
        force_point=section["force_point"],
        force_func=section["force_func"],
        vibration_point=section["vibration_point"],
        cycles=cycles,
    )


def _run_combined_section(
    name,
    cps,
    config,
    side_section,
    edge_section,
    side_force,
    side_cycles,
    edge_force,
    edge_cycles,
    sanding_speed,
):
    print(f"\n--- Tool 2 combined pass: {name} side then edge ---")
    _run_side_section(cps, config, side_section, side_force, side_cycles, sanding_speed)
    if stop_requested():
        raise RuntimeError("[Tool 2 Combined] Stop requested after side pass.")
    _run_edge_section(cps, config, edge_section, edge_force, edge_cycles, sanding_speed)
    if stop_requested():
        raise RuntimeError("[Tool 2 Combined] Stop requested after edge pass.")


def _get_points(door_num):
    points = [get_outer_corner_point(door_num, idx) for idx in range(4)]
    if not all(points):
        raise RuntimeError(f"Tool 2 combined cannot run Door {door_num}: missing outer corner points.")
    return points


def _build_small_sections(door_num):
    p1, p2, p3, p4 = _get_points(door_num)

    # Side tool contact points.
    side_right_1 = [p1[0] - 3, p1[1], 5, 0, 0, 0]
    side_right_pre_1 = [p1[0] - 10, p1[1], 5, 0, 0, 0]
    side_right_2 = [p2[0] - 3, p2[1], 5, 0, 0, 0]
    side_right_pre_2 = [p2[0] - 10, p2[1], 5, 0, 0, 0]
    side_right_12 = [p1[0] - 3, p1[1] + 2, 5, 0, 0, 0]

    side_top_1 = [p2[0], p2[1] + 4, 5, 0, 0, -90]
    side_top_pre_1 = [p2[0], p2[1] + 10, 5, 0, 0, -90]
    side_top_2 = [p3[0], p3[1] + 4, 5, 0, 0, -90]
    side_top_pre_2 = [p3[0], p3[1] + 10, 5, 0, 0, -90]
    side_top_12 = [p2[0] + 2, p2[1] + 4, 5, 0, 0, -90]

    side_left_1 = [p3[0] + 4, p3[1], 5, 0, 0, -180]
    side_left_pre_1 = [p3[0] + 11, p3[1], 5, 0, 0, -180]
    side_left_2 = [p4[0] + 4, p4[1], 5, 0, 0, -180]
    side_left_pre_2 = [p4[0] + 11, p4[1], 5, 0, 0, -180]
    side_left_12 = [p3[0] + 4, p3[1] - 2, 5, 0, 0, -180]

    side_bottom_1 = [p4[0], p4[1] - 12, 5, 0, 0, 90]
    side_bottom_pre_1 = [p4[0], p4[1] - 12, 5, 0, 0, 90]
    side_bottom_2 = [p1[0], p1[1] - 12, 5, 0, 0, 90]
    side_bottom_pre_2 = [p1[0], p1[1] - 12, 5, 0, 0, 90]
    side_bottom_12 = [p4[0], p4[1] - 12, 5, 0, 0, 90]

    # Edge tool contact points.
    edge_right_1 = [p1[0] - 5, p1[1], 10, 0, 22, 0]
    edge_right_pre_1 = [p1[0] - 5, p1[1], 10, 0, 22, 0]
    edge_right_2 = [p2[0] - 5, p2[1], 10, 0, 22, 0]
    edge_right_pre_2 = [p2[0] - 5, p2[1], 10, 0, 22, 0]
    edge_right_12 = [p1[0] - 5, p1[1] + 2, 10, 0, 22, 0]

    edge_top_1 = [p2[0], p2[1] - 5, 10, 0, 22, -90]
    edge_top_pre_1 = [p2[0], p2[1] - 5, 10, 0, 22, -90]
    edge_top_2 = [p3[0], p3[1] - 5, 10, 0, 22, -90]
    edge_top_pre_2 = [p3[0], p3[1] - 5, 10, 0, 22, -90]
    edge_top_12 = [p2[0] + 2, p2[1] - 5, 10, 0, 22, -90]

    edge_left_1 = [p3[0] + 5, p3[1], 10, 0, 22, -180]
    edge_left_pre_1 = [p3[0] + 5, p3[1], 10, 0, 22, -180]
    edge_left_2 = [p4[0] + 5, p4[1], 10, 0, 22, -180]
    edge_left_pre_2 = [p4[0] + 5, p4[1], 10, 0, 22, -180]
    edge_left_12 = [p3[0] + 5, p3[1] - 2, 10, 0, 22, -180]

    edge_bottom_1 = [p4[0], p4[1] - 2, 10, 0, 22, 90]
    edge_bottom_pre_1 = [p4[0], p4[1] - 2, 10, 0, 22, 90]
    edge_bottom_2 = [p1[0], p1[1] - 2, 10, 0, 22, 90]
    edge_bottom_pre_2 = [p1[0], p1[1] - 2, 10, 0, 22, 90]
    edge_bottom_12 = [p4[0] - 2, p4[1] - 2, 10, 0, 22, 90]

    side = {
        "right": _section([side_right_pre_1, side_right_1, side_right_12, side_right_2, side_right_pre_2], side_right_12, putForceXplus, side_right_2),
        "top": _section([side_top_pre_1, side_top_1, side_top_12, side_top_2, side_top_pre_2], side_top_12, putForceYminus1, side_top_2),
        "left": _section([side_left_pre_1, side_left_1, side_left_12, side_left_2, side_left_pre_2], side_left_12, putForceXminus, side_left_2),
        "bottom": _section([side_bottom_pre_1, side_bottom_1, side_bottom_12, side_bottom_2, side_bottom_pre_2], side_bottom_12, putForceYplus1, side_bottom_2),
    }
    edge = {
        "right": _section([edge_right_pre_1, edge_right_1, edge_right_12, edge_right_2, edge_right_pre_2], edge_right_12, putForceXplus, edge_right_2),
        "top": _section([edge_top_pre_1, edge_top_1, edge_top_12, edge_top_2, edge_top_pre_2], edge_top_12, putForceYminus1, edge_top_2),
        "left": _section([edge_left_pre_1, edge_left_1, edge_left_12, edge_left_2, edge_left_pre_2], edge_left_12, putForceXminus, edge_left_2),
        "bottom": _section([edge_bottom_pre_1, edge_bottom_1, edge_bottom_12, edge_bottom_2, edge_bottom_pre_2], edge_bottom_12, putForceYplus1, edge_bottom_2),
    }
    transitions = {
        "prehoming": [0, 0, 100, 0, 0, 0],
        "after_right": [p2[0] - 10, p2[1] + 10, 80, 0, 0, 0],
        "after_top": [p3[0] + 10, p3[1] + 10, 80, 0, 0, -90],
        "after_left": [p4[0] + 20, p4[1] - 30, 85, 0, 0, -180],
        "posthoming": [p1[0], p1[1] - 10, 100, 0, 0, 90],
    }
    return side, edge, transitions


def _build_big_sections(door_num):
    p1, p2, p3, p4 = _get_points(door_num)
    distance = (p4[0] - p1[0]) / 2

    def build(kind):
        is_edge = kind == "edge"
        z = 10 if is_edge else 5
        ry = 22 if is_edge else 0
        right_x = p1[0] - 4
        right_pre_x = p1[0] - 10
        upright_x = -distance * 2 if is_edge else -distance * 2 - 4
        upright_pre_x = -distance * 2 if is_edge else -distance * 2 - 2
        top_y = p2[1] + 4
        uptop_y = p2[1] - 5 if is_edge else p2[1] + 4
        uptop_pre_y = p2[1] - 5 if is_edge else p2[1] + 10
        left_x = p3[0] + 5
        left_pre_x = p3[0] + 10
        upleft_x = p1[0] + 3
        upleft_pre_x = p1[0] + 10
        bottom_y = p4[1] - 2 if is_edge else p4[1] - 12
        bottom_x_offset = -2 if is_edge else -2

        right_1 = [right_x, p1[1], z, 0, ry, 0]
        right_pre_1 = [right_pre_x, p1[1], z, 0, ry, 0]
        right_4 = [p2[0] - 4, p2[1] / 2, z, 0, ry, 0]
        right_4_pre = [p2[0] - 10, p2[1] / 2, z, 0, ry, 0]
        right_14 = [right_x, p1[1] + 2, z, 0, ry, 0]

        upright_4 = [upright_x, p2[1] / 2, z, 0, ry, 0]
        upright_4_pre = [upright_pre_x, p2[1] / 2, z, 0, ry, 0]
        upright_4_up = [upright_pre_x, p2[1] / 2 + 2, z, 0, ry, 0]
        upright_2 = [upright_pre_x, p2[1], z, 0, ry, 0]
        upright_2_pre = [upright_pre_x, p2[1], z, 0, ry, 0]

        topup_1 = [-distance * 2, uptop_y, z, 0, ry, -90]
        topup_pre_1 = [-distance * 2, uptop_pre_y, z, 0, ry, -90]
        topup_12 = [-distance * 2 + 2, uptop_y, z, 0, ry, -90]
        topup_2 = [p1[0], p3[1] - 5 if is_edge else p3[1] + 4, z, 0, ry, -90]
        topup_pre_2 = [p1[0], p3[1] - 5 if is_edge else p3[1] + 10, z, 0, ry, -90]

        left_5 = [left_x, p3[1] / 2, z, 0, ry, -180]
        left_5_down = [left_x, p3[1] / 2 - 2, z, 0, ry, -180]
        left_5_pre = [left_pre_x, p3[1] / 2, z, 0, ry, -180]
        left_2 = [p4[0] + 5, p4[1], z, 0, ry, -180]
        left_pre_2 = [p4[0] + 11, p4[1], z, 0, ry, -180]

        upleft_3 = [upleft_x, p3[1], z, 0, ry, -180]
        upleft_3_pre = [upleft_pre_x, p3[1], z, 0, ry, -180]
        upleft_3_down = [upleft_x, p3[1] - 2, z, 0, ry, -180]
        upleft_5 = [upleft_x, p3[1] / 2, z, 0, ry, -180]
        upleft_5_pre = [upleft_pre_x, p3[1] / 2, z, 0, ry, -180]

        bottom_1 = [p4[0], bottom_y, z, 0, ry, 90]
        bottom_pre_1 = [p4[0], bottom_y, z, 0, ry, 90]
        bottom_2 = [p1[0], p1[1] - 2 if is_edge else p1[1] - 12, z, 0, ry, 90]
        bottom_pre_2 = [p1[0], p1[1] - 2 if is_edge else p1[1] - 12, z, 0, ry, 90]
        bottom_12 = [p4[0] + bottom_x_offset, bottom_y, z, 0, ry, 90]

        return {
            "left_down": _section([left_5_pre, left_5, left_5_down, left_2, left_pre_2], left_5_down, putForceXminus, left_2),
            "bottom": _section([bottom_pre_1, bottom_1, bottom_12, bottom_2, bottom_pre_2], bottom_12, putForceYplus1, bottom_2),
            "right_down": _section([right_pre_1, right_1, right_14, right_4, right_4_pre], right_14, putForceXplus, right_4),
            "upright": _section([upright_4_pre, upright_4, upright_4_up, upright_2, upright_2_pre], upright_4_up, putForceXplus, upright_2),
            "topup": _section([topup_pre_1, topup_1, topup_12, topup_2, topup_pre_2], topup_12, putForceYminus1, topup_2),
            "upleft": _section([upleft_3_pre, upleft_3, upleft_3_down, upleft_5, upleft_5_pre], upleft_3_down, putForceXminus, upleft_5),
        }

    side = build("side")
    edge = build("edge")
    transitions = {
        "prehoming": [(p4[0] - p1[0]) + 10, p3[1] / 2, 100, 0, 0, -180],
        "after_left_down": [p4[0] + 20, p4[1] - 30, 85, 0, 0, -180],
        "after_bottom": [p1[0] - 10, p1[1] - 32, 80, 0, 0, 0],
        "after_right_down": [p2[0] - 10, p2[1] / 2, 80, 0, 0, 0],
        "prehome_upright": [(-distance * 2) - 10, p2[1] / 2, 120, 0, 0, 0],
        "after_upright": [(-distance * 2) - 10, p2[1] + 30, 80, 0, 0, -90],
        "after_topup": [p1[0] + 10, p3[1] + 20, 80, 0, 0, -180],
        "after_upleft": [p1[0] + 10, p3[1] / 2, 100, 0, 0, -180],
    }
    return side, edge, transitions


def _run_small_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps, config, robot_speed, sanding_speed):
    side, edge, transitions = _build_small_sections(door_num)
    x1 = get_door_position(door_num)

    _move_j7(cps, config, x1, robot_speed)
    _move_robot(cps, config, transitions["prehoming"], robot_speed)

    for name, transition_name in (
        ("right", "after_right"),
        ("top", "after_top"),
        ("left", "after_left"),
        ("bottom", "posthoming"),
    ):
        _run_combined_section(name, cps, config, side[name], edge[name], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
        if name == "left":
            _move_robot(cps, config, transitions[transition_name], robot_speed)
            moveOnlyJ6r(cps, -270, config, wait=True)
        else:
            _move_robot(cps, config, transitions[transition_name], robot_speed)


def _run_big_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps, config, robot_speed, sanding_speed):
    side, edge, transitions = _build_big_sections(door_num)
    p1, _, _, p4 = _get_points(door_num)
    x1 = get_door_position(door_num)
    x2 = x1 + (p4[0] - p1[0])

    _move_j7(cps, config, x1, robot_speed)
    _move_robot(cps, config, transitions["prehoming"], robot_speed)

    _run_combined_section("left lower", cps, config, side["left_down"], edge["left_down"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_left_down"], robot_speed)
    moveOnlyJ6r(cps, -270, config, wait=True)

    _run_combined_section("bottom", cps, config, side["bottom"], edge["bottom"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_bottom"], robot_speed)

    _run_combined_section("right lower", cps, config, side["right_down"], edge["right_down"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_right_down"], robot_speed)

    _move_j7(cps, config, x2, robot_speed, transition=True)
    _move_robot(cps, config, transitions["prehome_upright"], robot_speed)

    _run_combined_section("right upper", cps, config, side["upright"], edge["upright"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_upright"], robot_speed)

    _run_combined_section("top", cps, config, side["topup"], edge["topup"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_topup"], robot_speed)

    _run_combined_section("left upper", cps, config, side["upleft"], edge["upleft"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed)
    _move_robot(cps, config, transitions["after_upleft"], robot_speed)


def run_tool2_side_edge_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps):
    side_cycles = max(0, int(side_cycles))
    edge_cycles = max(0, int(edge_cycles))
    if side_cycles <= 0 and edge_cycles <= 0:
        return

    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    json_config = load_json_config()
    robot_speed = float(json_config.get("robotSpeed", 0.9))
    sanding_speed = float(json_config.get("sandingSpeed", 0.75))

    ylen_data = get_y_values(door_num, default_on_error=True)
    ylen = ylen_data["ylen"]
    if ylen == "null" or not isinstance(ylen, (int, float)):
        raise RuntimeError(
            f"Tool 2 combined cannot run Door {door_num}: invalid scanned ylen value {ylen!r}."
        )

    print(
        f"\n=== Tool 2 combined side+edge Door {door_num}: "
        f"side cycles={side_cycles}, edge cycles={edge_cycles}, ylen={ylen} ==="
    )
    if ylen > 600:
        _run_big_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps, config, robot_speed, sanding_speed)
    else:
        _run_small_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps, config, robot_speed, sanding_speed)
