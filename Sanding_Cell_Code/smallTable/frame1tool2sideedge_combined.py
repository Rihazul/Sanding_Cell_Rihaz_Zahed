import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Server_Better_V2 import (
    waitForSeventhAxisIdle,
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


def _move_robot(cps, config, point, robot_speed, *, wait_for_seventh=True):
    """Move the 6-axis arm.

    wait_for_seventh=False lets the arm travel WHILE the 7th axis is still
    moving, which is what keeps Table B smooth. Only safe for poses that are
    clear of the door for the whole rail stroke; the final approach to a
    prepoint must still wait.
    """
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
        require_seventh_ok=wait_for_seventh,
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


SAFETY_LIFT_Z_MM = 85


def _final_prepoint_for_cycles(section, cycles):
    points = section["points"]
    if not points:
        return None
    cycle_count = max(1, int(cycles))
    return points[-1] if cycle_count % 2 == 1 else points[0]


def _lift_from_section_endpoint(cps, config, section, cycles, robot_speed):
    endpoint = _final_prepoint_for_cycles(section, cycles)
    if endpoint is None:
        return
    lift_point = list(endpoint)
    lift_point[2] = max(float(lift_point[2]), SAFETY_LIFT_Z_MM)
    print(f"[Tool 2 Combined] Safety lift after edge pass to {lift_point}")
    _move_robot(cps, config, lift_point, robot_speed)


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
    robot_speed,
):
    print(f"\n--- Tool 2 combined pass: {name} side then edge ---")
    _run_side_section(cps, config, side_section, side_force, side_cycles, sanding_speed)
    if stop_requested():
        raise RuntimeError("[Tool 2 Combined] Stop requested after side pass.")
    _run_edge_section(cps, config, edge_section, edge_force, edge_cycles, sanding_speed)
    if stop_requested():
        raise RuntimeError("[Tool 2 Combined] Stop requested after edge pass.")
    _lift_from_section_endpoint(cps, config, edge_section, edge_cycles, robot_speed)


# Physical 7th-axis travel limit. A big door whose bottom station is already far
# along the rail cannot jump a full door length for the top half without
# exceeding this, so the top station is clamped and the top-half X points are
# shifted by the travel that was actually available.
TOOL2_J7_MAX_MM = 2310.0

# Prepoint standoff from the sanded face. Force search starts at the prepoint and
# closes this gap, so it has to be big enough to clear the surface but no bigger
# than necessary -- an oversized gap makes the search crawl.
#   side  (RY=0)  -> 15 mm
#   edge  (RY=22) ->  5 mm
TOOL2_SIDE_PREPOINT_MM = 15.0
TOOL2_EDGE_PREPOINT_MM = 5.0


def _prepoint_offset(is_edge: bool) -> float:
    return TOOL2_EDGE_PREPOINT_MM if is_edge else TOOL2_SIDE_PREPOINT_MM


def _get_points(door_num):
    points = [get_outer_corner_point(door_num, idx) for idx in range(4)]
    if not all(points):
        raise RuntimeError(f"Tool 2 combined cannot run Door {door_num}: missing outer corner points.")
    return points


def _build_small_sections(door_num):
    p1, p2, p3, p4 = _get_points(door_num)
    # Prepoints stand off the face by a fixed amount along that side's own normal
    # so force search always starts from a known gap. Several of these used to be
    # identical to their contact point (no standoff at all), which left the search
    # to find contact from zero distance.
    sp = TOOL2_SIDE_PREPOINT_MM
    ep = TOOL2_EDGE_PREPOINT_MM

    # Side tool contact points (RY=0).
    side_right_1 = [p1[0] - 3, p1[1], 5, 0, 0, 0]
    side_right_pre_1 = [p1[0] - 3 - sp, p1[1], 5, 0, 0, 0]
    side_right_2 = [p2[0] - 3, p2[1], 5, 0, 0, 0]
    side_right_pre_2 = [p2[0] - 3 - sp, p2[1], 5, 0, 0, 0]
    side_right_12 = [p1[0] - 3, p1[1] + 2, 5, 0, 0, 0]

    side_top_1 = [p2[0], p2[1] + 4, 5, 0, 0, -90]
    side_top_pre_1 = [p2[0], p2[1] + 4 + sp, 5, 0, 0, -90]
    side_top_2 = [p3[0], p3[1] + 4, 5, 0, 0, -90]
    side_top_pre_2 = [p3[0], p3[1] + 4 + sp, 5, 0, 0, -90]
    side_top_12 = [p2[0] + 2, p2[1] + 4, 5, 0, 0, -90]

    side_left_1 = [p3[0] + 4, p3[1], 5, 0, 0, -180]
    side_left_pre_1 = [p3[0] + 4 + sp, p3[1], 5, 0, 0, -180]
    side_left_2 = [p4[0] + 4, p4[1], 5, 0, 0, -180]
    side_left_pre_2 = [p4[0] + 4 + sp, p4[1], 5, 0, 0, -180]
    side_left_12 = [p3[0] + 4, p3[1] - 2, 5, 0, 0, -180]

    side_bottom_1 = [p4[0], p4[1] - 12, 5, 0, 0, 90]
    side_bottom_pre_1 = [p4[0], p4[1] - 12 - sp, 5, 0, 0, 90]
    side_bottom_2 = [p1[0], p1[1] - 12, 5, 0, 0, 90]
    side_bottom_pre_2 = [p1[0], p1[1] - 12 - sp, 5, 0, 0, 90]
    side_bottom_12 = [p4[0], p4[1] - 12, 5, 0, 0, 90]

    # Edge tool contact points (RY=22).
    edge_right_1 = [p1[0] - 5, p1[1], 10, 0, 22, 0]
    edge_right_pre_1 = [p1[0] - 5 - ep, p1[1], 10, 0, 22, 0]
    edge_right_2 = [p2[0] - 5, p2[1], 10, 0, 22, 0]
    edge_right_pre_2 = [p2[0] - 5 - ep, p2[1], 10, 0, 22, 0]
    edge_right_12 = [p1[0] - 5, p1[1] + 2, 10, 0, 22, 0]

    edge_top_1 = [p2[0], p2[1] - 5, 10, 0, 22, -90]
    edge_top_pre_1 = [p2[0], p2[1] - 5 + ep, 10, 0, 22, -90]
    edge_top_2 = [p3[0], p3[1] - 5, 10, 0, 22, -90]
    edge_top_pre_2 = [p3[0], p3[1] - 5 + ep, 10, 0, 22, -90]
    edge_top_12 = [p2[0] + 2, p2[1] - 5, 10, 0, 22, -90]

    edge_left_1 = [p3[0] + 5, p3[1], 10, 0, 22, -180]
    edge_left_pre_1 = [p3[0] + 5 + ep, p3[1], 10, 0, 22, -180]
    edge_left_2 = [p4[0] + 5, p4[1], 10, 0, 22, -180]
    edge_left_pre_2 = [p4[0] + 5 + ep, p4[1], 10, 0, 22, -180]
    edge_left_12 = [p3[0] + 5, p3[1] - 2, 10, 0, 22, -180]

    edge_bottom_1 = [p4[0], p4[1] - 2, 10, 0, 22, 90]
    edge_bottom_pre_1 = [p4[0], p4[1] - 2 - ep, 10, 0, 22, 90]
    edge_bottom_2 = [p1[0], p1[1] - 2, 10, 0, 22, 90]
    edge_bottom_pre_2 = [p1[0], p1[1] - 2 - ep, 10, 0, 22, 90]
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


def _build_big_sections(door_num, topxshift=0.0):
    p1, p2, p3, p4 = _get_points(door_num)
    distance = (p4[0] - p1[0]) / 2
    topxshift = float(topxshift)

    def build(kind):
        is_edge = kind == "edge"
        z = 10 if is_edge else 5
        ry = 22 if is_edge else 0
        # Prepoints sit one standoff outward from the contact pose along that
        # side's own normal, so force search always starts clear of the surface
        # by a known distance instead of the 0-7 mm this used to vary between.
        pre = _prepoint_offset(is_edge)
        right_x = p1[0] - 4
        right_pre_x = right_x - pre
        # Upper sections live in the J7-shifted frame: when the top station was
        # clamped the rail moved topxshift LESS than a full door length, so add
        # it back so these points still land on the same physical door.
        upright_x = (-distance * 2 if is_edge else -distance * 2 - 4) + topxshift
        upright_pre_x = upright_x - pre
        top_y = p2[1] + 4
        uptop_y = p2[1] - 5 if is_edge else p2[1] + 4
        uptop_pre_y = uptop_y + pre
        left_x = p3[0] + 5
        left_pre_x = left_x + pre
        upleft_x = p1[0] + 3 + topxshift
        upleft_pre_x = upleft_x + pre
        bottom_y = p4[1] - 2 if is_edge else p4[1] - 12
        bottom_x_offset = -2 if is_edge else -2

        right_1 = [right_x, p1[1], z, 0, ry, 0]
        right_pre_1 = [right_pre_x, p1[1], z, 0, ry, 0]
        right_4 = [p2[0] - 4, p2[1] / 2, z, 0, ry, 0]
        right_4_pre = [p2[0] - 4 - pre, p2[1] / 2, z, 0, ry, 0]
        right_14 = [right_x, p1[1] + 2, z, 0, ry, 0]

        upright_4 = [upright_x, p2[1] / 2, z, 0, ry, 0]
        upright_4_pre = [upright_pre_x, p2[1] / 2, z, 0, ry, 0]
        upright_4_up = [upright_pre_x, p2[1] / 2 + 2, z, 0, ry, 0]
        upright_2 = [upright_pre_x, p2[1], z, 0, ry, 0]
        upright_2_pre = [upright_pre_x, p2[1], z, 0, ry, 0]

        topup_1 = [-distance * 2 + topxshift, uptop_y, z, 0, ry, -90]
        topup_pre_1 = [-distance * 2 + topxshift, uptop_pre_y, z, 0, ry, -90]
        topup_12 = [-distance * 2 + 2 + topxshift, uptop_y, z, 0, ry, -90]
        topup_2 = [p1[0] + topxshift, p3[1] - 5 if is_edge else p3[1] + 4, z, 0, ry, -90]
        topup_pre_2 = [p1[0] + topxshift, p3[1] - 5 if is_edge else p3[1] + 10, z, 0, ry, -90]

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

        bottom_2_y = p1[1] - 2 if is_edge else p1[1] - 12
        bottom_1 = [p4[0], bottom_y, z, 0, ry, 90]
        bottom_pre_1 = [p4[0], bottom_y - pre, z, 0, ry, 90]
        bottom_2 = [p1[0], bottom_2_y, z, 0, ry, 90]
        bottom_pre_2 = [p1[0], bottom_2_y - pre, z, 0, ry, 90]
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
        # These four run AFTER the J7 jump, so they share the shifted frame.
        "prehome_upright": [(-distance * 2) - 10 + topxshift, p2[1] / 2, 120, 0, 0, 0],
        "after_upright": [(-distance * 2) - 10 + topxshift, p2[1] + 30, 80, 0, 0, -90],
        "after_topup": [p1[0] + 10 + topxshift, p3[1] + 20, 80, 0, 0, -180],
        "after_upleft": [p1[0] + 10 + topxshift, p3[1] / 2, 100, 0, 0, -180],
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
        _run_combined_section(name, cps, config, side[name], edge[name], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
        if name == "left":
            _move_robot(cps, config, transitions[transition_name], robot_speed)
            moveOnlyJ6r(cps, -270, config, wait=True)
        else:
            _move_robot(cps, config, transitions[transition_name], robot_speed)


def _run_big_combined(door_num, side_force, side_cycles, edge_force, edge_cycles, cps, config, robot_speed, sanding_speed):
    p1, _, _, p4 = _get_points(door_num)
    x1 = get_door_position(door_num)
    door_len_x = p4[0] - p1[0]
    x2_desired = x1 + door_len_x
    clamped = x2_desired > TOOL2_J7_MAX_MM
    x2 = TOOL2_J7_MAX_MM if clamped else x2_desired
    # The top-half points were laid out assuming J7 advances a full door_len. When
    # clamped it only advances (MAX - x1), so the points must move forward by the
    # SHORTFALL to land on the same physical door.
    topxshift = door_len_x - (TOOL2_J7_MAX_MM - x1) if clamped else 0.0
    if clamped:
        message = (
            f"[tool2 combined] door {door_num} top J7 clamped: desired={x2_desired:.1f} "
            f"> max={TOOL2_J7_MAX_MM:.1f}, using {x2:.1f} (topxshift={topxshift:.1f})"
        )
        logger = config.get("logger") if isinstance(config, dict) else None
        if logger:
            logger.info(message)
        else:
            print(message)
    side, edge, transitions = _build_big_sections(door_num, topxshift)

    _move_j7(cps, config, x1, robot_speed)
    _move_robot(cps, config, transitions["prehoming"], robot_speed)

    _run_combined_section("left lower", cps, config, side["left_down"], edge["left_down"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_left_down"], robot_speed)
    moveOnlyJ6r(cps, -270, config, wait=True)

    _run_combined_section("bottom", cps, config, side["bottom"], edge["bottom"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_bottom"], robot_speed)

    _run_combined_section("right lower", cps, config, side["right_down"], edge["right_down"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_right_down"], robot_speed)

    # Start the long rail jump, then let the arm swing to the upper prehome pose
    # while it travels. That pose sits at Z=120, clear of the door for the whole
    # stroke, so the two motions overlap instead of the arm idling until the rail
    # lands. The section that follows re-asserts the J7 wait before contact.
    _move_j7(cps, config, x2, robot_speed, transition=True)
    _move_robot(cps, config, transitions["prehome_upright"], robot_speed, wait_for_seventh=False)
    # Barrier: the arm rode to the prehome pose while the rail travelled, but
    # nothing below re-checks J7, so block here before any section approaches the
    # door. This is a no-op once the rail has already landed.
    if not waitForSeventhAxisIdle(cps, config, context="tool2_combined_big_top_station"):
        raise RuntimeError(
            f"[Tool 2 Combined] 7th axis did not reach the top station ({x2:.1f} mm) before sanding."
        )

    _run_combined_section("right upper", cps, config, side["upright"], edge["upright"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_upright"], robot_speed)

    _run_combined_section("top", cps, config, side["topup"], edge["topup"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_topup"], robot_speed)

    _run_combined_section("left upper", cps, config, side["upleft"], edge["upleft"], side_force, side_cycles, edge_force, edge_cycles, sanding_speed, robot_speed)
    _move_robot(cps, config, transitions["after_upleft"], robot_speed)


def run_tool2_side_edge_combined(door_number, side_force, side_cycles, edge_force, edge_cycles, cps):
    door_num = int(door_number)
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
