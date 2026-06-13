import os
import sys
import json

import yaml

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Server_Better_V2 import (
    communicate,
    setup_logger,
    waitForBlending,
    turn_vibration_on,
    turn_vibration_off,
    putForceZminus,
    releaseForce,
)
from smallTable.scancord import get_door_position, get_inner_corner_point, get_y_values


INTERNAL_OFFSET_MM = 38.0
INTERNAL_FORCE_N = 5
FORCE_APPROACH_Z_MM = 40.0


def load_config():
    with open("./configs/config.yaml", "r") as file:
        return yaml.safe_load(file)


def _load_cycle_config():
    try:
        with open("./configs/cycleData.json", "r") as file:
            return json.load(file)
    except Exception:
        return {}


def _get_motion_speeds(config):
    cycle_cfg = _load_cycle_config()
    ui_cfg = config.get("UI", {}) if isinstance(config, dict) else {}

    def _to_float(value, default):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    robot_default = ui_cfg.get("robotSpeed", 0.7)
    sand_default = ui_cfg.get("sandSpeed", 0.6)
    robot_speed = _to_float(cycle_cfg.get("robotSpeed", robot_default), robot_default)
    sanding_speed = _to_float(cycle_cfg.get("sandingSpeed", sand_default), sand_default)
    return robot_speed, sanding_speed


def _get_internal_bounds(door_number):
    c0 = get_inner_corner_point(door_number, 0)
    c1 = get_inner_corner_point(door_number, 1)
    c2 = get_inner_corner_point(door_number, 2)
    c3 = get_inner_corner_point(door_number, 3)

    x_min = min(c0[0], c1[0])
    x_max = max(c2[0], c3[0])
    y_min = min(c0[1], c3[1])
    y_max = max(c1[1], c2[1])

    left_x = x_min + INTERNAL_OFFSET_MM
    right_x = x_max - INTERNAL_OFFSET_MM
    bottom_y = y_min + INTERNAL_OFFSET_MM
    top_y = y_max - INTERNAL_OFFSET_MM

    if right_x <= left_x or top_y <= bottom_y:
        raise ValueError(
            f"Door {door_number}: internal offset {INTERNAL_OFFSET_MM} too large "
            f"for inner corner rectangle."
        )

    return left_x, right_x, bottom_y, top_y


def _build_internal_points(door_number, z):
    left_x, right_x, bottom_y, top_y = _get_internal_bounds(door_number)

    p0 = [left_x, bottom_y, z, 0, 0, 0]
    p1 = [left_x, top_y, z, 0, 0, 0]
    p2 = [right_x, top_y, z, 0, 0, 0]
    p3 = [right_x, bottom_y, z, 0, 0, 0]
    pre_start = [left_x, bottom_y, FORCE_APPROACH_Z_MM, 0, 0, 0]

    return pre_start, [p0, p1, p2, p3, p0]


def _build_split_internal_passes(door_number, z):
    left_x, right_x, bottom_y, top_y = _get_internal_bounds(door_number)
    middle_x = (left_x + right_x) / 2.0
    middle_y = (bottom_y + top_y) / 2.0

    bottom_pre = [left_x, middle_y, FORCE_APPROACH_Z_MM, 0, 0, 0]
    bottom_path = [
        [left_x, middle_y, z, 0, 0, 0],
        [left_x, bottom_y, z, 0, 0, 0],
        [right_x, bottom_y, z, 0, 0, 0],
        [right_x, middle_y, z, 0, 0, 0],
    ]

    # Moving J7 by middle_x requires subtracting middle_x from robot X
    # coordinates so the tool remains on the same scanned station positions.
    top_right_x = right_x - middle_x
    top_left_x = left_x - middle_x
    top_pre = [top_right_x, middle_y, FORCE_APPROACH_Z_MM, 0, 0, 0]
    top_path = [
        [top_right_x, middle_y, z, 0, 0, 0],
        [top_right_x, top_y, z, 0, 0, 0],
        [top_left_x, top_y, z, 0, 0, 0],
        [top_left_x, middle_y, z, 0, 0, 0],
    ]

    return middle_x, [(bottom_pre, bottom_path), (top_pre, top_path)]


def _requires_split_path(door_number):
    y_data = get_y_values(door_number, default_on_error=True)
    ylen = y_data.get("ylen") if isinstance(y_data, dict) else None
    use_split_path = isinstance(ylen, (int, float)) and ylen > 600
    print(
        f"door {door_number} Model D internal path: "
        f"{'two J7 positions' if use_split_path else 'one J7 position'} "
        f"(ylen={ylen})"
    )
    return use_split_path


def _resolve_force(user_force):
    if user_force is None:
        return INTERNAL_FORCE_N
    try:
        return float(user_force)
    except (TypeError, ValueError):
        return INTERNAL_FORCE_N


def _run_model_d_internal_for_door(door_number, z, cps, force=None):
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    robot_speed, sanding_speed = _get_motion_speeds(config)

    tcp = config["coords"]["tcptool1plane1"]
    ucs = config["coords"]["ucsTable1"]

    seventh = get_door_position(door_number)
    prehoming = [0, 0, 100, 0, 0, 0]
    applied_force = _resolve_force(force)

    if _requires_split_path(door_number):
        j7_offset, passes = _build_split_internal_passes(door_number, z)
        pass_targets = [seventh, seventh + j7_offset]
    else:
        pre_start, path_points = _build_internal_points(door_number, z)
        passes = [(pre_start, path_points)]
        pass_targets = [seventh]

    for pass_number, (j7_target, (pre_start, path_points)) in enumerate(
        zip(pass_targets, passes), start=1
    ):
        print(
            f"door {door_number} Model D internal pass {pass_number}/{len(passes)}: "
            f"J7={j7_target}"
        )
        communicate(
            cps=cps,
            config=config,
            seventh=j7_target,
            tcp=tcp,
            ucs=ucs,
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=pass_number == 1,
        )
        if pass_number == 1:
            communicate(
                cps=cps,
                config=config,
                point=prehoming,
                tcp=tcp,
                ucs=ucs,
                seventh=-1,
                speed=robot_speed,
                velocity_profile="robotspeed",
                wait=True,
            )
        communicate(
            cps=cps,
            config=config,
            point=pre_start,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=True,
        )

        force_applied = False
        vibration_on = False
        try:
            putForceZminus(
                cps=cps,
                force=applied_force,
                tcp=tcp,
                ucs=ucs,
                config=config,
            )
            force_applied = True
            turn_vibration_on(cps)
            vibration_on = True

            for point in path_points:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=tcp,
                    ucs=ucs,
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=True,
                )
        finally:
            if vibration_on:
                try:
                    waitForBlending(cps=cps, config=config)
                except Exception:
                    pass
                turn_vibration_off(cps)
            if force_applied:
                releaseForce(cps=cps, config=config)

        communicate(
            cps=cps,
            config=config,
            point=pre_start,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=True,
        )


def smalldoor1tool3(z, cps, force=None):
    _run_model_d_internal_for_door(1, z, cps, force=force)


def smalldoor2tool3(z, cps, force=None):
    _run_model_d_internal_for_door(2, z, cps, force=force)


def smalldoor3tool3(z, cps, force=None):
    _run_model_d_internal_for_door(3, z, cps, force=force)


def smalldoor4tool3(z, cps, force=None):
    _run_model_d_internal_for_door(4, z, cps, force=force)
