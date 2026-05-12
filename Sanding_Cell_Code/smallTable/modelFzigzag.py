# testcommu.py

import sys
import math
import os
import time
import json

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from Server_Better_V2 import (
    communicate,
    setup_logger,
    waitForBlending,
    turn_vibration_on,
    turn_vibration_off,
    putForceZminus,
    releaseForce,
)
from smallTable.scancord import (
    get_door_position,
    get_outer_corner_point,
    get_x_values,
    get_y_values,
)

TRANSFER_LIFT_Z_MM = 70.0
FORCE_APPROACH_LIFT_MM = 15.0
FORCE_PRESEEK_GAP_MM = 4.0
CIRCULAR_TOOL_XY_OFFSET_MM = 50.0


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    # Model F uses Tool 4 only; map legacy Tool 1 TCP key to Tool 4 TCP so
    # existing motion calls in this module run with Tool 4 calibration.
    coords = config.get("coords", {})
    if coords.get("tcptool4plane1"):
        coords["tcptool1plane1"] = coords["tcptool4plane1"]
    return config


def load_json_config():
    """Loads configuration from config.json."""
    with open("./configs/cycleData.json", "r") as file:
        config = json.load(file)
    return config


def _get_motion_speeds(config):
    cycle_cfg = load_json_config()
    ui_cfg = config.get("UI", {}) if isinstance(config, dict) else {}

    def _to_float(value, default):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    robot_default = ui_cfg.get("robotSpeed", 0.7)
    sanding_default = ui_cfg.get("sandSpeed", 0.6)

    robot_speed = _to_float(cycle_cfg.get("robotSpeed", robot_default), robot_default)
    sanding_speed = _to_float(
        cycle_cfg.get("sandingSpeed", sanding_default), sanding_default
    )

    return robot_speed, sanding_speed


def _with_lift(point, lift_mm):
    if not point:
        return point
    lifted = list(point)
    lifted[2] = float(lifted[2]) + float(lift_mm)
    return lifted


def _compute_boundary_points(door_num, frame_offset, z):
    p80 = get_outer_corner_point(door_num, 0)
    p70 = get_outer_corner_point(door_num, 1)
    p60 = get_outer_corner_point(door_num, 2)
    p50 = get_outer_corner_point(door_num, 3)

    p8 = [p80[0] + frame_offset, p80[1] + frame_offset, p80[2], p80[3], p80[4], 0]
    p7 = [p70[0] + frame_offset, p70[1] - frame_offset, p70[2], p70[3], p70[4], 0]
    p6 = [p60[0] - frame_offset, p60[1] - frame_offset, p60[2], p60[3], p60[4], 0]
    p5 = [p50[0] - frame_offset, p50[1] + frame_offset, p50[2], p50[3], p50[4], 0]

    distance = p6[0] - p8[0]

    point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
    point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
    point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
    point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]

    point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
    point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
    point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
    point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]

    x_coords = [point5u[0], point6u[0], point7u[0], point8u[0]]
    y_coords = [point5u[1], point6u[1], point7u[1], point8u[1]]
    z_coords = [point5u[2], point6u[2], point7u[2], point8u[2]]

    return {
        "p8": p8,
        "p7": p7,
        "p6": p6,
        "x_coords": x_coords,
        "y_coords": y_coords,
        "z_coords": z_coords,
    }


def generate_zigzag_path(
    x_coords,
    y_coords,
    z_coords,
    innerOffset,
    innerOffsetX,
    *,
    orientation="vertical",
    innerSandingOffset=50,
    edge_coverage=False,
):
    prepoint = None
    zigzag_coords = []

    xframe_1 = 0
    xframe_2 = 0

    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])

    if x_coords and y_coords and z_coords:
        z_zigzag = boundary_coords[0][2]
        base_xy_offset = float(CIRCULAR_TOOL_XY_OFFSET_MM)
        x_left_offset = base_xy_offset + float(innerOffsetX)
        x_right_offset = base_xy_offset + float(innerOffset)
        y_offset = base_xy_offset + float(innerOffset)

        modified_Point2 = [
            x_coords[1] + x_left_offset,
            y_coords[1] - y_offset,
        ]
        modified_Point3 = [
            x_coords[2] - x_right_offset,
            y_coords[2] - y_offset,
        ]
        modified_Point1 = [
            x_coords[0] + x_left_offset,
            y_coords[0] + y_offset,
        ]
        modified_Point4 = [
            x_coords[3] - x_right_offset,
            y_coords[3] + y_offset,
        ]

        xlen1 = abs(modified_Point3[0] - modified_Point1[0])
        xinner = xlen1 - xframe_1 - xframe_2

        x_min = min(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        x_max = max(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        y_min = min(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])
        y_max = max(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])

        orientation_mode = str(orientation or "vertical").strip().lower()
        if orientation_mode not in {"vertical", "horizontal"}:
            orientation_mode = "vertical"

        edge_coverage_coords = []
        edge_prepoint = None
        edge_offset = 1.75
        if edge_coverage:
            edge_total_offset = base_xy_offset + edge_offset
            edge_Point1 = [
                x_coords[0] + edge_total_offset,
                y_coords[0] + edge_total_offset,
            ]
            edge_Point2 = [
                x_coords[1] + edge_total_offset,
                y_coords[1] - edge_total_offset,
            ]
            edge_Point3 = [
                x_coords[2] - edge_total_offset,
                y_coords[2] - edge_total_offset,
            ]
            edge_Point4 = [
                x_coords[3] - edge_total_offset,
                y_coords[3] + edge_total_offset,
            ]

            if orientation_mode == "horizontal":
                edge_coverage_coords = [
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                    [edge_Point3[0], edge_Point3[1], z_zigzag, 0, 0, 0],
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                    [edge_Point1[0], edge_Point1[1], z_zigzag, 0, 0, 0],
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                ]
            else:
                edge_coverage_coords = [
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                    [edge_Point1[0], edge_Point1[1], z_zigzag, 0, 0, 0],
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                    [edge_Point3[0], edge_Point3[1], z_zigzag, 0, 0, 0],
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                ]

        if orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.floor(yinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [x_min, current_y, z_zigzag, 0, 0, 0],
                        [x_max, current_y, z_zigzag, 0, 0, 0],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                zigzag_coords.extend(
                    [
                        [x_min, y_max, z_zigzag, 0, 0, 0],
                        [x_max, y_max, z_zigzag, 0, 0, 0],
                    ]
                )
        else:
            # Vertical zigzag: sweep in X, stroke from bottom (y_min) to top (y_max).
            # This keeps the expected bottom->top, top->bottom behavior per column.
            xinner = abs(x_max - x_min)
            if xinner > 0:
                num_steps = math.ceil(xinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1
                adjusted_step = xinner / num_steps

                offset = 0.0
                toggle = 0

                while offset <= xinner + 1e-9:
                    current_x = x_min + offset
                    row_points = [
                        [current_x, y_min, z_zigzag, 0, 0, 0],
                        [current_x, y_max, z_zigzag, 0, 0, 0],
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                zigzag_coords.extend(
                    [
                        [x_min, y_min, z_zigzag, 0, 0, 0],
                        [x_min, y_max, z_zigzag, 0, 0, 0],
                    ]
                )

        for point in zigzag_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        for point in edge_coverage_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        if edge_coverage_coords:
            edge_prepoint = [
                abs(edge_coverage_coords[0][0]) + 0.5,
                edge_coverage_coords[0][1],
                z_zigzag,
                0,
                0,
                0,
            ]

        if orientation_mode == "horizontal":
            prepoint = [abs(x_min) + 0.5, y_max, z_zigzag, 0, 0, 0]
        else:
            prepoint = [abs(modified_Point1[0]) + 0.5, modified_Point1[1], z_zigzag, 0, 0, 0]
    return edge_coverage_coords, zigzag_coords, prepoint, edge_prepoint


def _perform_process_top(
    cps,
    config,
    points1,
    force,
    sanding_speed,
    robot_speed,
    seventh_target=None,
):
    if not points1:
        return

    start_point = [
        float(points1[0][0]),
        float(points1[0][1]),
        float(points1[0][2]),
        float(points1[0][3]),
        float(points1[0][4]),
        float(points1[0][5]),
    ]
    start_lift = _with_lift(start_point, FORCE_APPROACH_LIFT_MM)

    communicate(
        cps=cps,
        config=config,
        point=start_lift,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    if seventh_target is not None:
        # Let arm and 7th-axis overlap during transition, then hard-sync J7
        # before force engagement to avoid contact while J7 is still moving.
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_target,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=True,
        )
    preseek_point = _with_lift(start_point, FORCE_PRESEEK_GAP_MM)
    communicate(
        cps=cps,
        config=config,
        point=preseek_point,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    putForceZminus(
        cps=cps,
        force=force,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        config=config,
    )
    turn_vibration_on(cps)

    path_points = points1[1:]
    for point in path_points[:-1]:
        communicate(
            cps=cps,
            config=config,
            point=point,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=sanding_speed,
            velocity_profile="sandingspeed",
            wait=False,
        )
    final_point = [
        float(points1[-1][0]),
        float(points1[-1][1]),
        float(points1[-1][2]),
        float(points1[-1][3]),
        float(points1[-1][4]),
        float(points1[-1][5]),
    ]
    communicate(
        cps=cps,
        config=config,
        point=final_point,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=sanding_speed,
        velocity_profile="sandingspeed",
        wait=True,
    )

    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    releaseForce(cps=cps, config=config)
    last_point = points1[-1]
    last_lift = _with_lift(last_point, FORCE_APPROACH_LIFT_MM)
    communicate(
        cps=cps,
        config=config,
        point=last_lift,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )


def _split_big_door_zigzag(path_points, orientation):
    if not path_points:
        return [], []

    mode = str(orientation or "vertical").strip().lower()
    axis = 0 if mode == "vertical" else 1
    travel_axis = 1 if mode == "vertical" else 0

    strokes = []
    i = 0
    while i < len(path_points):
        if i + 1 < len(path_points):
            strokes.append([path_points[i], path_points[i + 1]])
            i += 2
        else:
            strokes.append([path_points[i]])
            i += 1

    scored = []
    for stroke in strokes:
        mean_axis = sum(float(p[axis]) for p in stroke) / len(stroke)
        scored.append((mean_axis, stroke))
    scored.sort(key=lambda item: item[0])

    half = max(1, len(scored) // 2)
    first_half = [s for _, s in scored[:half]]
    second_half = [s for _, s in scored[half:]]
    if not second_half:
        second_half = [s for _, s in scored[half - 1 :]]

    def _build(stroke_list):
        built = []
        for idx, stroke in enumerate(stroke_list):
            if len(stroke) == 1:
                built.extend(stroke)
                continue
            low, high = sorted(stroke, key=lambda p: float(p[travel_axis]))
            if idx % 2 == 0:
                built.extend([low, high])
            else:
                built.extend([high, low])
        return built

    return _build(first_half), _build(second_half)


def _run_door_small(door_num, force, z, cps, orientation):
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    robot_speed, sanding_speed = _get_motion_speeds(config)

    frame_offset = 0
    points = _compute_boundary_points(door_num, frame_offset, z)
    p8 = points["p8"]

    x1 = p8[0] + get_door_position(door_num)

    _, zigzag_path, _, _ = generate_zigzag_path(
        x_coords=points["x_coords"],
        y_coords=points["y_coords"],
        z_coords=points["z_coords"],
        innerOffset=0,
        innerOffsetX=0,
        orientation=orientation,
        edge_coverage=False,
    )

    communicate(
        cps=cps,
        config=config,
        seventh=x1,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=False,
    )
    _perform_process_top(
        cps,
        config,
        points1=zigzag_path,
        force=force,
        sanding_speed=sanding_speed,
        robot_speed=robot_speed,
        seventh_target=x1,
    )


def _run_door_big(door_num, force, z, cps, orientation):
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    robot_speed, sanding_speed = _get_motion_speeds(config)

    frame_offset = 0
    points = _compute_boundary_points(door_num, frame_offset, z)
    p8 = points["p8"]
    p7 = points["p7"]
    p6 = points["p6"]

    tcx0 = p8[0] + get_door_position(door_num)
    tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)

    _, zigzag_full, _, _ = generate_zigzag_path(
        x_coords=points["x_coords"],
        y_coords=points["y_coords"],
        z_coords=points["z_coords"],
        innerOffset=0,
        innerOffsetX=0,
        orientation=orientation,
        edge_coverage=False,
    )
    zigzag_path1, zigzag_path2_global = _split_big_door_zigzag(zigzag_full, orientation)

    if not zigzag_path1 and zigzag_full:
        zigzag_path1 = zigzag_full[:]
    if zigzag_path1:
        # Use the same local half-path for x2; seventh-axis shift places it on the
        # second half of the door without double-shifting toward the outside.
        zigzag_path2 = [list(p) for p in zigzag_path1]
    else:
        zigzag_path2 = zigzag_path2_global if zigzag_path2_global else zigzag_full[:]

    if zigzag_path1 and zigzag_path2:
        print(
            f"[ModelF] Door {door_num} split: x1 start={zigzag_path1[0][:3]}, "
            f"x2 start={zigzag_path2[0][:3]}"
        )

    for pass_index, (current_tcx, current_path) in enumerate([
        (tcx0, zigzag_path1),
        (tcx1, zigzag_path2),
    ]):
        if not current_path:
            continue
        seventh_wait = False
        communicate(
            cps=cps,
            config=config,
            seventh=current_tcx,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=seventh_wait,
        )
        _perform_process_top(
            cps,
            config,
            points1=current_path,
            force=force,
            sanding_speed=sanding_speed,
            robot_speed=robot_speed,
            seventh_target=current_tcx,
        )


def _run_door(door_num, force, z, cps, orientation, movement):
    # Model F: full-door zigzag only, no edge-coverage pass.
    orientation = str(orientation or "vertical").strip().lower()
    if orientation not in {"vertical", "horizontal"}:
        orientation = "vertical"

    if movement != "zigzag":
        print(
            f"[ModelF] Requested movement='{movement}' ignored; forcing 'zigzag'."
        )

    ylen_data = get_y_values(door_num, default_on_error=True)
    xlen_data = get_x_values(door_num, default_on_error=True)
    ylen = ylen_data.get("ylen")
    xlen = xlen_data.get("xlen")

    if ylen == "null" or xlen == "null":
        print(f"No door data available for door {door_num} - skipping operations")
        return

    try:
        ylen_num = float(ylen)
        xlen_num = float(xlen)
    except (TypeError, ValueError):
        print(
            f"Invalid ylen/xlen for door {door_num}: ylen={ylen} ({type(ylen)}), "
            f"xlen={xlen} ({type(xlen)})"
        )
        return

    # Large door rule: split only when BOTH dimensions exceed thresholds.
    if ylen_num > 600 and xlen_num > 280:
        print(
            f"[ModelF] Door {door_num}: large area detected (x={xlen_num:.1f}, y={ylen_num:.1f}) -> 2 seventh-axis passes."
        )
        _run_door_big(door_num, force, z, cps, orientation)
    else:
        _run_door_small(door_num, force, z, cps, orientation)


def smalldoor1zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    _run_door(1, force, z, cps, orientation, movement)


def smalldoor2zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    _run_door(2, force, z, cps, orientation, movement)


def smalldoor3zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    _run_door(3, force, z, cps, orientation, movement)


def smalldoor4zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None):
    _run_door(4, force, z, cps, orientation, movement)


if __name__ == "__main__":
    print("Model F zigzag module loaded. Use sandingModelFTableA() to run.")
