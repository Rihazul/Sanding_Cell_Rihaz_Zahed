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
    stop_requested,
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
from cycle_data_utils import get_inverse_overlap_step
from smallTable.stop_lift import (
    TABLE_A_XY_SAFE_Z_MM,
    move_tablea_xy_safe_then_prepoint,
    stop_lift_to_preheight_tableA,
)

TRANSFER_LIFT_Z_MM = 70.0
FORCE_APPROACH_LIFT_MM = 15.0
FORCE_PRESEEK_GAP_MM = 4.0
CIRCULAR_TOOL_XY_OFFSET_MM = 50.0
TOOL4_DIAMETER_MM = 135.0
POCKET_MAX_OVERLAP_MM = 100.0


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    # Model F uses Tool 4 only; map legacy Tool 1 TCP key to Tool 4 TCP so
    # existing motion calls in this module run with Tool 4 calibration.
    coords = config.get("coords", {})
    if coords.get("tcptool4plane1"):
        coords["tcptool1plane1"] = coords["tcptool4plane1"]

    # Model F big-door split paths can queue many sanding points with wait=False
    # before the final wait=True. Increase sanding wait/blend budgets so we do
    # not release force and lift early due timeout.
    door_cfg = config.setdefault("door", {})
    try:
        door_cfg["sandingMoveLMinTimeoutSec"] = max(
            300.0, float(door_cfg.get("sandingMoveLMinTimeoutSec", 60.0))
        )
    except (TypeError, ValueError):
        door_cfg["sandingMoveLMinTimeoutSec"] = 300.0
    try:
        door_cfg["sandingBlendTimeoutSeconds"] = max(
            120.0, float(door_cfg.get("sandingBlendTimeoutSeconds", 30.0))
        )
    except (TypeError, ValueError):
        door_cfg["sandingBlendTimeoutSeconds"] = 120.0
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


def _resolve_inner_sanding_offset(cycle_cfg, default=50.0):
    """
    Convert UI overlap (mm) to sanding step (mm) safely.
    UI range is overlap=0..100; step is tool diameter minus overlap.
    """
    try:
        return float(
            get_inverse_overlap_step(
                cycle_cfg or {},
                key="inverseOverlapping",
                tool_diameter_mm=TOOL4_DIAMETER_MM,
                max_overlap_mm=POCKET_MAX_OVERLAP_MM,
                min_step_mm=1.0,
            )
        )
    except Exception:
        return max(1.0, float(default))

def _build_zigzag_operation_sequence(requested_orientation, cycles):
    """Return the orientation order to run without repeating one orientation back-to-back by cycle."""
    total_cycles = max(1, int(cycles))
    requested = str(requested_orientation or "vertical").strip().lower()

    if requested == "horizontal":
        single_orientation = "horizontal"
    else:
        single_orientation = "vertical"

    if total_cycles == 1:
        if requested == "both":
            return ((1, "vertical"), (1, "horizontal"))
        return ((1, single_orientation),)

    return tuple(
        (cycle_number, "vertical" if cycle_number % 2 else "horizontal")
        for cycle_number in range(1, total_cycles + 1)
    )

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

    point5 = [-p5[0], p5[1], z, 0.0, 0.0, 0.0]
    point6 = [-p6[0], p6[1], z, 0.0, 0.0, 0.0]
    point7 = [-p7[0], p7[1], z, 0.0, 0.0, 0.0]
    point8 = [-p8[0], p8[1], z, 0.0, 0.0, 0.0]

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
        step_mm = max(1.0, float(innerSandingOffset))

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
                num_steps = max(1, math.ceil(yinner / step_mm))
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
                num_steps = math.ceil(xinner / step_mm)
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
    cycles=1,
    cycle_paths=None,
):
    if cycle_paths is None:
        cycle_paths = [
            (cycle_index + 1, "unchanged", points1)
            for cycle_index in range(max(1, int(cycles)))
        ]
    valid_cycle_paths = [
        (cycle_number, cycle_orientation, cycle_path)
        for cycle_number, cycle_orientation, cycle_path in cycle_paths
        if cycle_path
    ]
    if not valid_cycle_paths:
        return

    start_point = [float(value) for value in valid_cycle_paths[0][2][0]]
    start_lift = _with_lift(start_point, FORCE_APPROACH_LIFT_MM)

    try:
        releaseForce(cps=cps, config=config, wait_for_blending=False)
    except Exception:
        pass

    start_lift[2] = max(float(start_lift[2]), TABLE_A_XY_SAFE_Z_MM)
    move_tablea_xy_safe_then_prepoint(
        cps=cps,
        config=config,
        point=start_lift,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        robot_speed=robot_speed,
    )
    if seventh_target is not None:
        communicate(
            cps=cps,
            config=config,
            seventh=seventh_target,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=False,
            require_seventh_ok=True
        )
    communicate(
        cps=cps,
        config=config,
        point=start_point,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robotspeed",
        wait=True,
    )
    force_ok = putForceZminus(
        cps=cps,
        force=force,
        tcp=config["coords"]["tcptool4plane1"],
        ucs=config["coords"]["ucsTable1"],
        config=config,
    )
    if not force_ok:
        raise RuntimeError("Force seek failed before Model F sanding start point.")

    turn_vibration_on(cps)
    try:
        total_cycles = max(int(item[0]) for item in valid_cycle_paths)
        BATCH_WAIT_EVERY = 8
        sent_points = 0
        for path_index, (cycle_number, cycle_orientation, cycle_points) in enumerate(valid_cycle_paths):
            if stop_requested():
                raise RuntimeError("[ModelF] Stop requested.")
            print(
                f"[ModelF] Continuous cycle {cycle_number}/{total_cycles} "
                f"orientation={cycle_orientation}, points={len(cycle_points)}"
            )
            if path_index == 0:
                points_to_send = cycle_points[1:]
            else:
                points_to_send = cycle_points

            for point_index, point in enumerate(points_to_send):
                if stop_requested():
                    raise RuntimeError("[ModelF] Stop requested.")
                sent_points += 1
                is_last_path = path_index == len(valid_cycle_paths) - 1
                is_last_point = is_last_path and point_index == len(points_to_send) - 1
                is_batch_end = sent_points % BATCH_WAIT_EVERY == 0
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config["coords"]["tcptool4plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sandingspeed",
                    wait=is_last_point or is_batch_end,
                )
        waitForBlending(cps=cps, config=config)
    finally:
        if stop_requested():
            # Operator Stop: lift straight up (contact Z + 15 mm) with no force so the tool
            # releases the surface immediately. From there operator can re-send or home.
            stop_lift_to_preheight_tableA(
                cps,
                config,
                tcp=config["coords"]["tcptool4plane1"],
                ucs=config["coords"]["ucsTable1"],
                preheight_z_mm=float(start_point[2]) + FORCE_APPROACH_LIFT_MM,
                robot_speed=robot_speed,
                last_xy=(start_point[0], start_point[1]),
            )
        else:
            turn_vibration_off(cps)
            releaseForce(cps=cps, config=config)

    last_point = valid_cycle_paths[-1][2][-1]
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

    strokes = []
    i = 0
    while i < len(path_points):
        if i + 1 < len(path_points):
            strokes.append([path_points[i], path_points[i + 1]])
            i += 2
        else:
            strokes.append([path_points[i]])
            i += 1

    if mode == "horizontal":
        first_half = []
        second_half = []
        for stroke in strokes:
            if len(stroke) == 1:
                first_half.extend(stroke)
                second_half.extend(stroke)
                continue

            start = [float(value) for value in stroke[0]]
            end = [float(value) for value in stroke[1]]
            mid = [(start[idx] + end[idx]) / 2.0 for idx in range(6)]

            if start[0] <= end[0]:
                first_half.extend([start, mid])
                second_half.extend([mid, end])
            else:
                first_half.extend([mid, end])
                second_half.extend([start, mid])
        return first_half, second_half

    axis = 0
    travel_axis = 1

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


def _run_door_small(door_num, force, z, cps, orientation, cycles=1):
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    cycle_cfg = load_json_config()

    robot_speed, sanding_speed = _get_motion_speeds(config)
    inner_sanding_offset = _resolve_inner_sanding_offset(cycle_cfg, default=50.0)

    frame_offset = 0
    points = _compute_boundary_points(door_num, frame_offset, z)
    p8 = points["p8"]
    x1 = p8[0] + get_door_position(door_num)

    operation_sequence = _build_zigzag_operation_sequence(orientation, cycles)
    path_orientations = tuple(dict.fromkeys(
        cycle_orientation for _, cycle_orientation in operation_sequence
    ))

    paths_by_orientation = {}
    for path_orientation in path_orientations:
        _, generated_path, _, _ = generate_zigzag_path(
            x_coords=points["x_coords"],
            y_coords=points["y_coords"],
            z_coords=points["z_coords"],
            innerOffset=0,
            innerOffsetX=0,
            orientation=path_orientation,
            innerSandingOffset=inner_sanding_offset,
            edge_coverage=False,
        )
        paths_by_orientation[path_orientation] = generated_path

    cycle_paths = [
        (
            cycle_number,
            cycle_orientation,
            paths_by_orientation[cycle_orientation],
        )
        for cycle_number, cycle_orientation in operation_sequence
    ]
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
        points1=cycle_paths[0][2],
        force=force,
        sanding_speed=sanding_speed,
        robot_speed=robot_speed,
        seventh_target=x1,
        cycle_paths=cycle_paths,
    )

def _run_door_big(door_num, force, z, cps, orientation, cycles=1):
    config = load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])
    cycle_cfg = load_json_config()

    robot_speed, sanding_speed = _get_motion_speeds(config)
    inner_sanding_offset = _resolve_inner_sanding_offset(cycle_cfg, default=50.0)

    frame_offset = 0
    points = _compute_boundary_points(door_num, frame_offset, z)
    p8 = points["p8"]
    p7 = points["p7"]
    p6 = points["p6"]

    tcx0 = p8[0] + get_door_position(door_num)
    tcx1 = tcx0 + ((p6[0] - p7[0]) / 2)
    operation_sequence = _build_zigzag_operation_sequence(orientation, cycles)
    path_orientations = tuple(dict.fromkeys(
        cycle_orientation for _, cycle_orientation in operation_sequence
    ))

    def build_pass_plan(cycle_orientation):
        _, zigzag_full, _, _ = generate_zigzag_path(
            x_coords=points["x_coords"],
            y_coords=points["y_coords"],
            z_coords=points["z_coords"],
            innerOffset=0,
            innerOffsetX=0,
            orientation=cycle_orientation,
            innerSandingOffset=inner_sanding_offset,
            edge_coverage=False,
        )
        path1, path2_global = _split_big_door_zigzag(
            zigzag_full, cycle_orientation
        )
        if not path1 and zigzag_full:
            path1 = zigzag_full[:]

        if cycle_orientation == "horizontal" and path2_global:
            path2 = [list(point) for point in path2_global]
        elif path1:
            path2 = [list(point) for point in path1]
        else:
            path2 = path2_global if path2_global else zigzag_full[:]

        if cycle_orientation == "horizontal":
            x2_shift = float(tcx1) - float(tcx0)
            path2 = [
                [
                    float(point[0]) - x2_shift,
                    float(point[1]),
                    float(point[2]),
                    float(point[3]),
                    float(point[4]),
                    float(point[5]),
                ]
                for point in path2
            ]
        return [(tcx0, path1), (tcx1, path2)]

    plans_by_orientation = {
        path_orientation: build_pass_plan(path_orientation)
        for path_orientation in path_orientations
    }

    # Complete every selected pattern on one J7 half before moving to the
    # other half. Selecting both makes each cycle vertical + horizontal.
    for pass_index in range(2):
        cycle_paths = []
        current_tcx = None
        for operation_cycle, cycle_orientation in operation_sequence:
            cycle_tcx, cycle_path = plans_by_orientation[cycle_orientation][pass_index]
            current_tcx = cycle_tcx
            if cycle_path:
                cycle_paths.append(
                    (operation_cycle, cycle_orientation, cycle_path)
                )

        if not cycle_paths or current_tcx is None:
            continue
        print(
            f"[ModelF] Door {door_num} grouped J7 pass "
            f"{pass_index + 1}/2: J7={current_tcx}, "
            f"sequence={operation_sequence}"
        )
        communicate(
            cps=cps,
            config=config,
            seventh=current_tcx,
            tcp=config["coords"]["tcptool4plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=robot_speed,
            velocity_profile="robotspeed",
            wait=False,
        )
        _perform_process_top(
            cps,
            config,
            points1=cycle_paths[0][2],
            force=force,
            sanding_speed=sanding_speed,
            robot_speed=robot_speed,
            seventh_target=current_tcx,
            cycle_paths=cycle_paths,
        )

def _run_door(door_num, force, z, cps, orientation, movement, cycles=1):
    # Model F: full-door zigzag only, no edge-coverage pass.
    orientation = str(orientation or "vertical").strip().lower()
    if orientation not in {"vertical", "horizontal", "both"}:
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
    if ylen_num > 600 and xlen_num > 350:
        print(
            f"[ModelF] Door {door_num}: large area detected (x={xlen_num:.1f}, y={ylen_num:.1f}) -> 2 seventh-axis passes."
        )
        _run_door_big(
            door_num,
            force,
            z,
            cps,
            orientation,
            cycles=cycles,
        )
    else:
        _run_door_small(door_num, force, z, cps, orientation, cycles=cycles)


def smalldoor1zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None, cycles=1):
    _run_door(1, force, z, cps, orientation, movement, cycles=cycles)


def smalldoor2zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None, cycles=1):
    _run_door(2, force, z, cps, orientation, movement, cycles=cycles)


def smalldoor3zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None, cycles=1):
    _run_door(3, force, z, cps, orientation, movement, cycles=cycles)


def smalldoor4zizag(force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None, cycles=1):
    _run_door(4, force, z, cps, orientation, movement, cycles=cycles)


if __name__ == "__main__":
    print("Model F zigzag module loaded. Use sandingModelFTableA() to run.")
