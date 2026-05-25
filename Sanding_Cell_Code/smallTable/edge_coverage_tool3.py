from Server_Better_V2 import (
    communicate,
    putForceZminus,
    releaseForce,
    setup_logger,
    turn_vibration_off,
    turn_vibration_on,
    waitForBlending,
)
from smallTable.scancord import get_door_position, get_inner_corner_point
import json
import yaml


def _resolve_edge_speed(config):
    """
    Resolve edge coverage speed directly from cycleData.json `sandingSpeed`.
    """
    json_config = _load_json_config()
    raw_speed = json_config.get("sandingSpeed")
    if raw_speed is None:
        raw_speed = config.get("UI", {}).get("sandSpeed", 0.75)
    return float(raw_speed)


def _resolve_robot_speed(config):
    """
    Resolve robot transition speed directly from cycleData.json `robotSpeed`.
    """
    json_config = _load_json_config()
    raw_speed = json_config.get("robotSpeed")
    if raw_speed is None:
        raw_speed = config.get("UI", {}).get("robotSpeed", 0.9)
    return float(raw_speed)


def _resolve_sanding_blend_timeout(config, fallback=12.0):
    """
    Sanding paths (edge rectangle / zigzag) are longer than transition moves.
    Use a larger blend timeout to avoid releasing force/vibration mid-path.
    """
    try:
        door_cfg = config.get("door", {}) if isinstance(config, dict) else {}
        value = door_cfg.get(
            "sandingBlendTimeoutSeconds",
            door_cfg.get("blendTimeoutSeconds", fallback),
        )
        return max(2.0, float(value))
    except Exception:
        return float(fallback)


def build_edge_coverage_path(
    x_coords,
    y_coords,
    z_coords,
    *,
    orientation="horizontal",
    tool3x=38.1,
    tool3y=50.8,
    edge_margin=5,
    rx=-0.034,
    ry=0.556,
    rz=0.251,
):
    """Return rectangular edge-coverage points that end at zigzag start point."""
    if not x_coords or not y_coords or not z_coords:
        return []

    z_level = float(z_coords[0])
    orientation_mode = (orientation or "horizontal").lower()

    edge_point1 = [
        x_coords[0] + tool3x + edge_margin,
        y_coords[0] + tool3y + edge_margin,
        z_level,
    ]
    edge_point2 = [
        x_coords[1] + tool3x + edge_margin,
        y_coords[1] - tool3y - edge_margin,
        z_level,
    ]
    edge_point3 = [
        x_coords[2] - tool3x - edge_margin ,
        y_coords[2] - tool3y - edge_margin,
        z_level,
    ]
    edge_point4 = [
        x_coords[3] - tool3x - edge_margin,
        y_coords[3] + tool3y + edge_margin,
        z_level,
    ]

    if orientation_mode == "horizontal":
        path = [
            [edge_point2[0], edge_point2[1], edge_point2[2], rx, ry, rz],
            [edge_point3[0], edge_point3[1], edge_point3[2], rx, ry, rz],
            [edge_point4[0], edge_point4[1], edge_point4[2], rx, ry, rz],
            [edge_point1[0], edge_point1[1], edge_point1[2], rx, ry, rz],
            [edge_point2[0], edge_point2[1], edge_point2[2], rx, ry, rz],
        ]
    else:
        path = [
            [edge_point4[0], edge_point4[1], edge_point4[2], rx, ry, rz],
            [edge_point1[0], edge_point1[1], edge_point1[2], rx, ry, rz],
            [edge_point2[0], edge_point2[1], edge_point2[2], rx, ry, rz],
            [edge_point3[0], edge_point3[1], edge_point3[2], rx, ry, rz],
            [edge_point4[0], edge_point4[1], edge_point4[2], rx, ry, rz],
        ]

    for point in path:
        point[0] = abs(point[0])
        point[1] = abs(point[1])

    return path


def execute_edge_coverage(
    cps,
    config,
    edge_points,
    force,
    *,
    split=False,
    tcp_key="tcptool3plane1",
    ucs_key="ucsTable1",
):
    """Execute edge coverage path with force and vibration control."""
    if not edge_points:
        return False
    if float(force) <= 0.0:
        raise RuntimeError(
            f"[Edge Coverage] Invalid force={force}. Tool 3 edge coverage requires force > 0."
        )

    force_seek_linear = 5.0
    force_blending_timeout = 0.4 if split else 7.0

    force_ok = putForceZminus(
        cps=cps,
        force=force,
        tcp=config["coords"][tcp_key],
        ucs=config["coords"][ucs_key],
        config=config,
        search_linear_velocity=force_seek_linear,
        blending_timeout_s=force_blending_timeout,
    )
    if not force_ok:
        raise RuntimeError("[Edge Coverage] Failed to establish force contact before edge path.")

    edge_speed = _resolve_edge_speed(config)
    turn_vibration_on(cps)
    print(f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points")

    try:
        for idx, point in enumerate(edge_points):
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config["coords"][tcp_key],
                ucs=config["coords"][ucs_key],
                seventh=-1,
                speed=edge_speed,
                velocity_profile="sanding",
                speed_mode="linear",
                wait=idx == (len(edge_points) - 1),
            )

        waitForBlending(
            cps=cps,
            config=config,
            timeout_s=_resolve_sanding_blend_timeout(config),
        )
        print("[Edge Coverage] Completed linear edge path")
        return True
    finally:
        turn_vibration_off(cps)
        releaseForce(cps=cps, config=config)


def _load_config():
    with open("./configs/config.yaml", "r") as file:
        return yaml.safe_load(file)


def _load_json_config():
    with open("./configs/cycleData.json", "r") as file:
        return json.load(file)


def _build_pocket_xy_for_door(door_num, z):
    p8 = get_inner_corner_point(door_num, 0)
    p7 = get_inner_corner_point(door_num, 1)
    p6 = get_inner_corner_point(door_num, 2)
    p5 = get_inner_corner_point(door_num, 3)
    if not all((p5, p6, p7, p8)):
        raise RuntimeError(f"Missing pocket corner points for door {door_num}.")

    distance = p6[0] - p8[0]
    point5u = [-distance, p5[1], z]
    point6u = [-distance, p6[1], z]
    point7u = [0, p7[1], z]
    point8u = [0, p8[1], z]

    x_coords = [point5u[0], point6u[0], point7u[0], point8u[0]]
    y_coords = [point5u[1], point6u[1], point7u[1], point8u[1]]
    z_coords = [point5u[2], point6u[2], point7u[2], point8u[2]]
    seventh_pos = p8[0] + get_door_position(door_num)
    return x_coords, y_coords, z_coords, seventh_pos


def _run_single_door_edge_pass(cps, force, z, door_num, orientation):
    config = _load_config()
    config["logger"] = setup_logger(config["settings"]["debug"])

    x_coords, y_coords, z_coords, seventh_pos = _build_pocket_xy_for_door(door_num, z)
    edge_points = build_edge_coverage_path(
        x_coords=x_coords,
        y_coords=y_coords,
        z_coords=z_coords,
        orientation=orientation,
    )
    if not edge_points:
        return

    prepoint = [
        edge_points[0][0] + 0.5,
        edge_points[0][1],
        edge_points[0][2],
        edge_points[0][3],
        edge_points[0][4],
        edge_points[0][5],
    ]
    prehoming = [0, 200, 50, 0, 0, 0]

    robot_speed = _resolve_robot_speed(config)

    communicate(
        cps=cps,
        config=config,
        seventh=seventh_pos,
        tcp=config["coords"]["tcptool3plane1"],
        ucs=config["coords"]["ucsTable1"],
        speed=robot_speed,
        velocity_profile="robot",
        wait=True,
    )
    communicate(
        cps=cps,
        config=config,
        point=prepoint,
        tcp=config["coords"]["tcptool3plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robot",
        wait=True,
    )
    execute_edge_coverage(
        cps=cps,
        config=config,
        edge_points=edge_points,
        force=force,
        split=False,
    )
    communicate(
        cps=cps,
        config=config,
        point=prehoming,
        tcp=config["coords"]["tcptool3plane1"],
        ucs=config["coords"]["ucsTable1"],
        seventh=-1,
        speed=robot_speed,
        velocity_profile="robot",
        wait=True,
    )


def run_tool3_pocket_edge_cycles(
    count,
    force,
    door_num,
    z,
    cps,
    *,
    orientation="horizontal",
    spiral_settings=None,
):
    """Run Tool 3 pocket edge coverage only (no zigzag) for selected door."""
    del spiral_settings  # Compatibility with caller signature.
    if count <= 0:
        return
    if door_num not in (1, 2, 3, 4):
        raise ValueError(f"Invalid door number: {door_num}. Must be 1-4")

    for i in range(count):
        print(f"\n=== TOOL 3 EDGE CYCLE {i + 1}/{count} (Door {door_num}) ===")
        _run_single_door_edge_pass(
            cps=cps,
            force=force,
            z=z,
            door_num=door_num,
            orientation=orientation,
        )
