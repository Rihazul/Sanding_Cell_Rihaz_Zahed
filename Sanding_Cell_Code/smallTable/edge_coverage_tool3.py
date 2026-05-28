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
import math
import time
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


def _estimate_edge_blend_timeout(config, edge_points, edge_speed):
    """
    Compute a realistic blend wait timeout for the full edge contour.
    Low sanding speed + long contour can exceed fixed 12s easily.
    """
    base_timeout = _resolve_sanding_blend_timeout(config, fallback=20.0)
    if not edge_points or len(edge_points) < 2:
        return max(12.0, float(base_timeout))

    total_dist_mm = 0.0
    for i in range(1, len(edge_points)):
        prev = edge_points[i - 1]
        curr = edge_points[i]
        try:
            dx = float(curr[0]) - float(prev[0])
            dy = float(curr[1]) - float(prev[1])
            dz = float(curr[2]) - float(prev[2])
            total_dist_mm += math.sqrt(dx * dx + dy * dy + dz * dz)
        except Exception:
            continue

    try:
        base_velocity = float(config.get("door", {}).get("sanding", 375.0))
    except Exception:
        base_velocity = 375.0
    try:
        speed_ratio = max(0.01, min(1.0, float(edge_speed)))
    except Exception:
        speed_ratio = 1.0
    cmd_velocity = max(1.0, base_velocity * speed_ratio)
    est_travel_s = total_dist_mm / cmd_velocity

    # Include force-control drag and blending uncertainty with generous margin.
    dynamic_timeout = (est_travel_s * 2.5) + 8.0
    return min(120.0, max(float(base_timeout), dynamic_timeout))


def _verify_reached_last_edge_point(cps, target_point, timeout_s=25.0, tol_mm=25.0):
    """
    Fallback verifier when blending status times out:
    confirm TCP reached near the last contour point.
    """
    end_t = time.time() + max(1.0, float(timeout_s))
    while time.time() < end_t:
        act_pos = []
        nret = cps.HRIF_ReadActPos(0, 0, act_pos)
        if nret == 0 and isinstance(act_pos, (list, tuple)) and len(act_pos) > 8:
            try:
                dx = float(act_pos[6]) - float(target_point[0])
                dy = float(act_pos[7]) - float(target_point[1])
                dz = float(act_pos[8]) - float(target_point[2])
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist <= float(tol_mm):
                    return True
            except Exception:
                pass
        time.sleep(0.05)
    return False


def _wait_robot_motion_done(cps, timeout_s=45.0, poll_s=0.02):
    """
    Fallback motion completion check independent from blending state.
    Uses robot motion-done flag.
    """
    end_t = time.time() + max(1.0, float(timeout_s))
    while time.time() < end_t:
        robot_state = []
        nret = cps.HRIF_ReadRobotState(0, 0, robot_state)
        if nret == 0 and isinstance(robot_state, (list, tuple)) and len(robot_state) > 11:
            if str(robot_state[11]).strip() == "1":
                return True
        time.sleep(max(0.005, float(poll_s)))
    return False


def _resolve_force_seek_timeout(edge_speed, fallback=10.0):
    """
    Force-seek timeout must scale with sanding speed.
    At low user sanding speed (e.g. 0.15), fixed 10s is often too short.
    """
    try:
        speed_ratio = max(0.05, min(1.0, float(edge_speed)))
    except Exception:
        speed_ratio = 1.0

    # Keep 10s at >=35% speed; extend progressively below that.
    scale = max(1.0, 0.35 / speed_ratio)
    timeout = float(fallback) * scale
    return max(10.0, min(45.0, timeout))


def build_edge_coverage_path(
    x_coords,
    y_coords,
    z_coords,
    *,
    orientation="horizontal",
    tool3x=38.1,
    tool3y=50.8,
    edge_margin=2,
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
        x_coords[0] + tool3x + edge_margin + 4,
        y_coords[0] + tool3y + edge_margin,
        z_level,
    ]
    edge_point2 = [
        x_coords[1] + tool3x + edge_margin + 4, 
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

    edge_speed = _resolve_edge_speed(config)
    force_seek_linear = 5.0
    if edge_speed < 0.35:
        # Improve force seeking at very low sanding-speed settings.
        force_seek_linear = 8.0
    force_blending_timeout = 0.4 if split else 7.0
    force_seek_timeout = _resolve_force_seek_timeout(edge_speed, fallback=10.0)

    force_ok = putForceZminus(
        cps=cps,
        force=force,
        tcp=config["coords"][tcp_key],
        ucs=config["coords"][ucs_key],
        config=config,
        search_linear_velocity=force_seek_linear,
        blending_timeout_s=force_blending_timeout,
        max_seek_seconds=force_seek_timeout,
    )

    if not force_ok:
        # One retry with a longer timeout before failing the entire cycle.
        retry_timeout = min(60.0, force_seek_timeout * 1.5)
        retry_seek_linear = max(force_seek_linear, 10.0)
        if isinstance(config, dict) and config.get("logger"):
            config["logger"].warning(
                "[Edge Coverage] Force seek retry (timeout=%.1fs, linear=%.1f).",
                retry_timeout,
                retry_seek_linear,
            )
        force_ok = putForceZminus(
            cps=cps,
            force=force,
            tcp=config["coords"][tcp_key],
            ucs=config["coords"][ucs_key],
            config=config,
            search_linear_velocity=retry_seek_linear,
            blending_timeout_s=force_blending_timeout,
            max_seek_seconds=retry_timeout,
        )

    if not force_ok:
        raise RuntimeError("[Edge Coverage] Failed to establish force contact before edge path.")

    turn_vibration_on(cps)
    print(f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points")

    try:
        blend_timeout = _estimate_edge_blend_timeout(config, edge_points, edge_speed)
        if isinstance(config, dict) and config.get("logger"):
            config["logger"].info(
                "[Edge Coverage] blend timeout set to %.1fs (edge_speed=%.3f)",
                blend_timeout,
                float(edge_speed),
            )

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
                wait=False,
            )

        blend_ok = waitForBlending(
            cps=cps,
            config=config,
            timeout_s=blend_timeout,
        )
        if not blend_ok:
            # First fallback: wait for generic robot motion-done flag.
            motion_done = _wait_robot_motion_done(
                cps=cps,
                timeout_s=min(180.0, max(30.0, blend_timeout * 1.5)),
                poll_s=0.02,
            )
            if motion_done:
                if isinstance(config, dict) and config.get("logger"):
                    config["logger"].warning(
                        "[Edge Coverage] Blending timeout reported, but robot motion-done is true; continuing."
                    )
                print("[Edge Coverage] Completed linear edge path (motion-done fallback)")
                return True

            # Second fallback: approximate endpoint verification.
            reached_last = _verify_reached_last_edge_point(
                cps=cps,
                target_point=edge_points[-1],
                timeout_s=40.0,
                tol_mm=80.0,
            )
            if not reached_last:
                raise RuntimeError(
                    "[Edge Coverage] Blending wait timed out before contour completion."
                )
            if isinstance(config, dict) and config.get("logger"):
                config["logger"].warning(
                    "[Edge Coverage] Blending timeout reported, but final edge point was reached; continuing."
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
