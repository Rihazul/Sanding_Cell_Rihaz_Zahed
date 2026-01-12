# testcommu.py

import sys
import math
import os
import time
import json
import uuid
from typing import Optional

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Components.RobotState import RobotState
from Server_Better_V2 import (
    communicate,
    setup_logger,
    waitForBlending,
    turn_vibration_on,
    turn_vibration_off,
    putForce,
    releaseForce,
    putForceZplus,
    putForceZminus,
)
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
    get_x_values,
    get_y_values,
)


def compute_timeout(
    *,
    total_points: Optional[int] = None,
    cap: float = 72.0,
    velocity: float = 300.0,
) -> float:
    """
    Estimate a completion timeout based on total points.

    - Uses a simple model: time = (points / vmax) + overhead
    - Caps the timeout to avoid excessive waits.
    """
    if total_points is None:
        return cap

    distance_per_point = 4.64
    # vmax = 300.0
    # time_factor_max = float(total_points / vmax)
    time_factor = float(total_points * distance_per_point / (velocity))

    timeout = float(time_factor)
    print(
        f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
    )
    return timeout


DEFAULT_SPIRAL_LINEAR_SPEED = 200.0
DEFAULT_SPIRAL_RADIUS = 12.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _linear_speed_to_turns(speed_mm_s: float) -> int:
    """Map 100–300 mm/s to 20–10 turns (faster speed -> fewer turns)."""
    speed = _clamp(speed_mm_s, 100.0, 300.0)
    turns = 20.0 - (speed - 100.0) * (10.0 / 200.0)
    print(turns)
    return int(round(_clamp(turns, 10.0, 20.0)))


def apply_spiral_settings(settings: Optional[dict]) -> None:
    """Apply incoming spiral settings to defaults used during spiral generation."""
    global DEFAULT_SPIRAL_LINEAR_SPEED, DEFAULT_SPIRAL_RADIUS
    if not settings:
        return
    # Handle dataclass or plain dict
    radius_val = (
        getattr(settings, "radius_mm", None)
        if not isinstance(settings, dict)
        else settings.get("radius_mm")
    )
    linear_val = (
        getattr(settings, "linear_speed_mm_s", None)
        if not isinstance(settings, dict)
        else settings.get("linear_speed_mm_s")
    )
    DEFAULT_SPIRAL_LINEAR_SPEED = _clamp(
        float(linear_val if linear_val is not None else DEFAULT_SPIRAL_LINEAR_SPEED),
        100.0,
        300.0,
    )
    DEFAULT_SPIRAL_RADIUS = _clamp(
        float(radius_val if radius_val is not None else DEFAULT_SPIRAL_RADIUS),
        10.0,
        15.0,
    )


def generate_spiral_between_points(
    start_pose,
    end_pose,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = 80,
    orientation: str = "horizontal",
    x_bounds: Optional[tuple] = None,  # (x_min, x_max) for clamping
    y_bounds: Optional[tuple] = None,  # (y_min, y_max) for clamping
):
    """
    Build a spiral path between two cartesian poses (X, Y, Z, Rx, Ry, Rz).
    Keeps orientation from start_pose, interpolates center XY between poses,
    and adds radial offsets for each step.
    
    x_bounds/y_bounds: Optional tuple (min, max) to clamp spiral points within valid range.
    """
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]
    if radius is None:
        radius = DEFAULT_SPIRAL_RADIUS
    if y0 == y1 and orientation == "vertical":
        turns = 2
    elif x0 == x1 and orientation == "horizontal":
        turns = 2
    else:
        dist = max(abs(x1 - x0), abs(y1 - y0))
        # factor = 1.0
        # if orientation == "vertical":
        #     factor= 1.3
        # turns = int((4.0/260.0) * float(dist) * float(factor))
        turns = math.floor(dist/(radius*3))

    # Always map speed -> turns so higher speed yields fewer turns

    total_steps = int(turns * (360.0 / angle_step_deg))
    if max_points is not None:
        total_steps = max(1, min(total_steps, int(max_points)))
    else:
        total_steps = max(1, total_steps)
    points = list(start_pose[:6])  # start point

    for step in range(1, total_steps + 1):
        t = step / total_steps
        cx = x0 + (x1 - x0) * t
        cy = y0 + (y1 - y0) * t
        theta_deg = step * angle_step_deg
        theta = math.radians(theta_deg)

        px = cx + radius * math.cos(theta)
        py = cy + radius * math.sin(theta)
        pz = z0  # hold Z while spiraling along Y

        # Clamp to boundaries if provided (prevents points outside safety zone)
        if x_bounds is not None:
            px = max(x_bounds[0], min(x_bounds[1], px))
        if y_bounds is not None:
            py = max(y_bounds[0], min(y_bounds[1], py))

        points.extend([px, py, pz, rx, ry, rz])

    return points


def run_spiral_between_points(
    cps: CPSClient,
    config: dict,
    start_pose,
    end_pose,
    *,
    turns: Optional[int] = 12,
    radius: Optional[float] = None,
    angle_step_deg: float = 120.0,
    max_points: Optional[int] = 80,
    tcp: str = None,
    ucs: str = None,
    track_name: str = None,
    velocity: float = 300.0,
    accel: float = 500.0,
    jerk: float = 10000.0,
    box_id: int = 0,
    robot_id: int = 0,
    init_path: bool = True,
    orientation: str = "horizontal",
    x_bounds: Optional[tuple] = None,  # (x_min, x_max) for clamping
    y_bounds: Optional[tuple] = None,  # (y_min, y_max) for clamping
):
    """Push a spiral MovePathL between two poses.

    By default (`finalize=False`), the path is only pushed (optionally initialized
    with `init_path`) and execution must be triggered separately via
    `finalize_spiral_path`. Set `finalize=True` to push and execute immediately.
    """
    base_name = track_name or "spiral_path"
    track_name = track_name
    print(f"[Spiral] Using track name: {track_name}")
    tcp_name = tcp or config["coords"].get("tcptool1plane1")
    ucs_name = ucs or config["coords"].get("ucsTable1")

    all_points = generate_spiral_between_points(
        start_pose=start_pose,
        end_pose=end_pose,
        turns=turns,
        radius=DEFAULT_SPIRAL_RADIUS if radius is None else radius,
        angle_step_deg=angle_step_deg,
        max_points=max_points,
        orientation=orientation,
        x_bounds=x_bounds,
        y_bounds=y_bounds,
    )

    if len(all_points) % 6 != 0:
        print("[Spiral] Point list misaligned (not divisible by 6).")
        return False, None

    count = len(all_points) // 6
    print(f"[Spiral] Total points = {count}")

    if init_path:
        ret = cps.HRIF_InitMovePathL(
            box_id, robot_id, track_name, velocity, accel, jerk, ucs_name, tcp_name
        )
        print("[Spiral][Init] ret =", ret)
        if ret != 0:
            return False, None

    ret = cps.HRIF_PushMovePaths(box_id, robot_id, track_name, 1, count, all_points)
    print("[Spiral][Push] ret =", ret)
    if ret != 0:
        return False, None

    return True, count


def finalize_spiral_path(
    cps: CPSClient,
    track_name: str,
    *,
    box_id: int = 0,
    robot_id: int = 0,
    completion_timeout=76.0,
) -> bool:
    """End push, wait for readiness, execute MovePathL, and wait for completion."""
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
    print("[Spiral][EndPush] ret =", ret)
    if ret != 0:
        return False

    # Create a lightweight state helper for motion monitoring.
    robot_state = RobotState(config=load_config(), cps_client=cps)

    # Wait for PATH_READY
    start = time.time()
    state = []
    while True:
        st = []
        ret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, st)
        if ret == 0 and len(st) > 3 and st[2] == "3" and st[3] == "0":
            break
        elapsed = time.time() - start
        if elapsed > 120.0:
            print(f"[Spiral] Timeout waiting for PATH_READY after {elapsed:.1f}s:", st)
            return False
        time.sleep(0.1)
        print(f"Waiting for PATH_READY... t={elapsed:.1f}s state={st}\r", end="")

    turn_vibration_on(cps)

    print("[Spiral][Move] Starting MovePathL...")
    ret = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print("[Spiral][Move] ret =", ret)
    if ret != 0:
        return False

    # Wait for completion (track path state + robot flags)
    start = time.time()
    while True:
        pstate = []
        pret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
        if pret != 0:
            print("HRIF_ReadPathState failed:", pret)
            return False
        if len(pstate) > 3 and pstate[3] != "0":
            print(f"[Spiral] Path reported error: {pstate}")
            return False

        # Break early if the robot stays still for 0.5s.
        if robot_state.wait_until_still(still_seconds=0.2, poll_seconds=0.05):
            print("[Spiral] No motion detected for 0.5s; exiting wait loop.")
            break

        # motion_result = []
        # mret = cps.HRIF_IsMotionDone(box_id, robot_id, motion_result)
        # print(f"[Spiral] Path state: {pstate}, Motion done: {motion_result}")
        # if mret != 0:
        #     print("HRIF_IsMotionDone failed:", mret)
        #     return False

        # if motion_result[0]:
        #     break

        # if time.time() - start > completion_timeout:
        #     print("Timeout waiting for idle. Path state:", pstate)
        #     return False
        # time.sleep(0.1)
        # print(f"Waiting for spiral move to complete... path={pstate}\r", end="")

    return True


def generate_zigzag_path(
    x_coords,
    y_coords,
    z_coords,
    innerOffset,
    innerOffsetX,
    orientation="horizontal",
    movement="zigzag",
    innerSandingOffset=50,
    edge_coverage=False,
):
    """
    Generate a zigzag/spiral path for sanding.

    Args:
        edge_coverage: If True, prepend a rectangular edge coverage path
                       (P1→P2→P3→P4→P1) before the zigzag pattern. The zigzag
                       then starts immediately from P1 after edge coverage completes.
    """
    prepoint = None
    zigzag_coords = []

    # Parameters (adjust as needed)
    tool3y = 50.8  # Tool offset in Y
    tool3x = 38.1  # Tool offset in X
    xframe_1 = 0
    xframe_2 = 0

    # 1) Collect boundary coordinates as [x, y, z]
    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    # Close the loop by duplicating the first point at the end
    if boundary_coords:
        boundary_coords.append(boundary_coords[0][:])  # copy for safety

    # 2) Compute the zigzag path (offset corners + zigzag)
    zigzag_coords = []

    # Ensure we have valid coordinates
    if x_coords and y_coords and z_coords:
        # We'll assume the pocket's Z-level is the same as the first boundary point
        z_zigzag = boundary_coords[0][2]

        # For Pocket4, corners (P13, P14, P15, P16):
        modified_Point2 = [
            (x_coords[1]) / 1 + tool3x + innerOffsetX,
            y_coords[1] - tool3y - (innerOffset),
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            (x_coords[0]) / 1 + tool3x + innerOffsetX,
            y_coords[0] + tool3y + innerOffset,
        ]
        modified_Point4 = [
            x_coords[3] - tool3x - innerOffset,
            y_coords[3] + tool3y + innerOffset,
        ]
        print("modified_Point1:", modified_Point1)
        print("modified_Point2:", modified_Point2)
        print("modified_Point3:", modified_Point3)
        print("modified_Point4:", modified_Point4)

        # Bounding box for alternate orientations
        x_min = min(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        x_max = max(
            modified_Point1[0],
            modified_Point2[0],
            modified_Point3[0],
            modified_Point4[0],
        )
        y_min = min(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )
        y_max = max(
            modified_Point1[1],
            modified_Point2[1],
            modified_Point3[1],
            modified_Point4[1],
        )

        # Calculate available horizontal dimension
        xinner = abs(x_max - x_min)
        print("xinner=", xinner)

        orientation_mode = (orientation or "vertical").lower()
        movement_mode = (movement or "zigzag").lower()

        # Modified Point Layout:
        # modified_Point1 = Bottom-right
        # modified_Point2 = Top-right
        # modified_Point3 = Top-left
        # modified_Point4 = Bottom-left

        # Generate edge coverage path if enabled (rectangular path around the boundary)
        # The edge coverage must END at the point where zigzag/spiral will START
        # Edge coverage uses ORIGINAL boundary points (without innerOffsetX/innerOffset applied)
        edge_coverage_coords = []
        if edge_coverage:
            # Remove the offsets from modified points to get original boundary positions
            # Modified points already have tool3x, tool3y, innerOffsetX, and innerOffset applied
            # For edge coverage, we only want tool3x and tool3y, not the inner offsets
            edge_Point1 = [x_coords[0] + tool3x + 1.0, y_coords[0] + tool3y + 1.0, z_zigzag]
            edge_Point2 = [x_coords[1] + tool3x + 1.0, y_coords[1] - tool3y - 1.0, z_zigzag]
            edge_Point3 = [x_coords[2] - tool3x - 1.0, y_coords[2] - tool3y - 1.0, z_zigzag]
            edge_Point4 = [x_coords[3] - tool3x - 1.0, y_coords[3] + tool3y + 1.0, z_zigzag]

            # Edge coverage always uses the same path regardless of orientation
            # Edge coverage: P2 → P3 → P4 → P1 → P2 (starts at top-left like horizontal)
            edge_coverage_coords = [
                [
                    edge_Point2[0],
                    edge_Point2[1],
                    edge_Point2[2],
                    0,
                    0,
                    0,
                ],  # Start P2 (top-left)
                [
                    edge_Point3[0],
                    edge_Point3[1],
                    edge_Point3[2],
                    0,
                    0,
                    0,
                ],  # P3 (top-right)
                [
                    edge_Point4[0],
                    edge_Point4[1],
                    edge_Point4[2],
                    0,
                    0,
                    0,
                ],  # P4 (bottom-right)
                [
                    edge_Point1[0],
                    edge_Point1[1],
                    edge_Point1[2],
                    0,
                    0,
                    0,
                ],  # P1 (bottom-left)
                [
                    edge_Point2[0],
                    edge_Point2[1],
                    edge_Point2[2],
                    0,
                    0,
                    0,
                ],  # End P2 (top-left) - complete the loop
            ]
            print(
                "Edge coverage: P2 → P3 → P4 → P1 → P2 (same start position regardless of orientation)"
            )

        if movement_mode == "rect":
            zigzag_coords = [
                [modified_Point1[0], modified_Point1[1], z_zigzag, 0, 0, 0],
                [modified_Point2[0], modified_Point2[1], z_zigzag, 0, 0, 0],
                [modified_Point3[0], modified_Point3[1], z_zigzag, 0, 0, 0],
                [modified_Point4[0], modified_Point4[1], z_zigzag, 0, 0, 0],
                [modified_Point1[0], modified_Point1[1], z_zigzag, 0, 0, 0],
            ]
        elif orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.floor(yinner / innerSandingOffset)
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag rows from top (max y) to bottom (min y)
                # Starts at top-left (x_min, y_max)
                while offset <= yinner + 1e-9:
                    current_y = y_max - offset
                    row_points = [
                        [
                            x_min,
                            current_y,
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],  # Start at x_min (left side)
                        [
                            x_max,
                            current_y,
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],  # Go to x_max (right side)
                    ]
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
        else:
            # Vertical orientation - starts at top-right and goes left
            # This matches where edge coverage ends (P2 = top-right)
            # Uses bounding box values like horizontal does
            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1  # Prevent division by zero
                adjusted_step = xinner / num_steps
                print(f"[Vertical] xinner={xinner}, num_steps={num_steps}, adjusted_step={adjusted_step}")

                offset = 0.0
                toggle = 0

                # Build zigzag path from right to left (starting at x_max)
                # Starts at top-right to match edge coverage end point (P2)
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    current_x = x_max - offset
                    row_points = [
                        [
                            current_x,
                            y_max,
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],  # Start at y_max (top)
                        [
                            current_x,
                            y_min,
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],  # Go to y_min (bottom)
                    ]
                    # Reverse every other column to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                print(f"[Vertical] WARNING: xinner={xinner} is not > 0, no zigzag points generated")

        # Update coordinates to absolute values for edge coverage
        for point in edge_coverage_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        # Update only the y coordinate to its absolute value for zigzag
        for point in zigzag_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        if movement_mode == "rect":
            prepoint = [
                abs(zigzag_coords[0][0]) + 0.5,
                zigzag_coords[0][1],
                z_zigzag,
                0,
                0,
                0,
            ]
        elif orientation_mode == "horizontal":
            # Horizontal: edge coverage starts at P2 (top-left), zigzag starts at top-left
            if edge_coverage:
                prepoint = [
                    abs(modified_Point2[0]) + 0.5,
                    abs(modified_Point2[1]),
                    z_zigzag,
                    0,
                    0,
                    0,
                ]
            else:
                prepoint = [abs(x_min) + 0.5, y_max, z_zigzag, 0, 0, 0]
        else:
            # Vertical: edge coverage starts at same position as horizontal (P2/top-left)
            if edge_coverage:
                prepoint = [
                    abs(modified_Point2[0]) + 0.5,
                    abs(modified_Point2[1]),
                    z_zigzag,
                    0,
                    0,
                    0,
                ]
            else:
                prepoint = [
                    abs(modified_Point4[0]) + 0.5,
                    abs(modified_Point4[1]),
                    z_zigzag,
                    0,
                    0,
                    0,
                ]

        if edge_coverage:
            print(
                f"Edge coverage: {len(edge_coverage_coords)} points (MoveL), Zigzag: {len(zigzag_coords)} points (spiral)"
            )

    return edge_coverage_coords, zigzag_coords, prepoint


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


def smalldoor1zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor1zizagsmall(force, z, cps):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(1)
        print("x1:", x1)
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points

        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        edge_coverage_pathp1, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, edge_points, zigzag_points, force):
            # Vibration on

            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )
            print("Turned Vibration On")

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if edge_points and len(edge_points) > 0:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                turn_vibration_on(cps)
                for point in edge_points:
                    if point == edge_points[-1]:
                        # For the first point, wait for the move to complete
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=True,
                        )
                    else:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=False,
                        )
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
            turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Calculate boundaries from zigzag points for spiral clamping
            x_bounds = None
            y_bounds = None
            if zigzag_points and len(zigzag_points) > 0:
                all_x = [p[0] for p in zigzag_points]
                all_y = [p[1] for p in zigzag_points]
                x_bounds = (min(all_x), max(all_x))
                y_bounds = (min(all_y), max(all_y))
                print(f"[Spiral] Boundary clamping: x={x_bounds}, y={y_bounds}")

            # Move to first zigzag point to ensure proper transition from edge coverage
            if zigzag_points and len(zigzag_points) > 0:
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True,
                )

            for index, _ in enumerate(zigzag_points):
                point_A = zigzag_points[index]
                if index + 1 >= len(zigzag_points):
                    break
                point_B = zigzag_points[index + 1]

                # communicate(
                #     cps=cps,
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
                    x_bounds=x_bounds,
                    y_bounds=y_bounds,
                )
                if not success:
                    push_failed = True
                    break

                path_initialized = True
                # waitForBlending(cps=cps, config=config)
                total_count += count

            if path_initialized and not push_failed:
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release force
            releaseForce(cps=cps, config=config)

        # Original sequence with dynamic variables
        communicate(
            cps=cps,
            config=config,
            seventh=x1,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps,
            config=config,
            point=prepointp1,  # Dynamic prepoint
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        perform_process_top(
            cps,
            config,
            edge_points=edge_coverage_pathp1,
            zigzag_points=zigzag_pathp1,
            force=force,
        )  # Edge coverage + zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )

    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data["ylen"]

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor1zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor2zizag(
    force, z, cps, orientation="horizontal", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor2zizagsmall(force, z, cps):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(2)
        print("x1:", x1)
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        edge_coverage_pathp1, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, edge_points, zigzag_points, force):
            # Vibration on

            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if edge_points and len(edge_points) > 0:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                turn_vibration_on(cps)
                for point in edge_points:
                    if point == edge_points[-1]:
                        print("Last point of edge coverage, setting wait=True")
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=True,
                        )
                    else:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=False,
                        )
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
            turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab2"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Calculate boundaries from zigzag points for spiral clamping
            x_bounds = None
            y_bounds = None
            if zigzag_points and len(zigzag_points) > 0:
                all_x = [p[0] for p in zigzag_points]
                all_y = [p[1] for p in zigzag_points]
                x_bounds = (min(all_x), max(all_x))
                y_bounds = (min(all_y), max(all_y))
                print(f"[Spiral] Boundary clamping: x={x_bounds}, y={y_bounds}")

            # Move to first zigzag point to ensure proper transition from edge coverage
            if zigzag_points and len(zigzag_points) > 0:
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True,
                )

            for index, _ in enumerate(zigzag_points):
                point_A = zigzag_points[index]
                if index + 1 >= len(zigzag_points):
                    break
                point_B = zigzag_points[index + 1]

                # communicate(
                #     cps=cps,
                #     config=config,
                #     point=point_A,
                #     tcp=config['coords']['tcptool1plane1'],
                #     ucs=config['coords']['ucsTable1'],
                #     seventh=-1,
                #     speed=0.6,
                #     wait=True
                # )

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
                    x_bounds=x_bounds,
                    y_bounds=y_bounds,
                )
                if not success:
                    push_failed = True
                    break

                path_initialized = True
                # waitForBlending(cps=cps, config=config)
                total_count += count

            if path_initialized and not push_failed:
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release force
            releaseForce(cps=cps, config=config)

        # Original sequence with dynamic variables
        communicate(
            cps=cps,
            config=config,
            seventh=x1,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps,
            config=config,
            point=prepointp1,  # Dynamic prepoint
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        perform_process_top(
            cps,
            config,
            edge_points=edge_coverage_pathp1,
            zigzag_points=zigzag_pathp1,
            force=force,
        )  # Edge coverage + zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )

    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data["ylen"]

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor2zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor3zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor3zizagsmall(force, z, cps):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(3)
        print("x1:", x1)
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        edge_coverage_pathp1, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, edge_points, zigzag_points, force):
            # Vibration on

            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if edge_points and len(edge_points) > 0:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                turn_vibration_on(cps)
                for point in edge_points:
                    if point == edge_points[-1]:
                        print("Last point of edge coverage, setting wait=True")
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=True,
                        )
                    else:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=False,
                        )
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
            turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab3"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Calculate boundaries from zigzag points for spiral clamping
            x_bounds = None
            y_bounds = None
            if zigzag_points and len(zigzag_points) > 0:
                all_x = [p[0] for p in zigzag_points]
                all_y = [p[1] for p in zigzag_points]
                x_bounds = (min(all_x), max(all_x))
                y_bounds = (min(all_y), max(all_y))
                print(f"[Spiral] Boundary clamping: x={x_bounds}, y={y_bounds}")

            # Move to first zigzag point to ensure proper transition from edge coverage
            if zigzag_points and len(zigzag_points) > 0:
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True,
                )

            for index, _ in enumerate(zigzag_points):
                point_A = zigzag_points[index]
                if index + 1 >= len(zigzag_points):
                    break
                point_B = zigzag_points[index + 1]

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
                    x_bounds=x_bounds,
                    y_bounds=y_bounds,
                )
                if not success:
                    push_failed = True
                    break

                path_initialized = True
                total_count += count

            if path_initialized and not push_failed:
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release force
            releaseForce(cps=cps, config=config)

        # Original sequence with dynamic variables
        communicate(
            cps=cps,
            config=config,
            seventh=x1,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps,
            config=config,
            point=prepointp1,  # Dynamic prepoint
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        perform_process_top(
            cps,
            config,
            edge_points=edge_coverage_pathp1,
            zigzag_points=zigzag_pathp1,
            force=force,
        )  # Edge coverage + zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )

    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data["ylen"]

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor3zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


def smalldoor4zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    apply_spiral_settings(spiral_settings)

    def smalldoor4zizagsmall(force, z, cps):
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        # Set up logger
        config["logger"] = setup_logger(config["settings"]["debug"])

        # Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        # Main Points
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(4)
        print("x1:", x1)
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]
        # #Depth of the poocket z
        # z=-6.5
        # zigzag points
        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

        # Global variables
        # prepoint = None
        # zigzag_coords = []

        # Second Pocket 1st Cycle
        edge_coverage_pathp1, zigzag_pathp1, prepointp1 = generate_zigzag_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=17,
            innerOffsetX=17,
            orientation=orientation,
            movement=movement,
            innerSandingOffset=50,
            edge_coverage=True,  # Enable edge coverage with MoveL before spiral
        )
        print("edge_coverage_pathp1=", edge_coverage_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, edge_points, zigzag_points, force):
            # Vibration on

            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config["coords"]["tcptool1plane1"],
                ucs=config["coords"]["ucsTable1"],
                config=config,
            )

            # Step 1: Edge coverage with MoveL (linear path between modified points)
            if edge_points and len(edge_points) > 0:
                print(
                    f"[Edge Coverage] Starting linear MoveL for {len(edge_points)} edge points"
                )
                turn_vibration_on(cps)
                for point in edge_points:
                    if point == edge_points[-1]:
                        print("Last point of edge coverage, setting wait=True")
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=True,
                        )
                    else:
                        communicate(
                            cps=cps,
                            config=config,
                            point=point,
                            tcp=config["coords"]["tcptool1plane1"],
                            ucs=config["coords"]["ucsTable1"],
                            seventh=-1,
                            speed=float(json_config["sandingSpeed"]),
                            wait=False,
                        )
                # Wait for edge coverage to complete before starting spiral
                waitForBlending(cps=cps, config=config)
                print("[Edge Coverage] Completed linear edge path")
            turn_vibration_off(cps)

            # Step 2: Zigzag/Spiral motion
            spiral_track_name = "small_door_tab4"
            path_initialized = False
            push_failed = False
            total_count = 0

            # Calculate boundaries from zigzag points for spiral clamping
            x_bounds = None
            y_bounds = None
            if zigzag_points and len(zigzag_points) > 0:
                all_x = [p[0] for p in zigzag_points]
                all_y = [p[1] for p in zigzag_points]
                x_bounds = (min(all_x), max(all_x))
                y_bounds = (min(all_y), max(all_y))
                print(f"[Spiral] Boundary clamping: x={x_bounds}, y={y_bounds}")

            # Move to first zigzag point to ensure proper transition from edge coverage
            if zigzag_points and len(zigzag_points) > 0:
                print("[Spiral] Moving to first zigzag point:", zigzag_points[0])
                communicate(
                    cps=cps,
                    config=config,
                    point=zigzag_points[0],
                    tcp=config["coords"]["tcptool1plane1"],
                    ucs=config["coords"]["ucsTable1"],
                    seventh=-1,
                    speed=float(json_config["sandingSpeed"]),
                    wait=True,
                )

            for index, _ in enumerate(zigzag_points):
                point_A = zigzag_points[index]
                if index + 1 >= len(zigzag_points):
                    break
                point_B = zigzag_points[index + 1]

                print("Spiral move from A to B:", point_A, "->", point_B)
                success, count = run_spiral_between_points(
                    cps=cps,
                    config=config,
                    start_pose=point_A,
                    end_pose=point_B,
                    radius=12.0,
                    angle_step_deg=45.0,
                    track_name=spiral_track_name,
                    velocity=300.0,
                    accel=500.0,
                    jerk=10000.0,
                    init_path=not path_initialized,
                    orientation=orientation,
                    x_bounds=x_bounds,
                    y_bounds=y_bounds,
                )
                if not success:
                    push_failed = True
                    break

                path_initialized = True
                total_count += count

            if path_initialized and not push_failed:
                timeout = compute_timeout(
                    total_points=total_count, velocity=300.0 * 10.0 / 45.0
                )
                finalize_spiral_path(
                    cps,
                    spiral_track_name,
                    box_id=0,
                    robot_id=0,
                    completion_timeout=timeout,
                )

            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            # Release force
            releaseForce(cps=cps, config=config)

        # Original sequence with dynamic variables
        communicate(
            cps=cps,
            config=config,
            seventh=x1,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps,
            config=config,
            point=prepointp1,  # Dynamic prepoint
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        # # turn_vibration_on(cps)
        perform_process_top(
            cps,
            config,
            edge_points=edge_coverage_pathp1,
            zigzag_points=zigzag_pathp1,
            force=force,
        )  # Edge coverage + zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps,
            config=config,
            point=prehoming,
            tcp=config["coords"]["tcptool1plane1"],
            ucs=config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )

    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data["ylen"]

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):
        smalldoor4zizagsmall(force, z, cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")


if __name__ == "__main__":
    smalldoor1zizag(force=5, z=-6.5)
    smalldoor2zizag(force=5, z=-6.5)
    smalldoor3zizag(force=5, z=-6.5)
    # smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
