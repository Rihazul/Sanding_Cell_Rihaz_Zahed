"""
Clean + Correct Path-L Spiral Sanding Script for Hans Robot CPS API.
Uses HRIF_InitMovePathL → HRIF_PushMovePaths → HRIF_EndPushPathPoints →
HRIF_ReadPathState → HRIF_MovePathL sequence.

Add --run to actually execute robot motion.
"""

from __future__ import annotations
import sys, math, time, argparse, yaml
from pathlib import Path
from typing import List, Tuple, Any, Dict

# ---------------------------------------------------------------
# Import CPSClient from modules/
# ---------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.CPS import CPSClient


def read_current_position(cps: CPSClient, box_id: int, robot_id: int) -> List[float]:
    """
    Reads the current robot TCP cartesian position (X/Y/Z/Rx/Ry/Rz) directly.
    """
    data: List[str] = []
    ret = cps.HRIF_ReadActPos(box_id, robot_id, data)
    # HRIF_ReadActPos returns [joints 0-5][cartesian 6-11][tcp 12-17][ucs 18-23]
    if ret != 0 or len(data) < 12:
        print(f"[ReadPosition] Failed to read cartesian, ret={ret}, data={data}")
        return []

    try:
        return [float(v) for v in data[6:12]]
    except Exception as exc:
        print(f"[ReadPosition] Failed to parse cartesian values: {exc} | data={data}")
        return []


# ============================================================
# Spiral Generator
# ============================================================
def generate_spiral_points(
    center_x: float,
    center_y: float,
    z_height: float,
    max_radius: float,
    step: float,
    inward: bool = False,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 179.99,
) -> Tuple[List[float], Tuple[float, float]]:
    """
    Returns (flat_list_of_points, last_xy)
    Each point = [x, y, z, rx, ry, rz]
    """
    points: List[float] = []
    num_steps = int(max_radius / step)
    x = y = 0

    for i in range(num_steps + 1):
        theta_deg = i * 10
        theta = math.radians(theta_deg)

        r = i * step
        if r > max_radius:
            break

        radius = (max_radius - r) if inward else r

        x = float(center_x + radius * math.cos(theta))
        y = float(center_y + radius * math.sin(theta))

        points.extend([x, y, z_height, rx, ry, rz])

    return points, (x, y)


def generate_line_spiral(
    line_start: Tuple[float, float],
    line_end: Tuple[float, float],
    start_z: float,
    end_z: float,
    radius: float,
    pitch: float,
    angle_step_deg: float = 10.0,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 179.99,
) -> List[float]:
    """
    Create a spiral that follows a straight XY line while moving upward in Z.
    The spiral wraps around the line with constant radius and pitch until end_z.
    Returns a flat list of [x,y,z,rx,ry,rz] waypoints.
    """
    (x0, y0), (x1, y1) = line_start, line_end
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy) or 1.0
    ux, uy = dx / length, dy / length           # unit vector along line
    nx, ny = -uy, ux                            # unit normal for lateral offset

    points: List[float] = []
    theta_deg = 0.0
    z = start_z
    total_height = max(end_z - start_z, 1e-6)

    while z <= end_z:
        t = (z - start_z) / total_height
        cx = x0 + ux * length * t
        cy = y0 + uy * length * t

        theta = math.radians(theta_deg)
        offset_x = radius * math.cos(theta)
        offset_y = radius * math.sin(theta)

        px = cx + offset_x * nx
        py = cy + offset_y * ny

        points.extend([px, py, z, rx, ry, rz])

        theta_deg += angle_step_deg
        z += pitch * (angle_step_deg / 360.0)

    return points


def generate_spiral_between_points(
    start_pose: List[float],
    end_pose: List[float],
    turns: int,
    radius: float,
    pitch_per_turn: float,
    angle_step_deg: float = 10.0,
) -> List[float]:
    """
    Generate a continuous upward spiral that moves its center from point A to point B.
    start_pose/end_pose: [x, y, z, rx, ry, rz]
    Radius is constant; Z increases by pitch_per_turn per revolution.
    """
    x0, y0, z0, rx, ry, rz = start_pose[:6]
    x1, y1, _, _, _, _ = end_pose[:6]

    total_steps = int(turns * (360.0 / angle_step_deg))
    pitch_per_deg = pitch_per_turn / 360.0

    points: List[float] = []
    for step in range(total_steps + 1):
        t = step / total_steps
        cx = x0 + (x1 - x0) * t
        cy = y0 + (y1 - y0) * t
        theta_deg = step * angle_step_deg
        theta = math.radians(theta_deg)
        z = z0 + pitch_per_deg * theta_deg
        px = cx + radius * math.cos(theta)
        py = cy + radius * math.sin(theta)
        points.extend([px, py, z, rx, ry, rz])

    return points


# ============================================================
# Helpers
# ============================================================
def wait_for_path_ready(cps: CPSClient, box_id: int, robot_id: int,
                        track_name: str, timeout: float = 10.0) -> bool:

    start = time.time()
    result = []

    while True:
        ret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, result)

        if ret == 0 and result[2]=="3" and result[3] == "0":
            print("[Path] PATH_READY OK")
            return True

        if time.time() - start > timeout:
            print("[Path] Timeout waiting for PATH_READY:", result)
            return False

        time.sleep(0.1)
        print(f"[Path] Waiting for PATH_READY... | {result}\r")


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int,
                  timeout: float = 60.0) -> bool:

    start = time.time()
    state = []

    while True:
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            print("HRIF_ReadRobotState failed:", ret)
            return False

        in_motion = len(state) > 0 and state[2] == "3" and state[3] == "0"

        if in_motion:
            return True

        if time.time() - start > timeout:
            print("Timeout waiting for idle")
            return False

        time.sleep(0.1)


def run_spiral_line_path(cps: CPSClient, box_id: int, robot_id: int, track: str):
    """
    Execute spiral turns starting from the current pose, moving +400mm in Y.
    Radius = 12mm. Z is held constant (pitch_per_turn = 0).
    """
    start_pose = read_current_position(cps, box_id, robot_id)
    if not start_pose:
        start_pose = [320.04, 332.346, -10.661, 0.0, -0.001, 0.068]

    # Move +400mm along Y, hold Z constant
    end_pose = [start_pose[0], start_pose[1] + 400.0, start_pose[2], start_pose[3], start_pose[4], start_pose[5]]

    turns = 30
    radius = 12.0
    pitch_per_turn = 0.0  # hold Z

    spiral_path = generate_spiral_between_points(
        start_pose=start_pose,
        end_pose=end_pose,
        turns=turns,
        radius=radius,
        pitch_per_turn=pitch_per_turn,
        angle_step_deg=10.0,
    )

    all_points = start_pose + spiral_path

    total_len = len(all_points)
    remainder = total_len % 6
    print(f"[SpiralLine] len(all_points)={total_len}, remainder mod 6 = {remainder}")
    if remainder != 0:
        print("ERROR: Point list misaligned (not divisible by 6).")
        return

    count = total_len // 6
    print(f"[SpiralLine] Total points = {count}")

    ret = cps.HRIF_InitMovePathL(
        box_id, robot_id,
        track,
        100.0,     # velocity
        500.0,     # accel
        10000.0,   # jerk
        "Plane_1",    # UCS
        "TCP_Laser"
    )
    print("[SpiralLine][Init] ret =", ret)
    if ret != 0:
        print("[SpiralLine][Init] aborting due to error", ret)
        return

    print("[SpiralLine][Push] Sending points...")
    ret = cps.HRIF_PushMovePaths(
        box_id, robot_id,
        track,
        1,
        count,
        all_points
    )
    print("[SpiralLine][Push] ret =", ret)
    if ret != 0:
        print("[SpiralLine][Push] aborting due to error", ret)
        return

    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track)
    print("[SpiralLine][EndPush] ret =", ret)
    if ret != 0:
        print("[SpiralLine][EndPush] aborting due to error", ret)
        return

    wait_for_path_ready(cps, box_id, robot_id, track)

    print("[SpiralLine][Move] Starting MovePathL...")
    nRet = cps.HRIF_MovePathL(box_id, robot_id, track)
    print("[SpiralLine][Move] ret =", nRet)

    wait_for_idle(cps, box_id, robot_id)


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int,
                  timeout: float = 60.0) -> bool:

    start = time.time()
    state = []

    while True:
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            print("HRIF_ReadRobotState failed:", ret)
            return False

        in_motion = len(state) > 0 and state[2] == "3" and state[3] == "0"

        if in_motion:
            return True

        if time.time() - start > timeout:
            print("Timeout waiting for idle")
            return False

        time.sleep(0.1)


# ============================================================
# Path-L Sanding Execution
# ============================================================
def run_sanding_path(cps: CPSClient, box_id: int, robot_id: int, track: str):

    # CAUTION: This path assumes the current robot pose is inserted as the first waypoint.
    # base = [-572.28, -12.00, 582.59, 0, 0, 179.99]
    # Plane 1
    current_base = read_current_position(cps, box_id, robot_id)
    base = current_base if current_base else [320.04, 332.346, -10.661, 0.0, -0.001, 0.068]
    # Spiral outward
    spiral1, (x1, y1) = generate_spiral_points(
        base[0], base[1], base[2], max_radius=15.0, step=0.25, inward=False,
        rx=base[3], ry=base[4], rz=base[5]
    )

    # Second spiral slightly shifted
    spiral2, _ = generate_spiral_points(
        x1 + 2, y1 + 2, base[2], max_radius=15.0, step=0.25, inward=False,
        rx=base[3], ry=base[4], rz=base[5]
    )

    # Ensure the current point is the first waypoint in the path
    all_points = base + spiral1 + spiral2

    total_len = len(all_points)
    remainder = total_len % 6
    print(f"[Path] len(all_points)={total_len}, remainder mod 6 = {remainder}")
    if remainder != 0:
        print("ERROR: Point list misaligned (not divisible by 6).")
        return

    count = total_len // 6
    print(f"[Path] Total points = {count}")

    # -----------------------------------------------------------
    # 1. Init path
    # -----------------------------------------------------------
    ret = cps.HRIF_InitMovePathL(
        box_id, robot_id,
        track,
        300.0,     # velocity
        500.0,     # accel
        10000.0,   # jerk
        "Plane_1",    # UCS
        "TCP_Laser"
    )
    print("[Init] ret =", ret)
    if ret != 0:
        print("[Init] aborting due to error", ret)
        return

    # -----------------------------------------------------------
    # 2. Push points
    # -----------------------------------------------------------
    print("[Push] Sending points...")
    ret = cps.HRIF_PushMovePaths(
        box_id, robot_id,
        track,
        1,              # type = 1 (standard path)
        count,
        all_points
    )
    print("[Push] ret =", ret)
    if ret != 0:
        print("[Push] aborting due to error", ret)
        return

    # -----------------------------------------------------------
    # 3. End push
    # -----------------------------------------------------------
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track)
    print("[EndPush] ret =", ret)
    if ret != 0:
        print("[EndPush] aborting due to error", ret)
        return

    # -----------------------------------------------------------
    # 4. Wait for PATH_READY
    # -----------------------------------------------------------
    wait_for_path_ready(cps, box_id, robot_id, track)

    # -----------------------------------------------------------
    # 5. Start movement
    # -----------------------------------------------------------
    print("[Move] Starting MovePathL...")
    nRet = cps.HRIF_MovePathL(box_id, robot_id, track)
    print("[Move] ret =", nRet)

    # -----------------------------------------------------------
    # 6. Wait for completion
    # -----------------------------------------------------------
    wait_for_idle(cps, box_id, robot_id)

    # -----------------------------------------------------------
    # 7. Return to home
    # -----------------------------------------------------------



# ============================================================
# Main
# ============================================================
def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--box_id", type=int, default=0)
    parser.add_argument("--robot_id", type=int, default=0)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--spiral", action="store_true", help="Run line-spiral motion (30 turns, radius 12, pitch 0.10mm)")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    ip = cfg["server"]["cpip"]
    port = cfg["server"]["cps"]

    if not args.run and not args.spiral:
        print("Dry-run mode. Add --run for sanding or --spiral for line-spiral motion.")
        return 0

    cps = CPSClient()
    ret = cps.HRIF_Connect(args.box_id, ip, port)
    print("Connect ret =", ret)
    if ret != 0:
        return 1

    if args.spiral:
        track_name = 'spiral_line1'
    else:
        track_name= 'sanding_path1'

    try:
        if args.spiral:
            run_spiral_line_path(cps, args.box_id, args.robot_id, track_name)
        else:
            run_sanding_path(cps, args.box_id, args.robot_id, track_name)
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
