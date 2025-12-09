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

        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)

        points.extend([x, y, z_height, 0, 0, 179.99])

    return points, (x, y)


# ============================================================
# Helpers
# ============================================================
def wait_for_path_ready(cps: CPSClient, box_id: int, robot_id: int,
                        track_name: str, timeout: float = 10.0) -> bool:

    start = time.time()
    result = []

    while True:
        ret = cps.HRIF_ReadPathState(box_id, robot_id, track_name, result)

        if ret == 0 and result and result[0] == "PATH_READY":
            print("[Path] PATH_READY ✔")
            return True

        if time.time() - start > timeout:
            print("[Path] Timeout waiting for PATH_READY:", result)
            return False

        time.sleep(0.1)


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int,
                  timeout: float = 30.0) -> bool:

    start = time.time()
    state = []

    while True:
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            print("HRIF_ReadRobotState failed:", ret)
            return False

        in_motion = len(state) > 0 and state[0] == "1"

        if not in_motion:
            return True

        if time.time() - start > timeout:
            print("Timeout waiting for idle")
            return False

        time.sleep(0.1)


# ============================================================
# Path-L Sanding Execution
# ============================================================
def run_sanding_path(cps: CPSClient, box_id: int, robot_id: int):

    track = "SandingPath"

    base = [-572.28, -12.00, 582.59, 0, 0, 179.99]

    # Spiral outward
    spiral1, (x1, y1) = generate_spiral_points(
        base[0], base[1], base[2], max_radius=50, step=1.0, inward=False
    )

    # Second spiral slightly shifted
    spiral2, _ = generate_spiral_points(
        x1 + 2, y1 + 2, base[2], max_radius=50, step=1.0, inward=False
    )

    all_points = spiral1 + spiral2

    # Validate point count
    if len(all_points) % 6 != 0:
        print("ERROR: Point list misaligned (not divisible by 6).")
        return

    count = len(all_points) // 6
    print(f"[Path] Total points = {count}")

    # -----------------------------------------------------------
    # 1. Init path
    # -----------------------------------------------------------
    ret = cps.HRIF_InitMovePathL(
        box_id, robot_id,
        track,
        100.0,     # velocity
        500.0,     # accel
        10000.0,   # jerk
        "Base",    # UCS
        "TCP_Laser"
    )
    print("[Init] ret =", ret)

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

    # -----------------------------------------------------------
    # 3. End push
    # -----------------------------------------------------------
    ret = cps.HRIF_EndPushPathPoints(box_id, robot_id, track)
    print("[EndPush] ret =", ret)

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
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    ip = cfg["server"]["cpip"]
    port = cfg["server"]["cps"]

    if not args.run:
        print("Dry-run mode. Add --run to execute robot motion.")
        return 0

    cps = CPSClient()
    ret = cps.HRIF_Connect(args.box_id, ip, port)
    print("Connect ret =", ret)
    if ret != 0:
        return 1

    try:
        run_sanding_path(cps, args.box_id, args.robot_id)
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
