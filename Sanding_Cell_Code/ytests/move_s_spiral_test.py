"""
Quick launcher to exercise HRIF_MoveS and PathL test modes.

This will send real motion commands. Use --run to actually execute them; by
default it only prints what would be sent.
"""

from __future__ import annotations

import argparse
import sys
import math
import time
from typing import Any, Dict, List
import yaml
from pathlib import Path

# Ensure we can import modules/ when running from subfolders
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.CPS import CPSClient


# ============================================================
#  CLEAN SPIRAL GENERATOR (FIXED VERSION)
# ============================================================
def generate_spiral_points(center_x, center_y, z_height, max_radius, step, inward=False):
    """
    Generates spiral points with proper radius increments.
    inward=True: outside → inside
    inward=False: inside → outside
    """
    point_list = []
    x = y = 0

    num_steps = int(max_radius / step)

    for i in range(num_steps + 1):
        # angle grows slowly; ~10° per radial increment
        theta_deg = i * 10
        theta = math.radians(theta_deg)

        r = i * step
        if r > max_radius:
            break

        target_r = (max_radius - r) if inward else r

        x = center_x + target_r * math.cos(theta)
        y = center_y + target_r * math.sin(theta)

        # Build [x,y,z,rx,ry,rz]
        point_list.extend([x, y, z_height, 0, 0, 179.99])

    return point_list, (x, y)



# ============================================================
#  SUPPORT FUNCTIONS
# ============================================================
def get_cuur_version(cps: CPSClient):
    result = []
    nRet = cps.HRIF_ReadVersion(0, 0, result)
    if nRet != 0:
        print("Could not get version")
    print(f"result obtained: {result}")


def waitForBlending(cps: CPSClient):
    result = [False]
    start_time = time.time()

    while True:
        nret = cps.HRIF_IsBlendingDone(0, 0, result)
        print(f"Blending results : {result}", end="\r")

        if result[0]:
            break
        if time.time() - start_time >= 7:
            break

    print("\n")
    time.sleep(0.3)
    return


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> bool:
    start = time.monotonic()
    while True:
        state = []
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            print(f"[wait_for_idle] HRIF_ReadRobotState failed (ret={ret}, state={state})")
            return False

        in_motion = len(state) > 0 and state[0] == "1"
        if not in_motion:
            return True

        if time.monotonic() - start > timeout:
            print("[wait_for_idle] Timed out waiting for robot to become idle.")
            return False

        time.sleep(0.1)


def describe_error(cps: CPSClient, box_id: int, code: int) -> str:
    result = []
    ret = cps.HRIF_GetErrorCodeStr(box_id, code, result)
    if ret == 0 and result:
        return result[0]
    return f"lookup failed (ret={ret}, raw={result})"



# ============================================================
#  CLEAN PATH MODE SANDBOX
# ============================================================
def run_continuous_sanding(cps: CPSClient, box_id: int, robot_id: int):
    track_name = "SandingPath"

    # --- FIXED: Correct InitMovePathL call (no named args)
    cps.HRIF_InitMovePathL(
        box_id,
        robot_id,
        track_name,
        100.0,    # velocity
        500.0,    # acceleration
        10000.0,  # jerk
        "Base",   # UCS
        "TCP_Laser"  # TCP
    )

    # Base point for sanding
    base_pnt = [-572.28, -12.00, 582.59, -0.00, -0.00, 179.99]

    # Spiral 1 (outward)
    spiral1, (x1, y1) = generate_spiral_points(
        center_x=base_pnt[0],
        center_y=base_pnt[1],
        z_height=base_pnt[2],
        max_radius=50,
        step=1.0,
        inward=False
    )

    # Spiral 2 (slightly shifted)
    spiral2, _ = generate_spiral_points(
        center_x=x1 + 2,
        center_y=y1 + 2,
        z_height=base_pnt[2],
        max_radius=50,
        step=1.0,
        inward=False
    )

    all_points = spiral1 + spiral2
    total_points_count = len(all_points) // 6

    cps.HRIF_PushMovePaths(box_id, robot_id, track_name, 1, total_points_count, all_points)

    cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)

    print("Waiting for path calculation...")
    time.sleep(2)  # Optional: replace with HRIF_ReadPathState loop

    print("Starting continuous sanding...")
    nRet = cps.HRIF_MovePathL(box_id, robot_id, track_name)
    print(nRet)



# ============================================================
#  MOVE-S TEST CASES
# ============================================================
def build_cases(config: dict, tcp: str | None, ucs: str | None):
    coords = config.get("coords", {})
    tcp_name = tcp or coords.get("tcpDefault", "TCP")
    ucs_name = ucs or coords.get("ucsDefault", "Base")

    return [
        {
            "name": "gentle_small",
            "dSpiralIncrement": 1.0,
            "dSpiralDiameter": 5.0,
            "dVelocity": 60.0,
            "dAcc": 320.0,
            "dRadius": 80.0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "spiral-2",
        },
        {
            "name": "move_right",
            "dSpiralIncrement": 1.0,
            "dSpiralDiameter": 5.5,
            "dVelocity": 60.0,
            "dAcc": 320.0,
            "dRadius": 80.0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "spiral-2",
        },
        {
            "name": "move_left",
            "dSpiralIncrement": 1.0,
            "dSpiralDiameter": 5.0,
            "dVelocity": 60.0,
            "dAcc": 320.0,
            "dRadius": 80.0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "spiral-2",
        },
        {
            "name": "medium",
            "dSpiralIncrement": 1.5,
            "dSpiralDiameter": 10.0,
            "dVelocity": 80.0,
            "dAcc": 80.0,
            "dRadius": 30.0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "spiral-0",
        },
        {
            "name": "fast_large",
            "dSpiralIncrement": 2.0,
            "dSpiralDiameter": 20.0,
            "dVelocity": 80.0,
            "dAcc": 80.0,
            "dRadius": 40.0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "spiral-3",
        },
    ]



def run_case(cps: CPSClient, box_id: int, robot_id: int, case: Dict[str, Any], wait: bool, wait_timeout: float):
    print(f"\n[{case['name']}] Sending HRIF_MoveS with params: {case}")

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Aborting because robot is not idle.")
        return

    ret = cps.HRIF_MoveS(
        box_id,
        robot_id,
        case["dSpiralIncrement"],
        case["dSpiralDiameter"],
        case["dVelocity"],
        case["dAcc"],
        case["dRadius"],
        case["sTcpName"],
        case["sUcsName"],
        case["sCmdID"],
    )

    if ret == 0:
        print(f"[{case['name']}] HRIF_MoveS succeeded.")
    else:
        desc = describe_error(cps, box_id, ret)
        print(f"[{case['name']}] HRIF_MoveS failed: code={ret} ({desc})")

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Motion did not finish within timeout.")



# ============================================================
#  MAIN ENTRYPOINT
# ============================================================
def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Exercise HRIF_MoveS with sample parameter sets.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--box_id", type=int, default=0)
    parser.add_argument("--robot_id", type=int, default=0)
    parser.add_argument("--tcp", default="TCP_tool3plane1")
    parser.add_argument("--ucs", default="Base")
    parser.add_argument("--case", action="append")
    parser.add_argument("--wait", action="store_true")
    parser.add_argument("--wait-timeout", type=float, default=45.0)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--times", type=int, default=1)
    parser.add_argument("--path_mode", action="store_true")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    cases = build_cases(cfg, args.tcp, args.ucs)

    if args.case:
        cases = [c for c in cases if c["name"] in args.case]
        if not cases:
            print(f"No cases matched {args.case}")
            return 1

    if not args.run:
        print("Dry-run mode (add --run to execute). Cases to run:")
        for c in cases:
            print(f" - {c['name']}: {c}")
        return 0

    cps = CPSClient()
    ip = cfg["server"]["cpip"]
    port = cfg["server"]["cps"]
    ret = cps.HRIF_Connect(args.box_id, ip, port)
    if ret != 0:
        print(f"Failed to connect to controller (ret={ret})")
        return 1

    times = args.times

    try:
        if not args.path_mode:
            for case in cases:
                for _ in range(times):
                    run_case(cps, args.box_id, args.robot_id, case, wait=args.wait, wait_timeout=args.wait_timeout)
                    waitForBlending(cps)
        else:
            run_continuous_sanding(cps, args.box_id, args.robot_id)

    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
