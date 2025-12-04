"""
Quick launcher to exercise HRIF_MoveS with a few parameter sets.

This will send real motion commands. Use --run to actually execute them; by
default it only prints what would be sent. It also tries to look up controller
error strings to make debugging easier.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, List

import yaml

# Ensure we can import modules/ when running from ytests/
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.CPS import CPSClient

def waitForBlending(cps : CPSClient):
    result = [False]
    start_time = time.time()
        
    while True:
        nret = cps.HRIF_IsBlendingDone(0,0, result)
        print(f"Blending results : {result}", end='\r')
        if result[0]:
            break
        if time.time() - start_time >= 7:
            # Avoid blocking forever if the controller never reports done.
            break
    print("\n")
    time.sleep(0.3)
    return

def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> bool:
    """Poll robot state until it is no longer moving or times out."""
    start = time.monotonic()
    while True:
        state: List[str] = []
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
    """Resolve an error code to its controller-provided description."""
    result: List[str] = []
    ret = cps.HRIF_GetErrorCodeStr(box_id, code, result)
    if ret == 0 and result:
        return result[0]
    return f"lookup failed (ret={ret}, raw={result})"


def build_cases(config: dict, tcp: str | None, ucs: str | None) -> list[dict[str, Any]]:
    coords = config.get("coords", {})
    tcp_name = tcp or coords.get("tcpDefault", "TCP")
    ucs_name = ucs or coords.get("ucsDefault", "Base")

    # A few representative parameter sets to explore different radii and speeds.
    return [
        {
            "name": "gentle_small",
            "dSpiralIncrement": 1.0,
            "dSpiralDiameter": 5.0,
            "dVelocity": 110.0,
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
            "dVelocity": 110.0,
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
            "dVelocity": 110.0,
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


def run_case(cps: CPSClient, box_id: int, robot_id: int, case: Dict[str, Any], wait: bool, wait_timeout: float) -> None:
    print(f"\n[{case['name']}] Sending HRIF_MoveS with params: {case}")

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Aborting because robot is not idle.")
        return

    # HRIF_MoveS signature (per modules/CPS.py):
    #   HRIF_MoveS(boxID, rbtID, dSpiralIncrement, dSpiralDiameter,
    #              dVelocity, dAcc, dRadius, sTcpName, sUcsName, cmdID)
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


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Exercise HRIF_MoveS with sample parameter sets.")
    parser.add_argument("--config", default="configs/config.yaml", help="YAML config with cpip/cps.")
    parser.add_argument("--box-id", type=int, default=0, help="Control box ID.")
    parser.add_argument("--robot-id", type=int, default=0, help="Robot ID.")
    parser.add_argument("--tcp", help="Override TCP name (defaults to coords.tcpDefault or 'TCP').")
    parser.add_argument("--ucs", help="Override UCS name (defaults to coords.ucsDefault or 'Base').")
    parser.add_argument("--case", action="append", help="Run only named case(s); defaults to all.")
    parser.add_argument("--wait", action="store_true", help="Wait for idle before/after each command.")
    parser.add_argument("--wait-timeout", type=float, default=45.0, help="Seconds to wait for idle.")
    parser.add_argument("--run", action="store_true", help="Actually send commands (dry-run otherwise).")
    parser.add_argument("--times", help="Number of times to run each case.", type=int, default=1)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    cases = build_cases(cfg, args.tcp, args.ucs)
    if args.case:
        cases = [c for c in cases if c["name"] in args.case]
        if not cases:
            print(f"No cases matched {args.case}", file=sys.stderr)
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
        for case in cases:
            for _ in range(times):
                run_case(cps, args.box_id, args.robot_id, case, wait=args.wait, wait_timeout=args.wait_timeout)
                waitForBlending(cps)
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
