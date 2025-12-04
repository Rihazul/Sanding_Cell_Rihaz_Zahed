"""
Exercise HRIF_MoveLinearWeave with a few representative parameter sets.

Defaults to dry-run; add --run to actually talk to the controller.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import yaml

# Allow running from ytests/ by adding the repo root to sys.path.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.CPS import CPSClient


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def describe_error(cps: CPSClient, box_id: int, code: int) -> str:
    """Resolve an error code to its controller-provided description."""
    result: List[str] = []
    ret = cps.HRIF_GetErrorCodeStr(box_id, code, result)
    if ret == 0 and result:
        return result[0]
    return f"lookup failed (ret={ret}, raw={result})"


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> bool:
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


def wait_for_blending(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> None:
    """Poll blending-done status to avoid returning while motion is active."""
    result = [False]
    start_time = time.monotonic()
    while True:
        ret = cps.HRIF_IsBlendingDone(box_id, robot_id, result)
        if ret != 0:
            print(f"[wait_for_blending] HRIF_IsBlendingDone failed (ret={ret})")
            break
        if result[0]:
            break
        if time.monotonic() - start_time > timeout:
            print("[wait_for_blending] Timed out waiting for blending to finish.")
            break
        time.sleep(0.2)
    time.sleep(0.25)


def _pad_point(point: List[float]) -> List[float]:
    padded = list(point)[:6]
    while len(padded) < 6:
        padded.append(0.0)
    return padded


def offset_point(base: List[float], dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> List[float]:
    padded = _pad_point(base)
    padded[0] += dx
    padded[1] += dy
    padded[2] += dz
    return padded


def build_cases(config: dict, tcp: str | None, ucs: str | None) -> list[dict[str, Any]]:
    coords = config.get("coords", {})
    points_cfg = config.get("point", {})

    tcp_name = tcp or coords.get("tcpDefault", "TCP")
    ucs_name = ucs or coords.get("ucsDefault", "Base")

    start = _pad_point(points_cfg.get("safePointClose") or points_cfg.get("safePoint") or [0, 0, 0, 0, 0, 0])
    end_x = offset_point(start, dx=40.0)
    end_y = offset_point(start, dy=35.0)
    end_z = offset_point(start, dz=15.0)

    base_vel = coords.get("roboVelocity", 200)
    base_acc = coords.get("roboAcceleration", 200)
    base_radius = coords.get("transitionRadius", 10)

    return [
        {
            "name": "frame0_gentle",
            "StartPoint": start,
            "EndPoint": end_x,
            "dVelocity": base_vel * 0.3,
            "Acc": base_acc * 0.5,
            "dRadius": base_radius * 0.5,
            "dAmplitude": 3.0,
            "dIntervalDistance": 8.0,
            "nWeaveFrameType": 0,
            "dElevation": 0.0,
            "dAzimuth": 0.0,
            "dCentreRise": 0.0,
            "nEnableWaiTime": 0,
            "nPosiTime": 0,
            "nNegaTime": 0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "weave-gentle-x",
        },
        {
            "name": "frame1_with_wait",
            "StartPoint": start,
            "EndPoint": end_y,
            "dVelocity": base_vel * 0.4,
            "Acc": base_acc * 0.6,
            "dRadius": base_radius,
            "dAmplitude": 5.0,
            "dIntervalDistance": 6.0,
            "nWeaveFrameType": 1,
            "dElevation": 4.0,
            "dAzimuth": 15.0,
            "dCentreRise": 0.5,
            "nEnableWaiTime": 1,
            "nPosiTime": 60,
            "nNegaTime": 60,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "weave-y-hold",
        },
        {
            "name": "raised_centre",
            "StartPoint": start,
            "EndPoint": end_z,
            "dVelocity": base_vel * 0.35,
            "Acc": base_acc * 0.5,
            "dRadius": base_radius * 0.4,
            "dAmplitude": 6.0,
            "dIntervalDistance": 9.0,
            "nWeaveFrameType": 0,
            "dElevation": -5.0,
            "dAzimuth": -10.0,
            "dCentreRise": 2.0,
            "nEnableWaiTime": 0,
            "nPosiTime": 0,
            "nNegaTime": 0,
            "sTcpName": tcp_name,
            "sUcsName": ucs_name,
            "sCmdID": "weave-z-rise",
        },
    ]


def run_case(cps: CPSClient, box_id: int, robot_id: int, case: Dict[str, Any], wait: bool, wait_timeout: float) -> None:
    print(f"\n[{case['name']}] HRIF_MoveLinearWeave params:")
    print(case)

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Aborting because robot is not idle.")
        return

    ret = cps.HRIF_MoveLinearWeave(
        box_id,
        robot_id,
        case["StartPoint"],
        case["EndPoint"],
        case["dVelocity"],
        case["Acc"],
        case["dRadius"],
        case["dAmplitude"],
        case["dIntervalDistance"],
        case["nWeaveFrameType"],
        case["dElevation"],
        case["dAzimuth"],
        case["dCentreRise"],
        case["nEnableWaiTime"],
        case["nPosiTime"],
        case["nNegaTime"],
        case["sTcpName"],
        case["sUcsName"],
        case["sCmdID"],
    )

    if ret == 0:
        print(f"[{case['name']}] HRIF_MoveLinearWeave succeeded.")
    else:
        desc = describe_error(cps, box_id, ret)
        print(f"[{case['name']}] HRIF_MoveLinearWeave failed: code={ret} ({desc})")

    if wait:
        wait_for_blending(cps, box_id, robot_id, wait_timeout)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Exercise HRIF_MoveLinearWeave with sample parameter sets.")
    parser.add_argument("--config", default="configs/config.yaml", help="YAML config with cpip/cps.")
    parser.add_argument("--box-id", type=int, default=0, help="Control box ID.")
    parser.add_argument("--robot-id", type=int, default=0, help="Robot ID.")
    parser.add_argument("--tcp", help="Override TCP name (defaults to coords.tcpDefault or 'TCP').")
    parser.add_argument("--ucs", help="Override UCS name (defaults to coords.ucsDefault or 'Base').")
    parser.add_argument("--case", action="append", help="Run only named case(s); defaults to all.")
    parser.add_argument("--wait", action="store_true", help="Wait for idle before/after each command.")
    parser.add_argument("--wait-timeout", type=float, default=45.0, help="Seconds to wait for idle/blending.")
    parser.add_argument("--run", action="store_true", help="Actually send commands (dry-run otherwise).")
    parser.add_argument("--times", type=int, default=1, help="Number of times to run each case.")
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

    try:
        for case in cases:
            for _ in range(args.times):
                run_case(cps, args.box_id, args.robot_id, case, wait=args.wait, wait_timeout=args.wait_timeout)
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
