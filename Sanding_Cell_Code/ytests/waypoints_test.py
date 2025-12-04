"""
Quick launcher to exercise the waypoint-related CPS APIs with varied cases.

By default this only prints the parameters; add --run to actually send commands.
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


def _pad(values: List[float]) -> List[float]:
    padded = list(values)[:6]
    while len(padded) < 6:
        padded.append(0.0)
    return padded


def offset_point(base: List[float], dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> List[float]:
    padded = _pad(base)
    padded[0] += dx
    padded[1] += dy
    padded[2] += dz
    return padded


def describe_error(cps: CPSClient, box_id: int, code: int) -> str:
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


def build_cases(config: dict, tcp: str | None, ucs: str | None) -> list[dict[str, Any]]:
    coords = config.get("coords", {})
    points_cfg = config.get("point", {})

    tcp_name = tcp or coords.get("tcpDefault", "TCP")
    ucs_name = ucs or coords.get("ucsDefault", "Base")

    base_cart = _pad(points_cfg.get("safePointClose") or points_cfg.get("safePoint") or [0, 0, 0, 0, 0, 0])
    joint_seed = _pad(points_cfg.get("safePointAngle") or [0, 0, 0, 0, 0, 0])

    linear_target = offset_point(base_cart, dx=30.0)
    arc_aux = offset_point(base_cart, dy=20.0)
    arc_end = offset_point(base_cart, dx=30.0, dy=20.0)
    rel_mask = [1, 0, 0, 0, 0, 0]
    rel_target = [15.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    tcp_pose = _pad(points_cfg.get("tcpPoseDemo", [0, 0, 0, 0, 0, 0]))
    ucs_pose = _pad(points_cfg.get("ucsPoseDemo", [0, 0, 0, 0, 0, 0]))

    return [
        {
            "name": "waypoint_joint_mode",
            "fn": "HRIF_WayPoint",
            "args": [0, base_cart, joint_seed, tcp_name, ucs_name, 80.0, 60.0, 0.0, 1, 0, 0, 0, "wp-joint-seed"],
            "summary": "Joint move using taught joint pose.",
        },
        {
            "name": "waypoint_linear_cart",
            "fn": "HRIF_WayPoint",
            "args": [1, linear_target, joint_seed, tcp_name, ucs_name, 150.0, 120.0, 8.0, 0, 0, 0, 0, "wp-linear"],
            "summary": "Linear cartesian move with blending radius.",
        },
        {
            "name": "waypoint_ex_cartesian",
            "fn": "HRIF_WayPointEx",
            "args": [1, offset_point(base_cart, dz=20.0), joint_seed, tcp_pose, ucs_pose, 120.0, 80.0, 5.0, 0, 0, 0, 0, "wp-ex-cart"],
            "summary": "WaypointEx with explicit TCP/UCS values.",
        },
        {
            "name": "waypoint_arc_transition",
            "fn": "HRIF_WayPoint2",
            "args": [arc_end, arc_aux, joint_seed, tcp_name, ucs_name, 140.0, 100.0, 10.0, 2, 0, 0, 0, 0, "wp-arc"],
            "summary": "Waypoint2 arc segment via auxiliary point.",
        },
        {
            "name": "waypoint_relative_x",
            "fn": "HRIF_WayPointRel",
            "args": [1, 0, base_cart, joint_seed, 0, rel_mask, rel_target, tcp_name, ucs_name, 100.0, 80.0, 5.0, 0, 0, 0, 0, "wp-rel-x"],
            "summary": "Relative cartesian step along +X.",
        },
    ]


def run_case(cps: CPSClient, box_id: int, robot_id: int, case: Dict[str, Any], wait: bool, wait_timeout: float) -> None:
    print(f"\n[{case['name']}] {case['fn']} params:")
    print(f"  summary: {case.get('summary', '')}")
    print(f"  args: {case['args']}")

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Aborting because robot is not idle.")
        return

    fn = getattr(cps, case["fn"])
    params = [box_id, robot_id] + case["args"]
    ret = fn(*params)

    if ret == 0:
        print(f"[{case['name']}] {case['fn']} succeeded.")
    else:
        desc = describe_error(cps, box_id, ret)
        print(f"[{case['name']}] {case['fn']} failed: code={ret} ({desc})")

    if wait:
        wait_for_blending(cps, box_id, robot_id, wait_timeout)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Exercise waypoint APIs with sample parameter sets.")
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
            print(f" - {c['name']} ({c['fn']}): {c.get('summary', '')}")
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
