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
from modules.CPS import CPSClient


# =====================
# CONFIG + UTIL HELPERS
# =====================

def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _pad(values: List[float]) -> List[float]:
    """Pad any coordinate list to length 6."""
    padded = list(values)[:6]
    while len(padded) < 6:
        padded.append(0.0)
    return padded


def offset_point(base: List[float], dx=0.0, dy=0.0, dz=0.0) -> List[float]:
    """Offset XYZ only (keep RPY intact)."""
    base = _pad(base)
    base[0] += dx
    base[1] += dy
    base[2] += dz
    return base


# =============================
# HELPER: ROBOT STATE + ERRORS
# =============================

def describe_error(cps: CPSClient, box_id: int, code: int) -> str:
    result: List[str] = []
    ret = cps.HRIF_GetErrorCodeStr(box_id, code, result)
    if ret == 0 and result:
        return result[0]
    return f"lookup failed (ret={ret}, raw={result})"


def wait_for_idle(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> bool:
    """Wait until robot state indicates idle."""
    start = time.monotonic()
    while True:
        state: List[str] = []
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            print(f"[wait_for_idle] HRIF_ReadRobotState failed (ret={ret}, state={state})")
            return False

        in_motion = (len(state) > 0 and state[0] == "1")
        if not in_motion:
            return True

        if time.monotonic() - start > timeout:
            print("[wait_for_idle] Timed out waiting for robot to become idle.")
            return False

        time.sleep(0.1)


def wait_for_blending(cps: CPSClient, box_id: int, robot_id: int, timeout: float) -> None:
    """Wait for blending process to finish."""
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

    time.sleep(0.25)  # extra safety wait


# =========================
# BUILDING TEST CASES
# =========================

def build_cases(config: dict, tcp: str | None, ucs: str | None) -> list[dict[str, Any]]:
    coords = config.get("coords", {})
    points = config.get("point", {})

    tcp_name = tcp or coords.get("tcpDefault", "TCP")
    ucs_name = ucs or coords.get("ucsDefault", "Base")

    base_cart = _pad(points.get("safePoint") or [0, 0, 0, 0, 0, 0])
    joint_seed = _pad(points.get("safePointAngle") or [0, 0, 0, 0, 0, 0])

    linear_target = offset_point(base_cart, dx=30.0)
    arc_aux = offset_point(base_cart, dy=20.0)
    arc_end = offset_point(base_cart, dx=30.0, dy=20.0)

    tcp_pose = _pad(points.get("tcpPoseDemo", [0, 0, 0, 0, 0, 0]))
    ucs_pose = _pad(points.get("ucsPoseDemo", [0, 0, 0, 0, 0, 0]))

    rel_mask = [1, 0, 0, 0, 0, 0]
    rel_target = [15.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    return [

        # -----------------------
        # WayPoint (joint move)
        # -----------------------
        {
            "name": "waypoint_joint_mode",
            "fn": "HRIF_WayPoint",
            "args": [
                0,              # type = joint mode
                base_cart,      # EndPos
                joint_seed,     # JointPos
                tcp_name,
                ucs_name,
                80.0,           # Vel
                60.0,           # Acc
                0.0,            # Radius
                1, 0, 0, 0,     # isJoint, isSeek, bit, state
                "wp-joint"
            ],
            "summary": "Joint move using taught joint pose."
        },

        # -----------------------
        # WayPoint (linear move)
        # -----------------------
        {
            "name": "waypoint_linear_cart",
            "fn": "HRIF_WayPoint",
            "args": [
                1,              # type = linear cartesian
                linear_target,
                joint_seed,
                tcp_name,
                ucs_name,
                150.0,
                120.0,
                8.0,            # Radius for blending
                0, 0, 0, 0,
                "wp-linear"
            ],
            "summary": "Linear cartesian move with blending radius."
        },

        # -----------------------
        # WayPointEx
        # -----------------------
        {
            "name": "waypoint_ex_cartesian",
            "fn": "HRIF_WayPointEx",
            "args": [
                1,
                offset_point(base_cart, dz=20.0),
                joint_seed,
                tcp_pose,
                ucs_pose,
                120.0,
                80.0,
                5.0,
                0, 0, 0, 0,
                "wp-ex-cart"
            ],
            "summary": "WaypointEx with explicit TCP/UCS values."
        },

        # -----------------------
        # WayPoint2 (arc move)
        # Corrected for Option A API
        # -----------------------
        {
            "name": "waypoint_arc_transition",
            "fn": "HRIF_WayPoint2",
            "args": [
                arc_end,            # EndPos
                arc_aux,            # AuxPos
                joint_seed,         # AcsPos (joint reference)
                tcp_name,
                ucs_name,
                140.0,              # Vel
                100.0,              # Acc
                10.0,               # Radius
                2,                  # type = arc
                0,                  # isJoint
                0,                  # isSeek
                0,                  # bit
                0,                  # state
                "wp-arc"            # cmdID
            ],
            "summary": "Waypoint2 arc segment via auxiliary point."
        },

        # -----------------------
        # WayPointRel
        # -----------------------
        {
            "name": "waypoint_relative_x",
            "fn": "HRIF_WayPointRel",
            "args": [
                1,                  # MoveType = linear
                0,                  # isJoint
                base_cart,
                joint_seed,
                0,                  # refType
                rel_mask,
                rel_target,
                tcp_name,
                ucs_name,
                100.0,
                80.0,
                5.0,
                0, 0, 0, 0,
                "wp-rel-x"
            ],
            "summary": "Relative cartesian step along +X."
        }
    ]


# =========================
# EXECUTION LOGIC
# =========================

def run_case(cps: CPSClient, box_id: int, robot_id: int,
             case: Dict[str, Any], wait: bool, wait_timeout: float) -> None:

    print(f"\n[{case['name']}] {case['fn']} params:")
    print(f"  summary: {case.get('summary', '')}")
    print(f"  args: {case['args']}")

    if wait and not wait_for_idle(cps, box_id, robot_id, wait_timeout):
        print(f"[{case['name']}] Aborting: robot not idle.")
        return

    fn = getattr(cps, case["fn"])
    params = [box_id, robot_id] + case["args"]

    ret = fn(*params)
    if ret == 0:
        print(f"[{case['name']}] {case['fn']} succeeded.")
    else:
        desc = describe_error(cps, box_id, ret)
        print(f"[{case['name']}] {case['fn']} FAILED: code={ret} ({desc})")

    if wait:
        wait_for_blending(cps, box_id, robot_id, wait_timeout)


# =========================
# MAIN ENTRY POINT
# =========================

def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Exercise waypoint APIs with sample parameter sets.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--box-id", type=int, default=0)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--tcp")
    parser.add_argument("--ucs")
    parser.add_argument("--case", action="append")
    parser.add_argument("--wait", action="store_true")
    parser.add_argument("--wait-timeout", type=float, default=45.0)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--times", type=int, default=1)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    cases = build_cases(cfg, args.tcp, args.ucs)

    if args.case:
        cases = [c for c in cases if c["name"] in args.case]
        if not cases:
            print(f"No matching cases: {args.case}", file=sys.stderr)
            return 1

    if not args.run:
        print("Dry-run mode (add --run to execute). Cases:")
        for c in cases:
            print(f" - {c['name']} ({c['fn']}): {c.get('summary', '')}")
        return 0

    cps = CPSClient()
    ret = cps.HRIF_Connect(args.box_id, cfg["server"]["cpip"], cfg["server"]["cps"])
    if ret != 0:
        print(f"Failed to connect to controller (ret={ret})")
        return 1

    try:
        for case in cases:
            for _ in range(args.times):
                run_case(cps, args.box_id, args.robot_id, case,
                         wait=args.wait, wait_timeout=args.wait_timeout)
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
