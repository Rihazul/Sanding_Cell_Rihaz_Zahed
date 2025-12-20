"""
Minimal Y-axis (7th axis) bounce test: read the current position, move
25 mm left/right, and repeat until interrupted.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List

import yaml

# Allow running from ytests/ by adding the repo root to sys.path.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from modules.CPS import CPSClient


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bounce the Y/7th axis left/right by a fixed distance.")
    parser.add_argument("--config", default="configs/config.yaml", help="YAML config with CPS connection info.")
    parser.add_argument("--box-id", type=int, default=0, help="Control box ID.")
    parser.add_argument("--motor-id", default="J7", help="Motor ID/index for Kinco plugin (default: 1).")
    parser.add_argument("--distance", type=float, default=100.0, help="Distance to move left/right in mm.")
    parser.add_argument("--start-pos", type=float, default=0.0, help="Fallback start position if read fails.")
    parser.add_argument("--velocity", type=float, help="Motor speed (defaults to UI.seventhAxisSpeed * seventhAxis.maxSpeed).")
    parser.add_argument("--pause", type=float, default=2.0, help="Pause between moves in seconds.")
    parser.add_argument("--tolerance", type=float, default=0.1, help="Target tolerance in mm.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Seconds to wait for each move.")
    parser.add_argument("--cycles", type=int, help="Optional number of left/right cycles (omit for continuous).")
    return parser.parse_args(argv)


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def describe_error(cc_client: CPSClient, code: int) -> str:
    res: List[str] = []
    ret = cc_client.HRIF_GetErrorCodeStr(0, code, res)
    if ret == 0 and res:
        return res[0]
    return f"(no description, ret={ret}, raw={res})"


def default_velocity(cfg: dict) -> float:
    ui = cfg.get("UI", {})
    seventh = cfg.get("seventhAxis", {})
    speed_pct = ui.get("seventhAxisSpeed")
    max_speed = seventh.get("maxSpeed")
    if speed_pct is None or max_speed is None:
        return 50.0
    return float(speed_pct) / 100.0 * float(max_speed)


def init_position(cc_client: CPSClient) -> float | None:
    ret = cc_client.HRIF_HRApp(0, "HR_Motor", "MotorMovePositionSpeed", ["J7", "500.00", "200.0"], [])
    if ret != 0:
        print(f"[error] MotorReadPosition failed (ret={ret})")
        return  None
    return 500.00


def ensure_motor_connected(cc_client: CPSClient, motor_id: str = "J7") -> bool:
    """Attempt to connect the motor once to clear 208xx errors."""
    res: List[str] = []

    nret = cc_client.HRIF_HRApp(0, 'HR_Motor','GetMotorList', [], [])
    time.sleep(0.2)
    ret = cc_client.HRIF_HRApp(0, 'HR_Motor', 'MotorConnect', ["J7"], res)
    if ret != 0:
        print(f"[warn] MotorConnect({motor_id}) -> {ret} {describe_error(cc_client, ret)}; plugin may not be running.")
        return False
    return True


def move_seventh_axis(cc_client: CPSClient, motor_id: str, target: float, velocity: float, tolerance: float, timeout: float) -> bool:
    """Simplified version of seventhGoToPos: power on, move, poll, then power off."""
    out: List[str] = []

    start = time.monotonic()
    while True:
        cc_client.HRIF_HRApp(0, "HR_Motor", "MotorGetState", ["J7"], out)
        if out[2] == '0':
            break 
        # if time.monotonic() - start > timeout:
        #     print(f"[move] Timed out waiting for {target} (last pos={pos})")
        #     return False
        time.sleep(0.2)

    ret = cc_client.HRIF_HRApp(0, "HR_Motor", "MotorMovePositionSpeed", ["J7", "600.00", "200.0"], [])
    if ret != 0:
        print(f"[move] MotorPowerOn failed (ret={ret}, res={out})")
        return False

    start = time.monotonic()
    while True:
        cc_client.HRIF_HRApp(0, "HR_Motor", "MotorGetState", [motor_id], out)
        if out[2] == '0':
            break 
        # if time.monotonic() - start > timeout:
        #     print(f"[move] Timed out waiting for {target} (last pos={pos})")
        #     return False
        time.sleep(0.2)

    ret = cc_client.HRIF_HRApp(0, "HR_Motor", "MotorMovePositionSpeed", ["J7", "400.00", "200.0"], [])
    if ret != 0:
        print(f"[move] MotorPowerOn failed (ret={ret}, res={out})")
        return False

    start = time.monotonic()
    while True:
        cc_client.HRIF_HRApp(0, "HR_Motor", "MotorGetState", [motor_id], out)
        if out[2] == '0':
            break 
        # if time.monotonic() - start > timeout:
        #     print(f"[move] Timed out waiting for {target} (last pos={pos})")
        #     return False
        time.sleep(0.2)
    
    return True

def bounce_loop(
    cc_client: CPSClient,
    motor_id: str,
    distance: float,
    velocity: float,
    pause: float,
    tolerance: float,
    timeout: float,
    cycles: int | None,
    fallback_start: float,
) -> bool:
    center = init_position(cc_client)
    if center is None:
        print(f"[warn] Using fallback start position: {fallback_start}")
        center = fallback_start
    targets = (center - distance, center + distance)
    print(f"Start @ {center:.2f} mm; bouncing between {targets[0]:.2f} and {targets[1]:.2f}. Ctrl+C to stop.")

    completed = 0
    while cycles is None or completed < cycles:
        for tgt in targets:
            print("Moving...r")
            if not move_seventh_axis(cc_client, motor_id, tgt, velocity, tolerance, timeout):
                return False
            if pause > 0:
                time.sleep(pause)
        completed += 1
    return True


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    cfg = load_config(args.config)
    vel = args.velocity if args.velocity is not None else default_velocity(cfg)

    cps = CPSClient()
    ret = cps.HRIF_Connect(args.box_id, cfg["server"]["cpip"], cfg["server"]["cps"])
    if ret != 0:
        print(f"Failed to connect to controller (ret={ret})")
        return 1

    try:
        if not ensure_motor_connected(cps, args.motor_id):
            print("Kinco plugin not available (HRApp missing). Start the HR_Motor plugin/driver, then retry.")
            return 1
        bounce_loop(
            cps,
            motor_id=args.motor_id,
            distance=args.distance,
            velocity=vel,
            pause=args.pause,
            tolerance=args.tolerance,
            timeout=args.timeout,
            cycles=args.cycles,
            fallback_start=args.start_pos,
        )
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        cps.HRIF_DisConnect(args.box_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
