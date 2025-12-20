"""
Bounce the end effector left/right using MoveL, based on the live cartesian
position read via Components/RobotState.

Defaults: +/-20 mm along the Y axis, continuous loop. Use --cycles to limit.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List

# Allow running from ytests/ by adding the repo root to sys.path.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Components.RobotState import RobotState


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MoveL bounce: +/- distance along an axis using RobotState.")
    parser.add_argument("--distance", type=float, default=100.0, help="Distance in mm to move left/right (default: 20).")
    parser.add_argument(
        "--axis",
        type=int,
        default=1,
        choices=range(3),
        metavar="{0,1,2}",
        help="Axis index to offset (0=X, 1=Y, 2=Z). Default: 1 (Y).",
    )
    parser.add_argument("--cycles", type=int, help="Optional number of left/right cycles. Omit for continuous.")
    parser.add_argument("--pause", type=float, default=0.2, help="Pause between moves (seconds).")
    parser.add_argument("--speed", type=float, default=200.0, help="Override linear speed (mm/s). Defaults to coords.roboVelocity.")
    parser.add_argument("--acc", type=float, default=220.0, help="Override acceleration. Defaults to coords.roboAcceleration.")
    parser.add_argument("--radius", type=float, help="Blend radius. Defaults to coords.transitionRadius.")
    parser.add_argument("--tcp", help="TCP name override.")
    parser.add_argument("--ucs", help="UCS name override.")
    parser.add_argument("--wait-timeout", type=float, default=30.0, help="Seconds to wait for each MoveL to finish.")
    parser.add_argument("--cmd-prefix", default="movel-bounce", help="Command ID prefix for logging/trace.")
    return parser.parse_args(argv)


def get_current_pose(robot: RobotState) -> tuple[List[float], List[float]]:
    """Fetch and return (cartesian, joints) from the controller."""
    if not robot.fetch_and_update_pos():
        raise RuntimeError("Failed to read current pose.")
    cart = list(robot.state.get("cartesian_position", []))
    joints = list(robot.state.get("joints_position", []))
    if len(cart) < 6 or len(joints) < 6:
        raise RuntimeError(f"Unexpected pose lengths: cart={len(cart)}, joints={len(joints)}")
    return cart[:6], joints[:6]


def build_targets(cart: List[float], axis: int, distance: float) -> tuple[List[float], List[float]]:
    left = list(cart)
    right = list(cart)
    left[axis] -= distance
    right[axis] += distance
    return left, right


def bounce(robot: RobotState, args: argparse.Namespace) -> None:
    # Fix endpoints once to avoid drift; bounce between the same two points.
    cart0, joints0 = get_current_pose(robot)
    left, right = build_targets(cart0, args.axis, args.distance)
    targets = (left, right)
    print(f"Start @ {cart0[:3]} -> bouncing between {left[:3]} and {right[:3]}")

    cycle = 0
    while args.cycles is None or cycle < args.cycles:
        for idx, target in enumerate(targets):
            direction = "left" if idx == 0 else "right"
            cmd_id = f"{args.cmd_prefix}-{cycle}-{direction}"
            ok = robot.move_l(
                point=target,
                raw_acs=joints0,
                tcp=args.tcp,
                ucs=args.ucs,
                speed=args.speed,
                acc=args.acc,
                radius=args.radius,
                cmd_id=cmd_id,
                wait=True,
                wait_timeout=args.wait_timeout,
            )
            if not ok:
                raise RuntimeError(f"MoveL {direction} failed.")
            if args.pause > 0:
                time.sleep(args.pause)
        cycle += 1


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    robot: RobotState | None = None
    try:
        robot = RobotState()
        bounce(robot, args)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as exc:
        print(f"Error: {exc}")
        return 1
    finally:
        try:
            # RobotInteractions handles disconnect via CPSClient destructor; be explicit if available.
            if robot is not None and hasattr(robot, "cps"):
                robot.cps.HRIF_DisConnect(0)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
