from __future__ import annotations

import argparse
import sys
from typing import Sequence

from Components.RobotInteractions import RobotInteractions, load_config

DEFAULT_BOX_ID = 0
DEFAULT_ROBOT_ID = 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Command parser for direct robot interactions."
    )
    parser.add_argument(
        "--config",
        default="configs/config.yaml",
        help="Path to YAML config with controller connection info.",
    )
    parser.add_argument(
        "--box-id",
        type=int,
        default=DEFAULT_BOX_ID,
        help="Control box id (default: 0).",
    )
    parser.add_argument(
        "--robot-id",
        type=int,
        default=DEFAULT_ROBOT_ID,
        help="Robot id (default: 0).",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    movel = subparsers.add_parser(
        "MoveL",
        aliases=["movel", "moveL"],
        help="Linear move to a cartesian pose.",
    )
    movel.add_argument(
        "point",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="Target cartesian pose.",
    )
    movel.add_argument(
        "--raw-acs",
        nargs=6,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Target joint pose; defaults to zeros if omitted.",
    )
    movel.add_argument("--tcp", help="TCP name; defaults to coords.tcpDefault.")
    movel.add_argument("--ucs", help="UCS name; defaults to coords.ucsDefault.")
    movel.add_argument("--speed", type=float, help="Linear/orientation speed.")
    movel.add_argument("--acc", type=float, help="Acceleration.")
    movel.add_argument("--radius", type=float, help="Blend radius.")
    movel.add_argument(
        "--seek",
        action="store_true",
        help="Stop when seek-bit equals seek-state.",
    )
    movel.add_argument(
        "--seek-bit",
        type=int,
        default=0,
        help="Digital input bit index used for seek.",
    )
    movel.add_argument(
        "--seek-state",
        type=int,
        default=0,
        help="Digital input state to stop on when seek is enabled.",
    )
    movel.add_argument(
        "--cmd-id",
        default="0",
        help="Identifier attached to the waypoint.",
    )
    movel.add_argument(
        "--wait",
        dest="wait",
        action="store_true",
        default=True,
        help="Wait for the motion to complete (default).",
    )
    movel.add_argument(
        "--no-wait",
        dest="wait",
        action="store_false",
        help="Return immediately after sending the command.",
    )
    movel.add_argument(
        "--wait-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for motion completion; <=0 disables the timeout.",
    )
    movel.set_defaults(handler=run_move_l)
    return parser


def run_move_l(robot: RobotInteractions, args) -> int:
    success = robot.move_l(
        box_id=args.box_id,
        robot_id=args.robot_id,
        point=args.point,
        raw_acs=args.raw_acs,
        tcp=args.tcp,
        ucs=args.ucs,
        speed=args.speed,
        acc=args.acc,
        radius=args.radius,
        seek=args.seek,
        seek_bit=args.seek_bit,
        seek_state=args.seek_state,
        cmd_id=args.cmd_id,
        wait=args.wait,
        wait_timeout=args.wait_timeout,
    )
    return 0 if success else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        config = load_config(args.config)
    except FileNotFoundError:
        print(f"Config file not found: {args.config}", file=sys.stderr)
        return 1

    robot = RobotInteractions(config=config)
    if not robot.cps.HRIF_IsConnected(args.box_id):
        robot.logger.error("Failed to establish connection to the robot controller.")
        return 1

    handler = getattr(args, "handler", None)
    if handler is None:
        parser.error("No command handler configured.")

    try:
        return handler(robot, args)
    finally:
        try:
            robot.cps.HRIF_DisConnect(args.box_id)
        except Exception:
            robot.logger.debug("Failed to disconnect cleanly", exc_info=True)


if __name__ == "__main__":
    sys.exit(main())
