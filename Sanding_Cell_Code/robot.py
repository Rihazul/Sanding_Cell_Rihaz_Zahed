from __future__ import annotations

import argparse
import multiprocessing
import os
import sys
from typing import Sequence

from Components.RobotInteractions import RobotInteractions, RobotInteractionsAsync, load_config

DEFAULT_BOX_ID = 0
DEFAULT_ROBOT_ID = 0


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--config",
        default="configs/config.yaml",
        help="Path to YAML config with controller connection info.",
    )
    common.add_argument(
        "--box-id",
        type=int,
        default=DEFAULT_BOX_ID,
        help="Control box id (default: 0).",
    )
    common.add_argument(
        "--robot-id",
        type=int,
        default=DEFAULT_ROBOT_ID,
        help="Robot id (default: 0).",
    )
    common.add_argument(
        "--async",
        dest="async_mode",
        action="store_true",
        help="Run the command via background worker thread.",
    )

    parser = argparse.ArgumentParser(
        description="Command parser for direct robot interactions.",
        parents=[common],
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    movel = subparsers.add_parser(
        "MoveL",
        aliases=["movel", "moveL"],
        help="Linear move to a cartesian pose.",
        parents=[common],
        add_help=False,
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

    for name, handler in (
        ("GrpEnable", run_grp_enable),
        ("GrpDisable", run_grp_disable),
        ("GrpReset", run_grp_reset),
        ("GrpStop", run_grp_stop),
        ("GrpInterrupt", run_grp_interrupt),
        ("GrpContinue", run_grp_continue),
        ("OpenFreeDrive", run_grp_open_free_drive),
        ("CloseFreeDrive", run_grp_close_free_drive),
    ):
        sub = subparsers.add_parser(
            name,
            help=f"Run {name} on the controller.",
            parents=[common],
            add_help=False,
        )
        sub.set_defaults(handler=handler)

    return parser


def run_move_l(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    payload = {
        "box_id": args.box_id,
        "robot_id": args.robot_id,
        "point": args.point,
        "raw_acs": args.raw_acs,
        "tcp": args.tcp,
        "ucs": args.ucs,
        "speed": args.speed,
        "acc": args.acc,
        "radius": args.radius,
        "seek": args.seek,
        "seek_bit": args.seek_bit,
        "seek_state": args.seek_state,
        "cmd_id": args.cmd_id,
        "wait": args.wait,
        "wait_timeout": args.wait_timeout,
    }

    if args.async_mode and async_runner:
        async_runner.submit_move_l(**payload)
        print("MoveL queued for async execution (background thread).")
        return 0

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
        show_progress=args.wait,
    )
    return 0 if success else 1


def _run_simple_group(robot: RobotInteractions, args, method: str, async_runner: RobotInteractionsAsync | None) -> int:
    fn = getattr(robot, method)

    if args.async_mode and async_runner:
        async_runner.submit_callable(method, lambda: fn(box_id=args.box_id, robot_id=args.robot_id))
        print(f"{method} queued for async execution (background thread).")
        return 0

    success = fn(box_id=args.box_id, robot_id=args.robot_id)
    return 0 if success else 1



def run_grp_enable(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_enable", async_runner)

def run_grp_disable(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_disable", async_runner)

def run_grp_reset(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_reset", async_runner)


def run_grp_stop(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_stop", async_runner)


def run_grp_interrupt(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_interrupt", async_runner)


def run_grp_continue(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_continue", async_runner)


def run_grp_open_free_drive(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_open_free_drive", async_runner)


def run_grp_close_free_drive(robot: RobotInteractions, args, async_runner: RobotInteractionsAsync | None = None) -> int:
    return _run_simple_group(robot, args, "grp_close_free_drive", async_runner)


def _execute_command(args) -> int:
    try:
        config = load_config(args.config)
    except FileNotFoundError:
        print(f"Config file not found: {args.config}", file=sys.stderr)
        return 1

    robot = RobotInteractions(config=config)
    async_runner = RobotInteractionsAsync(robot) if getattr(args, "async_mode", False) else None

    if not robot.cps.HRIF_IsConnected(args.box_id):
        robot.logger.error("Failed to establish connection to the robot controller.")
        return 1

    handler = getattr(args, "handler", None)
    if handler is None:
        raise SystemExit("No command handler configured.")

    try:
        return handler(robot, args, async_runner)
    finally:
        try:
            if async_runner:
                async_runner.wait_for_all()
                async_runner.shutdown()
            robot.cps.HRIF_DisConnect(args.box_id)
        except Exception:
            robot.logger.debug("Failed to disconnect cleanly", exc_info=True)


def _run_in_subprocess(args) -> None:
    # Run synchronously in a background process so the shell returns immediately.
    args = argparse.Namespace(**vars(args))
    args.async_mode = False
    # Detach child stdio so the parent terminal stays clean; logs should go to files/UI.
    try:
        devnull = open(os.devnull, "w", encoding="utf-8", errors="ignore")
        sys.stdout = devnull
        sys.stderr = devnull
    except Exception:
        pass
    _execute_command(args)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if getattr(args, "async_mode", False):
        proc = multiprocessing.Process(target=_run_in_subprocess, args=(args,))
        proc.daemon = False
        proc.start()
        print(f"Command running asynchronously in background (pid={proc.pid}). Progress/logs will stream here.")
        return 0

    return _execute_command(args)


if __name__ == "__main__":
    sys.exit(main())

