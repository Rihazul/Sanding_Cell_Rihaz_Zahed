from __future__ import annotations

import argparse
import sys
import time
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


def configure_frames(cps, box_id: int, robot_id: int, ucs_name: str, tcp_name: str, logger) -> bool:
    current_ucs: list[str] = []
    target_ucs: list[str] = []
    ret = cps.HRIF_ReadCurUCS(box_id, robot_id, current_ucs)
    if ret != 0:
        logger.error(f"Failed to read current UCS (code {ret}).")
        return False
    ret = cps.HRIF_ReadUCSByName(box_id, robot_id, ucs_name, target_ucs)
    if ret != 0:
        logger.error(f"Failed to read UCS '{ucs_name}' (code {ret}).")
        return False
    if current_ucs != target_ucs:
        ret = cps.HRIF_SetUCSByName(box_id, robot_id, ucs_name)
        if ret != 0:
            logger.error(f"Could not set UCS to '{ucs_name}' (code {ret}).")
            return False
        time.sleep(0.1)

    current_tcp: list[str] = []
    target_tcp: list[str] = []
    ret = cps.HRIF_ReadCurTCP(box_id, robot_id, current_tcp)
    if ret != 0:
        logger.error(f"Failed to read current TCP (code {ret}).")
        return False
    ret = cps.HRIF_ReadTCPByName(box_id, robot_id, tcp_name, target_tcp)
    if ret != 0:
        logger.error(f"Failed to read TCP '{tcp_name}' (code {ret}).")
        return False
    if current_tcp != target_tcp:
        ret = cps.HRIF_SetTCPByName(box_id, robot_id, tcp_name)
        if ret != 0:
            logger.error(f"Could not set TCP to '{tcp_name}' (code {ret}).")
            return False
        time.sleep(0.1)

    return True


def wait_for_motion_done(cps, box_id: int, robot_id: int, logger, timeout: float | None) -> bool:
    start = time.monotonic()
    while True:
        state: list[str] = []
        ret = cps.HRIF_ReadRobotState(box_id, robot_id, state)
        if ret != 0:
            logger.error(f"Failed to read robot state (code {ret}).")
            return False
        if len(state) > 11 and state[11] == "1":
            return True
        if timeout is not None and timeout > 0 and (time.monotonic() - start) > timeout:
            logger.error("Timed out waiting for motion completion.")
            return False
        time.sleep(0.1)

def describe_error(cps, box_id: int, code: int, logger) -> None:
    """Fetch and log a human-readable error string when possible."""
    result: list[str] = []
    ret = cps.HRIF_GetErrorCodeStr(box_id, code, result)
    if ret == 0 and result:
        logger.error(f"Controller description for error {code}: {result[0]}")
    else:
        logger.error(f"Unable to fetch error description for {code} (ret={ret}, raw={result})")


def log_robot_state(cps, box_id: int, logger) -> None:
    """Log current robot state flags for quick diagnosis."""
    result: list[str] = []
    ret = cps.HRIF_ReadRobotState(box_id, 0, result)
    if ret != 0:
        logger.error(f"Failed to read robot state (code {ret}).")
        return
    if len(result) < 13:
        logger.error(f"Unexpected robot state payload: {result}")
        return
    flags = {
        "in_motion": result[0] == "1",
        "enabled": result[1] == "1",
        "has_error": result[2] == "1",
        "error_code": result[3],
        "error_axis": result[4],
        "lock_state": result[5] == "1",
        "paused": result[6] == "1",
        "emergency_stop": result[7] == "1",
        "light_screen": result[8] == "1",
        "powered_on": result[9] == "1",
        "ebox_connected": result[10] == "1",
        "point_motion_done": result[11] == "1",
        "in_position": result[12] == "1",
    }
    logger.error(f"Robot state flags: {flags}")

def run_move_l(robot: RobotInteractions, args) -> int:
    config = robot.config
    coords = config.get("coords", {})
    cps = robot.cps
    logger = robot.logger

    tcp = args.tcp or coords.get("tcpDefault")
    ucs = args.ucs or coords.get("ucsDefault")
    speed = args.speed if args.speed is not None else coords.get("roboVelocity", 200.0)
    acc = args.acc if args.acc is not None else coords.get("roboAcceleration", 200.0)
    radius = args.radius if args.radius is not None else coords.get("transitionRadius", 0.0)
    raw_acs = args.raw_acs if args.raw_acs is not None else [0.0] * 6
    point = list(args.point)

    if tcp is None:
        logger.error("TCP name is required (provide --tcp or set coords.tcpDefault in config).")
        return 1
    if ucs is None:
        logger.error("UCS name is required (provide --ucs or set coords.ucsDefault in config).")
        return 1
    if len(raw_acs) != 6:
        logger.error("Raw joint pose must contain 6 values.")
        return 1

    logger.info("Setting frames before MoveL...")
    if not configure_frames(cps, args.box_id, args.robot_id, ucs, tcp, logger):
        return 1

    ret = cps.HRIF_MoveL(
        args.box_id,
        args.robot_id,
        point,
        raw_acs,
        tcp,
        ucs,
        speed,
        acc,
        radius,
        1 if args.seek else 0,
        args.seek_bit,
        args.seek_state,
        args.cmd_id,
    )
    if ret != 0:
        logger.error(f"HRIF_MoveL failed with code {ret}.")
        describe_error(cps, args.box_id, ret, logger)
        log_robot_state(cps, args.box_id, logger)
        return 1

    if args.wait:
        timeout = args.wait_timeout if args.wait_timeout > 0 else None
        if not wait_for_motion_done(cps, args.box_id, args.robot_id, logger, timeout):
            return 1

    logger.info("MoveL completed successfully.")
    return 0


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
