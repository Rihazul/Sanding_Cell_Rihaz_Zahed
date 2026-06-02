import argparse
import os
import time

import yaml

from modules.CPS import CPSClient


def load_endpoint():
    config_path = os.path.join(os.path.dirname(__file__), "configs", "config.yaml")
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}
    server = cfg.get("server", {})
    return server.get("cpip"), int(server.get("cps"))


def hr_motor(cps, cmd, params):
    out = []
    ret = cps.HRIF_HRAppCmd(0, "HR_Motor", cmd, params, out)
    print(f"{cmd}({params}) -> ret={ret}, out={out}")
    return ret, out


def ok(ret, out):
    if ret not in (0, None):
        return False
    if not isinstance(out, list) or not out:
        return False
    return str(out[0]).strip() == "0"


def state_is_stopped(out):
    if not ok(0, out):
        return False
    if len(out) < 3:
        return False
    return str(out[2]).strip() == "0"


def wait_stopped(cps, motor, timeout):
    start = time.time()
    while (time.time() - start) < timeout:
        ret, out = hr_motor(cps, "MotorGetState", [motor])
        if ok(ret, out) and state_is_stopped(out):
            print("Motor is stopped.")
            return True
        time.sleep(0.05)
    print(f"Timeout after {timeout:.1f}s waiting for stop.")
    return False


def main():
    parser = argparse.ArgumentParser(description="Manual-style J7 MotoApp test")
    parser.add_argument("--motor", default="J7", help="Motor name, default J7")
    parser.add_argument("--target", type=float, help="Target position in mm")
    parser.add_argument("--speed", type=float, default=200.0, help="Speed in mm/s")
    parser.add_argument("--wait", action="store_true", help="Wait until MotorGetState shows stopped")
    parser.add_argument("--timeout", type=float, default=60.0, help="Wait timeout in seconds")

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("connect", help="Run MotorConnect")
    sub.add_parser("state", help="Run MotorGetState")
    sub.add_parser("stop", help="Run MotorStop")
    sub.add_parser("move-position", help="Run MotorMovePosition")
    sub.add_parser("move-position-speed", help="Run MotorMovePositionSpeed")

    args = parser.parse_args()

    host, port = load_endpoint()
    if not host or not port:
        print("Invalid CPS endpoint in config.yaml")
        return 2

    cps = CPSClient()
    ret = cps.HRIF_Connect(0, host, port)
    print(f"HRIF_Connect({host}:{port}) -> ret={ret}")
    if ret != 0:
        return 1

    try:
        if args.command == "connect":
            ret, out = hr_motor(cps, "MotorConnect", [args.motor])
            return 0 if ok(ret, out) else 1

        if args.command == "state":
            ret, out = hr_motor(cps, "MotorGetState", [args.motor])
            return 0 if ok(ret, out) else 1

        if args.command == "stop":
            ret, out = hr_motor(cps, "MotorStop", [args.motor])
            return 0 if ok(ret, out) else 1

        ret, out = hr_motor(cps, "MotorConnect", [args.motor])
        if not ok(ret, out):
            print("MotorConnect failed.")
            return 1

        if args.target is None:
            print("--target is required for move commands.")
            return 2

        if args.command == "move-position":
            ret, out = hr_motor(cps, "MotorMovePosition", [args.motor, args.target])
        else:
            ret, out = hr_motor(
                cps,
                "MotorMovePositionSpeed",
                [args.motor, args.target, args.speed],
            )

        if not ok(ret, out):
            print("Move command failed.")
            return 1

        if args.wait:
            return 0 if wait_stopped(cps, args.motor, args.timeout) else 1
        return 0
    finally:
        cps.HRIF_DisConnect(0)
        print("Disconnected.")


if __name__ == "__main__":
    raise SystemExit(main())
