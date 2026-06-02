import argparse
import os
import time

import yaml

from modules.CPS import CPSClient


def load_cps_endpoint(config_path):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}
    server = cfg.get("server", {})
    return server.get("cpip"), int(server.get("cps"))


def hr_motor(cps, cmd, params):
    out = []
    ret = cps.HRIF_HRApp(0, "HR_Motor", cmd, params, out)
    print(f"{cmd}({params}) -> ret={ret}, out={out}")
    return ret, out


def hr_ok(ret, out):
    if ret not in (0, None):
        return False
    if not isinstance(out, list) or not out:
        return False
    return str(out[0]).strip() == "0"


def is_done(state_out):
    # Accept done only when the payload itself is valid and motion field is 0.
    if not isinstance(state_out, list) or len(state_out) < 3:
        return False
    if str(state_out[0]).strip() != "0":
        return False
    return str(state_out[2]).strip() == "0"


def wait_done(cps, motor, timeout_s=60.0, poll_s=0.05):
    start = time.time()
    while True:
        ret, out = hr_motor(cps, "MotorGetState", [motor])
        if hr_ok(ret, out) and is_done(out):
            return True
        if (time.time() - start) >= timeout_s:
            return False
        time.sleep(poll_s)


def parse_targets(target_text, target_value):
    targets = []
    if target_text:
        for p in target_text.split(","):
            p = p.strip()
            if p:
                targets.append(float(p))
    if target_value is not None:
        targets.append(float(target_value))
    return targets


def main():
    parser = argparse.ArgumentParser(description="Simple J7 MotoApp test")
    parser.add_argument("--motor", default="J7", help="Motor name (default: J7)")
    parser.add_argument(
        "--cmd",
        choices=("position", "position_speed"),
        default="position",
        help="MotoApp move command to test (default: position)",
    )
    parser.add_argument("--speed", type=float, default=200.0, help="mm/s")
    parser.add_argument("--target", type=float, default=None, help="Single target mm")
    parser.add_argument("--targets", default="", help="Comma list, e.g. 0,250,700,0")
    parser.add_argument("--state-only", action="store_true", help="Only read motor state")
    parser.add_argument("--timeout", type=float, default=60.0, help="Wait timeout per move (s)")
    args = parser.parse_args()

    config_path = os.path.join(os.path.dirname(__file__), "configs", "config.yaml")
    host, port = load_cps_endpoint(config_path)
    if not host or not port:
        print("Invalid CPS endpoint in config.yaml")
        return 2

    targets = parse_targets(args.targets, args.target)

    cps = CPSClient()
    ret = cps.HRIF_Connect(0, host, port)
    print(f"HRIF_Connect({host}:{port}) -> ret={ret}")
    if ret != 0:
        return 1

    try:
        ret, out = hr_motor(cps, "MotorConnect", [args.motor])
        if not hr_ok(ret, out):
            print("MotorConnect failed.")
            return 1

        if args.state_only:
            hr_motor(cps, "MotorGetState", [args.motor])
            return 0

        if not targets:
            print("No target provided. Use --target or --targets.")
            return 0

        for i, target in enumerate(targets, start=1):
            if args.cmd == "position":
                print(f"\nMove {i}/{len(targets)}: {args.motor} -> {target} mm")
                ret, out = hr_motor(cps, "MotorMovePosition", [args.motor, target])
            else:
                print(f"\nMove {i}/{len(targets)}: {args.motor} -> {target} mm @ {args.speed} mm/s")
                ret, out = hr_motor(cps, "MotorMovePositionSpeed", [args.motor, target, args.speed])
            if not hr_ok(ret, out):
                print("Move command failed.")
                return 1
            if not wait_done(cps, args.motor, timeout_s=args.timeout):
                print("Timed out waiting for done state.")
                return 1
            print("Done.")

        return 0
    finally:
        cps.HRIF_DisConnect(0)
        print("Disconnected.")


if __name__ == "__main__":
    raise SystemExit(main())
