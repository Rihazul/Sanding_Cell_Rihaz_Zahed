import os

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


def main():
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
        hr_motor(cps, "MotorConnect", ["J7"])
        hr_motor(cps, "MotorMovePosition", ["J7", 700])
        return 0
    finally:
        cps.HRIF_DisConnect(0)
        print("Disconnected.")


if __name__ == "__main__":
    raise SystemExit(main())
