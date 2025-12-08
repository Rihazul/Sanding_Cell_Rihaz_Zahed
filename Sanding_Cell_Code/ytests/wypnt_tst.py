import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from modules.CPS import CPSClient

# -----------------------
# FIXED CONTROLLER VALUES
# -----------------------
BOX_ID = 0
ROBOT_ID = 0

ROBOT_IP = "192.168.0.10"   # cpip from your config
ROBOT_PORT = 10003          # cps from your config


def main():
    cps = CPSClient()

    print(f"Connecting to {ROBOT_IP}:{ROBOT_PORT} ...")
    ret = cps.HRIF_Connect(BOX_ID, ROBOT_IP, ROBOT_PORT)
    print("Connect returned =", ret)

    if ret != 0:
        print("❌ Failed to connect — stopping.")
        return

    # -----------------------
    # SIMPLE VALID ARC MOVE
    # -----------------------
    EndPos = [-542.28, 8.0, 582.585, 0, 0, 179.99]
    AuxPos = [-560.00, 12.0, 585.00, 0, 0, 179.99]   # must NOT be collinear
    JointSeed = [0, 0, -75, 0, -105, 90]

    TcpName = "TCP_Laser"
    UcsName = "Base"

    Vel = 140.0
    Acc = 100.0
    Radius = 10.0
    MoveType = 2       # 2 = ARC
    isJoint = 0
    isSeek = 0
    bit = 0
    state = 0
    cmdID = "arc-test"

    print("Sending HRIF_WayPoint2...")

    ret = cps.HRIF_WayPoint2(
        BOX_ID,
        ROBOT_ID,
        EndPos,
        AuxPos,
        JointSeed,
        TcpName,
        UcsName,
        Vel,
        Acc,
        Radius,
        MoveType,
        isJoint,
        isSeek,
        bit,
        state,
        cmdID
    )

    print("HRIF_WayPoint2 returned =", ret)

    cps.HRIF_DisConnect(BOX_ID)
    print("Disconnected.")


if __name__ == "__main__":
    main()
