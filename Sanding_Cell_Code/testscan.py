# bottomcycle
import time
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient
import math

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

class RobotInteractions:
    
    def __init__(self):
        # Load configuration with console logging enabled
        self.config = load_config()
        self.config['logger'] = setup_logger(self.config['settings']['debug'])
        self.cps = CPSClient()
        self.initialize_comms()
        pass 

    def initialize_comms(self):
        print("Initializing communications...")
        # Connect to robot
        IP = self.config['server']['cpip']
        port = self.config['server']['cps']
        ret = self.cps.HRIF_Connect(0, IP, port)

        if ret != 0:
            print(f"Error {ret}, Failed to connect to robot.")
        else:
            print("Successfully connected to robot.")

class RobotState(RobotInteractions):
    """
    Lightweight robot state helper.
    All data is stored in self.state.
    """

    def __init__(self):
        super().__init__()

        self.state = {
            # UCS Positions
            "UCS" : [],
            # TCP Configurations
            "TCP" : [],

            # Digital states
            "DigitalIOs": {
                "D01" : False,
                "D02" : False,
                "D03" : False,
                "D04" : False,
                "D05" : False,
                "D06" : False,
                "D07" : False,
                "C01" : False,
                "C02" : False,
                "C03" : False,
                "C04" : False,
                "C05" : False,
                "C06" : False,
                "C07" : False,
                "DI1" : False,
                "DI2" : False,
                "DI3" : False,
                "DI4" : False,
                "DI5" : False,
                "DI6" : False,
                "DI7" : False,
                "CI1" : False,
                "CI2" : False,
                "CI3" : False,
                "CI4" : False,
                "CI5" : False,
                "CI6" : False,
                "CI7" : False,
                },
            
            # Joint positions
            "cartesian_position" : [],
            "robot_rail_position" : 0.0,
            "joints_position" : [],

            # Misc
            "flags" : {},
        }
    
    # ------------------------------------------------------------------ #
    # Robot state flags (HRIF_ReadRobotState)
    # ------------------------------------------------------------------ #
    def update_flags(self) -> bool:
        """Update robot status flags into self.state['flags']."""
        result = []
        nRet = self.cps.HRIF_ReadRobotState(0, 0, result)

        if nRet != 0:
            self.config["logger"].error(f"[RobotState] HRIF_ReadRobotState ERROR: {nRet}")
            return False

        try:
            flags = {
                "in_motion":       result[0] == "1",
                "enabled":         result[1] == "1",
                "has_error":       result[2] == "1",
                "error_code":      result[3],
                "error_axis":      result[4],
                "lock_state":      result[5] == "1",
                "paused":          result[6] == "1",
                "emergency_stop":  result[7] == "1",
                "light_screen":    result[8] == "1",
                "powered_on":      result[9] == "1",
                "ebox_connected":  result[10] == "1",
                "point_motion_done": result[11] == "1",
                "in_position":     result[12] == "1",
            }
        except Exception as e:
            self.config["logger"].error(f"[RobotState] Parse HRIF_ReadRobotState failed: {e}")
            return False

        self.state["flags"] = flags
        return True

    def update_pose(self):

        # Define the return value-empty list
        result = [ ]
        nRet = self.cps.HRIF_ReadActPos(0,0, result)

        # Read the joint position variable
        dJ1 = float(result[0])
        dJ2 = float(result[1])
        dJ3 = float(result[2])
        dJ4 = float(result[3])
        dJ5 = float(result[4])
        dJ6 = float(result[5])

        # Read the spatial position variable
        dX = float(result[6])
        dY = float(result[7])
        dZ = float(result[8])
        dRx = float(result[9])
        dRy = float(result[10])
        dRz = float(result[11])

        # Read the tool coordinate variable
        dTcp_X = float(result[12])
        dTcp_Y = float(result[13])
        dTcp_Z = float(result[14])
        dTcp_Rx = float(result[15])
        dTcp_Ry = float(result[16])
        dTcp_Rz = float(result[17])

        # Read the user coordinate variable
        dUcs_X = float(result[18])
        dUcs_Y = float(result[19])
        dUcs_Z = float(result[20])
        dUcs_Rx = float(result[21])
        dUcs_Ry = float(result[22])
        dUcs_Rz = float(result[23])

        self.state["cartesian_position"] = [ dRx,  dY,  dY,  dRx,  dRy,  dRz]
        

    def update_joints(self, joints_deg):
        """
        joints_deg: iterable of 7 joint values [j1..j7] in degrees
        """
        for i, val in enumerate(joints_deg, start=1):
            self.state["joints"][f"j{i}"] = val

    def update_do(self, do_dict):
        """
        do_dict: e.g. {"do1": 1, "do2": 0, ...}
        """
        self.state["do"].update(do_dict)

    def update_co(self, co_dict):
        self.state["co"].update(co_dict)

    def update_ucs(self, active, available=None):
        self.state["ucs"]["active"] = active
        if available is not None:
            self.state["ucs"]["available"] = list(available)

    def update_tcp(self, active, available=None):
        self.state["tcp"]["active"] = active
        if available is not None:
            self.state["tcp"]["available"] = list(available)

    def format_status_line(self) -> str:
        s = self.state

        # --- unpack main pieces ---
        cp = s["cartesian_position"]      # [x, y, z, rx, ry, rz]
        # jp = s["joints_position"]         # [j1..j6]
        # rail = s["robot_rail_position"]
        # dio = s["DigitalIOs"]
        # flags = s["flags"]

        # --- digital I/O summaries ---
        # do_on = sum(v for k, v in dio.items() if k.startswith("D0"))   # D01..D07
        # co_on = sum(v for k, v in dio.items() if k.startswith("C0"))   # C01..C07
        # di_on = sum(v for k, v in dio.items() if k.startswith("DI"))   # DI1..DI7
        # ci_on = sum(v for k, v in dio.items() if k.startswith("CI"))   # CI1..CI7

        # do_total = len([k for k in dio if k.startswith("D0")])
        # co_total = len([k for k in dio if k.startswith("C0")])
        # di_total = len([k for k in dio if k.startswith("DI")])
        # ci_total = len([k for k in dio if k.startswith("CI")])

        # # --- UCS / TCP counts ---
        # ucs_count = len(s["UCS"])
        # tcp_count = len(s["TCP"])

        # --- flags summary (compact) ---
        # if flags:
        #     # e.g. "in_motion:1 enabled:1 error:0"
        #     flags_str = " ".join(f"{k}:{int(bool(v))}" for k, v in flags.items())
        # else:
        #     flags_str = "-"

        line = (
            # Cartesian pose
            f"XYZ: {cp[0]:7.2f} {cp[1]:7.2f} {cp[2]:7.2f} | "
            # f"RXRYRZ: {cp[3]:7.2f} {cp[4]:7.2f} {cp[5]:7.2f} | "

            # # Rail & joints
            # f"Rail: {rail:7.2f} | "
            # f"J: {jp[0]:6.2f} {jp[1]:6.2f} {jp[2]:6.2f} "
            # f"{jp[3]:6.2f} {jp[4]:6.2f} {jp[5]:6.2f} | "

            # # Frames
            # f"UCS:{ucs_count} TCP:{tcp_count} | "

            # # Digital IO
            # f"DO:{do_on}/{do_total} CO:{co_on}/{co_total} "
            # f"DI:{di_on}/{di_total} CI:{ci_on}/{ci_total} | "

            # # Misc flags
            # f"Flags: {flags_str}"
        )

        return line

    def print_status(self):
        """Prints a single status line that gets overwritten every call."""
        print(self.format_status_line(), end="\r", flush=True)

def main():
    testscript = RobotState()     
    while True:
        testscript.update_pose()
        testscript.print_status()

if __name__ == "__main__":
    main()