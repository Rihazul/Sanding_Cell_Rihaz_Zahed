from __future__ import annotations

from typing import Iterable

from Components.RobotInteractions import RobotInteractions
from modules.CPS import CPSClient


class RobotState(RobotInteractions):
    """
    Lightweight robot state helper; stores a snapshot in ``self.state``.

    
    Has the following functions:
    Fetch Functions:
    - ``fetch_and_update_pos()``: Fetch and update robot positions from the controller.
    - ``fetch_and_update_flags()``: Fetch and update robot status flags from the controller.

    Print/Logging Functions:
    - ``print_status()``: Print a formatted status line to the console.
    - ``format_status_line()``: Format the current robot status into a string.

    """

    def __init__(self, config: dict | None = None, cps_client: CPSClient | None = None):
        super().__init__(config=config, cps_client=cps_client)
        self.state = {
            "UCS": {"active": None, "available": []},
            "TCP": {"active": None, "available": []},
            "DigitalIOs": {
                "D01": False,
                "D02": False,
                "D03": False,
                "D04": False,
                "D05": False,
                "D06": False,
                "D07": False,
                "C01": False,
                "C02": False,
                "C03": False,
                "C04": False,
                "C05": False,
                "C06": False,
                "C07": False,
                "DI1": False,
                "DI2": False,
                "DI3": False,
                "DI4": False,
                "DI5": False,
                "DI6": False,
                "DI7": False,
                "CI1": False,
                "CI2": False,
                "CI3": False,
                "CI4": False,
                "CI5": False,
                "CI6": False,
                "CI7": False,
            },
            "cartesian_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "robot_rail_position": 0.0,
            "joints_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "flags": {},
        }

    @staticmethod
    def _as_bool(value) -> bool:
        """Convert incoming IO values into booleans."""
        if isinstance(value, str):
            return value.strip().lower() not in ("0", "false", "off", "")
        return bool(value)

    def fetch_and_update_flags(self) -> bool:
        """Update robot status flags into self.state['flags']."""
        result: list[str] = []
        nRet = self.cps.HRIF_ReadRobotState(0, 0, result)

        if nRet != 0:
            self.logger.error(f"[RobotState] HRIF_ReadRobotState ERROR: {nRet}")
            return False

        if len(result) < 13:
            self.logger.error(
                f"[RobotState] HRIF_ReadRobotState returned {len(result)} fields, expected at least 13"
            )
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
        except Exception as exc:
            self.logger.error(f"[RobotState] Parse HRIF_ReadRobotState failed: {exc}")
            return False

        self.state["flags"] = flags
        return True

    def fetch_and_update_pos(self) -> bool:
        """
        
        Fetch and update robot positions from the robot controller.
        Read cartesian, joint, TCP, and UCS positions from the controller.
        
        Returns:
            bool: True if the position update was successful, False otherwise.
        """

        result: list[str] = []
        nRet = self.cps.HRIF_ReadActPos(0, 0, result)
        if nRet != 0:
            self.logger.error(f"[RobotState] HRIF_ReadActPos ERROR: {nRet}")
            return False

        if len(result) < 24:
            self.logger.error(
                f"[RobotState] HRIF_ReadActPos returned {len(result)} fields, expected at least 24"
            )
            return False

        try:
            joints = [float(val) for val in result[0:6]]
            cartesian = [float(val) for val in result[6:12]]
            tcp_frame = [float(val) for val in result[12:18]]
            ucs_frame = [float(val) for val in result[18:24]]
        except (TypeError, ValueError) as exc:
            self.logger.error(f"[RobotState] Failed to parse HRIF_ReadActPos payload: {exc}")
            return False

        self.state["joints_position"] = joints
        self.state["cartesian_position"] = cartesian
        self.state["TCP"] = tcp_frame
        self.state["UCS"] = ucs_frame

        return True

    def format_status_line(self) -> str:
        s = self.state

        # --- unpack main pieces ---
        cp = s.get("cartesian_position") or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if len(cp) < 6:
            cp = list(cp) + [0.0] * (6 - len(cp))
        # jp = s["joints_position"]         # [j1..j6]
        # rail = s["robot_rail_position"]
        # dio = s["DigitalIOs"]
        # flags = s["flags"]

        line = (
            f"XYZ: {cp[0]:7.2f} {cp[1]:7.2f} {cp[2]:7.2f} | "
            f"RXRYRZ: {cp[3]:7.2f} {cp[4]:7.2f} {cp[5]:7.2f}"
        )

        return line

    def print_status(self):
        """Prints a single status line that gets overwritten every call."""
        print(self.format_status_line(), end="\r", flush=True)


def main():
    robot_state = RobotState()
    while True:
        if not robot_state.update_pos():
            break
        robot_state.print_status()


if __name__ == "__main__":
    main()
