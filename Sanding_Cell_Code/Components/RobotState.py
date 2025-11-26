from __future__ import annotations

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
            "TCP": {"active": None, "available": ["TCP_Laser", "TCP_tool1", "TCP_toolVert", "TCP_tool1_real", "TCP_laserplane1", "TCP_tool1plane1",  "TCP_tool2plane1", "TCP_tool3plane1"]},
            "UCS": {"active": None, "available": ['Base', 'Plane_1', 'Plane_2']},
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

        # TCP lookup helpers; map coordinates <-> name
        self.coord_to_TCP: dict[tuple[float, ...], str] = {}
        self.tcp_by_name: dict[str, tuple[float, ...]] = {}

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
        self.state["tcp_frame"] = tcp_frame
        self.state["ucs_frame"] = ucs_frame

        return True

    def __fetch_and_update_tcp(self, TcpName) -> bool:
        """
        Docstring for fetch_and_update_tcp

        :param self: Description
        :return: Description
        :rtype: bool
        """

        result: list[str] = []
        nRet = self.cps.HRIF_ReadTCPByName(0,0, TcpName, result)
        if nRet != 0:
            self.logger.error(f"[RobotState] HRIF_ReadTCPByName ERROR: {nRet}")
            return False

        try:
            coords_tuple = self._normalize_coords(result)
        except (TypeError, ValueError) as exc:
            self.logger.error(f"[RobotState] Unable to parse TCP '{TcpName}': {exc}")
            return False

        self.coord_to_TCP[coords_tuple] = TcpName
        self.tcp_by_name[TcpName] = coords_tuple
        self.logger.info(f"[RobotState] HRIF_ReadTCPByName SUCCESS: {TcpName} : {coords_tuple}")

        return True

    def fetch_and_update_tcp(self) -> bool:
        """
        Docstring for fetch_and_update_tcp

        :param self: Description
        :return: Description
        :rtype: bool
        """

        self.coord_to_TCP.clear()
        self.tcp_by_name = {}

        for TCP in self.state["TCP"]["available"]:
            if not self.__fetch_and_update_tcp(TCP):
                return False

        result: list[str] = []
        # Read tool coordinates
        nRet = self.cps.HRIF_ReadCurTCP(0,0,result)
        if nRet != 0:
            self.logger.error(f"[RobotState] HRIF_ReadCurTCP ERROR: {nRet}")
            return False

        try:
            active_coords = self._normalize_coords(result)
        except (TypeError, ValueError) as exc:
            self.logger.error(f"[RobotState] Failed to parse HRIF_ReadCurTCP payload: {exc}")
            self.state["TCP"]["active"] = None
            return False

        active_name = self._find_matching_tcp_name(active_coords)
        self.state["TCP"]["active"] = active_name
        if active_name:
            self.logger.info(f"[RobotState] HRIF_ReadCurTCP SUCCESS: Active TCP : {active_name}")
        else:
            self.logger.warning("[RobotState] HRIF_ReadCurTCP returned coords that did not match a known TCP")

        return True

    def _normalize_coords(self, coords) -> tuple[float, ...]:
        """Convert raw CPS coordinate list into a hashable tuple of floats."""
        return tuple(float(val) for val in list(coords)[:6])

    def _find_matching_tcp_name(self, active_coords: tuple[float, ...]) -> str | None:
        """Find a TCP name that matches the active coordinates (with a small tolerance)."""
        direct_match = self.coord_to_TCP.get(active_coords)
        if direct_match:
            return direct_match

        for coords, name in self.coord_to_TCP.items():
            if len(coords) != len(active_coords):
                continue
            if all(abs(a - b) < 1e-3 for a, b in zip(coords, active_coords)):
                return name
        return None

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
        print(self.format_status_line(), end="", flush=True)



def main():
    robot_state = RobotState()
    while True:
        if not robot_state.update_pos():
            break
        robot_state.print_status()


if __name__ == "__main__":
    main()
