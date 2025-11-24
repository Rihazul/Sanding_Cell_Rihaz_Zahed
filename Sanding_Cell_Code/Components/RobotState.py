from __future__ import annotations

import asyncio
from typing import Any, Sequence

from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Grid
from textual.widgets import Footer, Header, Static

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


class RobotStateDashboard(App):
    """Textual dashboard that renders the robot state snapshot in vibrant panels."""

    CSS = """
    Screen {
        background: #0f131a;
        color: $text;
        padding: 1;
    }
    #state-grid {
        grid-template-columns: 1fr 1fr;
        gap: 1;
        padding: 0 1;
    }
    .state-panel {
        border: round $accent;
        min-height: 8;
        padding: 1 2;
        background: $surface-dark;
    }
    """

    BINDINGS = [
        ("r", "refresh", "Refresh state"),
        ("q", "quit", "Quit dashboard"),
    ]

    def __init__(self, robot_state: RobotState | None = None, refresh_interval: float = 1.5, **kwargs):
        super().__init__(**kwargs)
        self.robot_state = robot_state or RobotState()
        self.refresh_interval = refresh_interval

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Grid(id="state-grid"):
            yield Static("", id="cartesian-panel", classes="state-panel")
            yield Static("", id="joint-panel", classes="state-panel")
            yield Static("", id="tcp-panel", classes="state-panel")
            yield Static("", id="ucs-panel", classes="state-panel")
            yield Static("", id="dio-panel", classes="state-panel")
            yield Static("", id="flags-panel", classes="state-panel")
        yield Footer()

    async def on_mount(self) -> None:
        self.set_interval(self.refresh_interval, self._refresh_panels)
        await self._refresh_panels()

    async def action_refresh(self) -> None:  # pragma: no cover - UI helper
        """Manual binding that forces a synchronous refresh."""
        await self._refresh_panels()

    async def _refresh_panels(self) -> None:
        await self._refresh_robot_state()
        state = self.robot_state.state or {}
        self.query_one("#cartesian-panel", Static).update(self._render_cartesian(state))
        self.query_one("#joint-panel", Static).update(self._render_joints(state))
        self.query_one("#tcp-panel", Static).update(self._render_frame("TCP Frame", state.get("TCP")))
        self.query_one("#ucs-panel", Static).update(self._render_frame("UCS Frame", state.get("UCS")))
        self.query_one("#dio-panel", Static).update(self._render_digital_io(state.get("DigitalIOs", {})))
        self.query_one("#flags-panel", Static).update(self._render_flags(state.get("flags", {})))

    async def _refresh_robot_state(self) -> None:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._call_update_methods)

    def _call_update_methods(self) -> None:
        for method_name in ("fetch_and_update_pos", "fetch_and_update_flags"):
            method = getattr(self.robot_state, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception as exc:  # pragma: no cover - best effort logging
                    self.log(f"Unable to call {method_name}: {exc}")

    def _render_cartesian(self, state: dict[str, Any]) -> Panel:
        coords = list(state.get("cartesian_position") or [])
        coords += [0.0] * max(0, 6 - len(coords))
        table = Table.grid(expand=True)
        table.add_column()
        table.add_column(justify="right")
        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for label, value in zip(labels, coords[:6]):
            table.add_row(label, f"{value:7.2f}")
        table.add_row("Rail", f"{state.get('robot_rail_position', 0.0):7.3f}")
        return Panel(table, title="Cartesian Position", border_style="cyan")

    def _render_joints(self, state: dict[str, Any]) -> Panel:
        joints = list(state.get("joints_position") or [])
        table = Table.grid(expand=True)
        table.add_column()
        table.add_column(justify="right")
        for idx, value in enumerate(joints[:6], 1):
            table.add_row(f"J{idx}", f"{value:7.2f}")
        if len(joints) < 6:
            for idx in range(len(joints) + 1, 7):
                table.add_row(f"J{idx}", "  0.00")
        return Panel(table, title="Joint Positions", border_style="green")

    def _render_frame(self, title: str, payload: Sequence[float] | dict[str, Any] | None) -> Panel:
        table = Table.grid(expand=True)
        table.add_column()
        table.add_column(justify="right")

        if isinstance(payload, dict):
            active = payload.get("active")
            available = payload.get("available") or []
            active_display = str(active) if active not in (None, "", []) else "—"
            table.add_row("Active", active_display)
            table.add_row("Available", ", ".join(str(item) for item in available) or "none")
        else:
            numbers = list(payload or [])
            labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            for label, value in zip(labels, numbers[:6]):
                table.add_row(label, f"{value:7.2f}")
            if len(numbers) > 6:
                table.add_row("Extra", " ".join(f"{value:7.2f}" for value in numbers[6:]))

        return Panel(table, title=title, border_style="bright_blue")

    def _render_digital_io(self, io_map: dict[str, bool]) -> Panel:
        outputs = {k: v for k, v in io_map.items() if not k.startswith(("DI", "CI"))}
        inputs = {k: v for k, v in io_map.items() if k.startswith(("DI", "CI"))}

        def build_table(title: str, entries: dict[str, bool]) -> Table:
            tbl = Table(title=title, show_header=False, expand=True)
            tbl.add_column()
            tbl.add_column(justify="center")
            if entries:
                for key in sorted(entries):
                    tbl.add_row(key, self._bool_text(entries[key]))
            else:
                tbl.add_row("<empty>", Text("-", style="dim"))
            return tbl

        body = Group(build_table("Outputs", outputs), build_table("Inputs", inputs))
        return Panel(body, title="Digital IOs", border_style="magenta")

    def _render_flags(self, flags: dict[str, Any]) -> Panel:
        table = Table(show_header=False, expand=True)
        table.add_column()
        table.add_column(justify="right")
        if flags:
            for name, value in sorted(flags.items()):
                table.add_row(name.replace("_", " ").title(), self._format_flag_value(value))
        else:
            table.add_row("No flags reported", "")
        return Panel(table, title="Robot Flags", border_style="yellow")

    def _format_flag_value(self, value: Any) -> Text:
        if isinstance(value, bool):
            return self._bool_text(value)
        return Text(str(value), style="bold")

    def _bool_text(self, value: bool) -> Text:
        status = bool(value)
        style = "bold green" if status else "dim"
        return Text("ON" if status else "OFF", style=style)

def main():
    robot_state = RobotState()
    while True:
        if not robot_state.update_pos():
            break
        robot_state.print_status()


if __name__ == "__main__":
    main()
