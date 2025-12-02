from __future__ import annotations

import asyncio
import os
from datetime import datetime
from typing import Any, Sequence

from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Grid
from textual.widgets import (
    DataTable,
    Footer,
    Header,
    Static,
    TabPane,
    TabbedContent,
)

from Components.RobotState import RobotState


class RobotStateDashboard(App):
    """Textual dashboard that renders the robot state snapshot in panels."""

    CSS = """
    Screen {
        background: #0f131a;
        color: $text;
        padding: 1;
    }
    #positions-grid,
    #frames-grid,
    #io-grid {
        grid-columns: 1fr 1fr;
        grid-gutter: 1;
        padding: 0 1;
    }
    .state-panel {
        border: round $accent;
        min-height: 8;
        padding: 1 2;
        background: $surface-darken-3;
    }
    #tcp-panel {
        height: 100%;
        overflow-y: auto;
    }
    """

    BINDINGS = [
        ("r", "refresh", "Refresh state"),
        ("c", "capture_point", "Capture point"),
        ("q", "quit", "Quit dashboard"),
        ("left", "previous_tab", "Previous tab"),
        ("right", "next_tab", "Next tab"),
        ("enter", "enter_section", "Enter section"),
        ("escape", "exit_section", "Exit section"),
    ]

    TAB_ORDER = ("positions", "frames", "io")

    def __init__(
        self,
        robot_state: RobotState | None = None,
        refresh_interval: float = 1.5,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self.robot_state = robot_state or RobotState()
        self.refresh_interval = refresh_interval
        self.capture_dir = "./logs"
        self.capture_file = os.path.join(self.capture_dir, "captured_points.log")
        self.captured_points: list[dict[str, Any]] = []
        os.makedirs(self.capture_dir, exist_ok=True)

        # For IO tables keyboard focus logic
        self._table_names = ("dio", "flags")
        self._active_table: str | None = None
        self._next_table_index = 0
        self._io_refresh_paused = False

    # -------------------------------------------------------------------------
    # Layout
    # -------------------------------------------------------------------------
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with TabbedContent(initial=self.TAB_ORDER[0], id="state-tabs"):
            # Positions tab
            with TabPane("Positions", id="positions"):
                with Grid(id="positions-grid"):
                    yield Static("", id="cartesian-panel", classes="state-panel")
                    yield Static("", id="joint-panel", classes="state-panel")

            # Frames tab (UCS panel + TCP DataTable)
            with TabPane("Frames", id="frames"):
                with Grid(id="frames-grid"):
                    with Static(id="tcp-panel", classes="state-panel"):
                        yield DataTable(id="tcp-table")
                    with Static(id="ucs-panel", classes="state-panel"):
                        yield DataTable(id="ucs-table")

            # IO tab (Digital IO + Flags)
            with TabPane("IO", id="io"):
                with Grid(id="io-grid"):
                    with Static("", id="dio-panel", classes="state-panel"):
                        yield DataTable(id="dio-table")
                    with Static("", id="flags-panel", classes="state-panel"):
                        yield DataTable(id="flags-table")
        yield Footer()

    # -------------------------------------------------------------------------
    # Lifecycle / refresh
    # -------------------------------------------------------------------------
    async def on_mount(self) -> None:
        self.set_interval(self.refresh_interval, self._refresh_panels)
        await self._refresh_panels()

    async def action_refresh(self) -> None:  # pragma: no cover - UI helper
        """Manual binding that forces a synchronous refresh."""
        await self._refresh_panels(force_io_refresh=True)

    async def action_capture_point(self) -> None:  # pragma: no cover - UI helper
        """Capture the current cartesian point along with active TCP and UCS."""
        await self._refresh_robot_state()
        self._capture_current_point()

    async def _refresh_panels(self, *, force_io_refresh: bool = False) -> None:
        await self._refresh_robot_state()
        state = self.robot_state.state or {}

        # Positions
        self.query_one("#cartesian-panel", Static).update(
            self._render_cartesian(state)
        )
        self.query_one("#joint-panel", Static).update(
            self._render_joints(state)
        )

        # Frames
        tcp_payload = state.get("tcp_frame", state.get("TCP"))
        self.query_one("#tcp-panel", Static).update(
            self._render_frame("TCP Frame", tcp_payload)
        )
        ucs_payload = state.get("ucs_frame", state.get("UCS"))
        self.query_one("#ucs-panel", Static).update(
            self._render_frame("UCS Frame", ucs_payload)
        )

        # IO
        should_update_io = force_io_refresh or not self._io_refresh_paused
        if should_update_io:
            self._update_dio_table(state.get("DigitalIOs", {}))
            self._update_flags_table(state.get("flags", {}))

        # Frame tables (always ok to refresh; it doesn't affect IO keyboard control)
        self._update_tcp_table()
        self._update_ucs_table()

    async def _refresh_robot_state(self) -> None:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._call_update_methods)

    def _call_update_methods(self) -> None:
        # Only read position + flags here; TCP definitions can be fetched
        # separately in your RobotState if you like.
        for method_name in (
            "fetch_and_update_pos",
            "fetch_and_update_flags",
            "fetch_and_update_tcp",
            "fetch_and_update_ucs",
        ):
            method = getattr(self.robot_state, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception as exc:  # pragma: no cover - best effort logging
                    self.log(f"Unable to call {method_name}: {exc}")

    # -------------------------------------------------------------------------
    # Tab navigation for IO tables
    # -------------------------------------------------------------------------
    def action_previous_tab(self) -> None:
        self._shift_tab(-1)

    def action_next_tab(self) -> None:
        self._shift_tab(1)

    def _shift_tab(self, direction: int) -> None:
        tabs = self.query_one("#state-tabs", TabbedContent)
        order = self.TAB_ORDER
        current = tabs.active or order[0]
        try:
            index = order.index(current)
        except ValueError:
            index = 0
        tabs.active = order[(index + direction) % len(order)]

    def action_enter_section(self) -> None:
        """Enter one of the IO tables (dio / flags) so arrow keys move the cursor."""
        if self._active_table is not None:
            return
        table_name = self._table_names[self._next_table_index]
        self._focus_table(table_name)
        self._active_table = table_name
        self._next_table_index = (self._next_table_index + 1) % len(
            self._table_names
        )
        self._io_refresh_paused = True

    def action_exit_section(self) -> None:
        """Return focus to tab header, resume IO refresh."""
        if self._active_table is None:
            return
        tabs = self.query_one("#state-tabs", TabbedContent)
        tabs.focus()
        self._active_table = None
        self._io_refresh_paused = False

    def _focus_table(self, table_name: str) -> None:
        table = self.query_one(f"#{table_name}-table", DataTable)
        table.cursor_type = "row"
        table.focus()

    def _update_dio_table(self, io_map: dict[str, bool]) -> None:
        table = self.query_one("#dio-table", DataTable)
        table.clear(columns=True)

        table.add_column("Name", width=10)
        table.add_column("Direction", width=12)
        table.add_column("State", width=8)

        if not io_map:
            table.add_row("<empty>", "", Text("-", style="dim"))
            return

        entries = sorted(io_map.items())

        for name, value in entries:
            direction = "Input" if name.startswith(("DI", "CI")) else "Output"
            table.add_row(
                name,
                Text(direction, justify="center"),
                self._bool_text(value),
            )

    def _update_flags_table(self, flags: dict[str, Any]) -> None:
        table = self.query_one("#flags-table", DataTable)
        table.clear(columns=True)

        table.add_column("Flag", width=20)
        table.add_column("Value", width=10)

        if not flags:
            table.add_row("No flags reported", "")
            return

        for name, value in sorted(flags.items()):
            table.add_row(
                name.replace("_", " ").title(),
                self._format_flag_value(value),
            )

    # -------------------------------------------------------------------------
    # TCP/UCS tables
    # -------------------------------------------------------------------------
    def _update_tcp_table(self) -> None:
        """Show TCP names alongside their corresponding coordinates."""

        table = self.query_one("#tcp-table", DataTable)
        table.clear(columns=True)

        table.add_column("TCP Name", width=30)
        table.add_column("TCP Coords")

        tcp_by_name = getattr(self.robot_state, "tcp_by_name", None) or {}
        if not tcp_by_name:
            legacy_map = getattr(self.robot_state, "coord_to_TCP", {}) or {}
            if legacy_map:
                tcp_by_name = {name: coords for coords, name in legacy_map.items()}

        if not tcp_by_name:
            table.add_row(Text("<no TCP data>", style="dim"), "")
            return

        active_tcp = None
        tcp_state = self.robot_state.state.get("TCP")
        if isinstance(tcp_state, dict):
            active_tcp = tcp_state.get("active")

        for name in sorted(tcp_by_name):
            coords = tcp_by_name[name]
            coord_text = self._coords_to_text(coords)
            if name == active_tcp:
                name_cell = Text(name, style="bold green")
                coord_cell = Text(coord_text, style="bold green")
            else:
                name_cell = Text(name, style="white")
                coord_cell = Text(coord_text)
            table.add_row(name_cell, coord_cell)

    def _update_ucs_table(self) -> None:
        """Show UCS names alongside their corresponding coordinates."""

        table = self.query_one("#ucs-table", DataTable)
        table.clear(columns=True)

        table.add_column("UCS Name", width=30)
        table.add_column("UCS Coords")

        ucs_by_name = getattr(self.robot_state, "ucs_by_name", None) or {}
        if not ucs_by_name:
            legacy_map = getattr(self.robot_state, "coord_to_UCS", {}) or {}
            if legacy_map:
                ucs_by_name = {name: coords for coords, name in legacy_map.items()}

        if not ucs_by_name:
            table.add_row(Text("<no UCS data>", style="dim"), "")
            return

        active_ucs = None
        ucs_state = self.robot_state.state.get("UCS")
        if isinstance(ucs_state, dict):
            active_ucs = ucs_state.get("active")

        for name in sorted(ucs_by_name):
            coords = ucs_by_name[name]
            coord_text = self._coords_to_text(coords)
            if name == active_ucs:
                name_cell = Text(name, style="bold green")
                coord_cell = Text(coord_text, style="bold green")
            else:
                name_cell = Text(name, style="white")
                coord_cell = Text(coord_text)
            table.add_row(name_cell, coord_cell)

    # -------------------------------------------------------------------------
    # Panels (Rich)
    # -------------------------------------------------------------------------
    def _render_cartesian(self, state: dict[str, Any]) -> Panel:
        coords = list(state.get("cartesian_position") or [])
        coords += [0.0] * max(0, 6 - len(coords))

        active_tcp = None
        tcp_state = state.get("TCP")
        if isinstance(tcp_state, dict):
            active_tcp = tcp_state.get("active")
        active_ucs = None
        ucs_state = state.get("UCS")
        if isinstance(ucs_state, dict):
            active_ucs = ucs_state.get("active")

        table = Table.grid(expand=True)
        table.add_column()
        table.add_column(justify="right")

        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for label, value in zip(labels, coords[:6]):
            table.add_row(label, f"{value:7.2f}")

        table.add_row("Rail", f"{state.get('robot_rail_position', 0.0):7.3f}")

        tcp_display = active_tcp if active_tcp not in (None, "", []) else "-"
        ucs_display = active_ucs if active_ucs not in (None, "", []) else "-"
        title = f"Cartesian Coordinates  --  TCP: {tcp_display}  |  UCS: {ucs_display}"

        return Panel(table, title=title, border_style="cyan")

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

    def _render_frame(
        self,
        title: str,
        payload: Sequence[float] | dict[str, Any] | None,
    ) -> Panel:
        """Generic 6-DOF frame renderer used for UCS (and can be reused elsewhere)."""
        table = Table.grid(expand=True)
        table.add_column()
        table.add_column(justify="right")

        if isinstance(payload, dict):
            active = payload.get("active")
            available = payload.get("available") or []
            active_display = str(active) if active not in (None, "", []) else "-"
            table.add_row("Active", active_display)
            table.add_row(
                "Available",
                ", ".join(str(item) for item in available) or "none",
            )
        else:
            numbers = list(payload or [])
            labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            for label, value in zip(labels, numbers[:6]):
                table.add_row(label, f"{value:7.2f}")
            if len(numbers) > 6:
                table.add_row(
                    "Extra",
                    " ".join(f"{value:7.2f}" for value in numbers[6:]),
                )

        return Panel(table, title=title, border_style="bright_blue")

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _capture_current_point(self) -> None:
        """Persist the latest point along with active TCP/UCS selections."""
        state = self.robot_state.state or {}

        coords = list(state.get("cartesian_position") or [])
        coords += [0.0] * max(0, 6 - len(coords))

        tcp_state = state.get("TCP") if isinstance(state, dict) else {}
        ucs_state = state.get("UCS") if isinstance(state, dict) else {}
        tcp_name = tcp_state.get("active") if isinstance(tcp_state, dict) else None
        ucs_name = ucs_state.get("active") if isinstance(ucs_state, dict) else None

        tcp_display = tcp_name or "-"
        ucs_display = ucs_name or "-"

        timestamp = datetime.now().isoformat(timespec="seconds")
        capture_line = (
            f"{timestamp} | TCP={tcp_display} | UCS={ucs_display} | "
            f"X={coords[0]:.3f}, Y={coords[1]:.3f}, Z={coords[2]:.3f}, "
            f"Rx={coords[3]:.3f}, Ry={coords[4]:.3f}, Rz={coords[5]:.3f}"
        )

        self.captured_points.append(
            {
                "timestamp": timestamp,
                "tcp": tcp_display,
                "ucs": ucs_display,
                "point": coords[:6],
            }
        )

        try:
            with open(self.capture_file, "a", encoding="utf-8") as handle:
                handle.write(capture_line + "\n")
        except Exception as exc:
            self.log(f"Capture failed: {exc}")
            return

        self.log(f"Captured point -> {capture_line}")

    def _format_flag_value(self, value: Any) -> Text:
        if isinstance(value, bool):
            return self._bool_text(value)
        return Text(str(value), style="bold")

    def _bool_text(self, value: bool) -> Text:
        status = bool(value)
        style = "bold green" if status else "dim"
        return Text("ON" if status else "OFF", style=style)

    def _coords_to_text(self, coords: Sequence[Any]) -> str:
        """Format coordinate list/tuple into a short comma-separated string."""
        converted: list[str] = []
        for coord in list(coords)[:6]:
            try:
                converted.append(f"{float(coord):.3f}")
            except Exception:
                converted.append("??")
        return ", ".join(converted) if converted else "-"
