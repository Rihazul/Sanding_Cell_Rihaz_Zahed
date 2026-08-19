from __future__ import annotations

import argparse
import threading
import time
from typing import Optional

from Components.RobotState import RobotState
from Components.RobotStateDashboard import RobotStateDashboard
from Components.RobotInteractions import load_config


def _updater_loop(robot_state: RobotState, interval: float, stop_event: threading.Event):
    """Background loop that keeps robot state fresh."""
    while not stop_event.is_set():
        if not robot_state.fetch_and_update_flags():
            stop_event.set()
            break
        if not robot_state.fetch_and_update_pos():
            stop_event.set()
            break
        if not robot_state.fetch_and_update_tcp():
            stop_event.set()
            break
        if not robot_state.fetch_and_update_ucs():
            stop_event.set()
            break
        
        stop_event.wait(interval)


def _render_loop(robot_state: RobotState, interval: float, stop_event: threading.Event):
    """Simple textual UI that prints the latest state."""
    while not stop_event.is_set():
        line = robot_state.format_status_line()
        try:
            print(line, end="\r", flush=True)
        except OSError:
            stop_event.set()
            break
        stop_event.wait(interval)


def _run_console_ui(robot_state: RobotState, update_interval: float, ui_interval: float) -> None:
    """Keep the legacy console loops running with configurable intervals."""
    stop_event = threading.Event()

    updater = threading.Thread(
        target=_updater_loop,
        args=(robot_state, update_interval, stop_event),
        daemon=True,
    )
    renderer = threading.Thread(
        target=_render_loop,
        args=(robot_state, ui_interval, stop_event),
        daemon=True,
    )

    updater.start()
    renderer.start()

    try:
        while updater.is_alive() and renderer.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    stop_event.set()
    updater.join(timeout=1.0)
    renderer.join(timeout=1.0)


def _run_textual_ui(
    robot_state: RobotState,
    refresh_interval: float,
) -> None:
    """Launch the textual dashboard when a fancy UI is requested."""
    RobotStateDashboard(
        robot_state=robot_state,
        refresh_interval=refresh_interval,
    ).run()


def main(argv: Optional[list[str]] = None):
    parser = argparse.ArgumentParser(description="Robot state monitor")
    parser.add_argument(
        "--update-interval",
        type=float,
        default=0.2,
        help="Seconds between state refreshes",
    )
    parser.add_argument(
        "--ui-interval",
        type=float,
        default=0.2,
        help="Seconds between UI refreshes (console mode)",
    )

    parser.add_argument(
        "--ui",
        choices=("textual", "console"),
        default="textual",
        help="Choose between the Textual dashboard or the legacy console spinner",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable console logger output (INFO level) in addition to file logs",
    )
    parser.add_argument(
        "--verbose-log-only",
        action="store_true",
        help="Enable verbose logging to file only (no console output)",
    )
    parser.add_argument(
        "--no-file-logs",
        action="store_true",
        help="Disable writing to app.log (only console if enabled)",
    )
    args = parser.parse_args(argv)

    config = load_config()
    settings = config.setdefault("settings", {})
    enable_console_logging = False
    if args.verbose_log_only:
        enable_console_logging = False
    elif args.verbose:
        enable_console_logging = True
    elif settings.get("debug", False):
        enable_console_logging = True
    settings["debug"] = enable_console_logging

    file_logging = not args.no_file_logs if hasattr(args, 'no_file_logs') else True
    settings["file_logging"] = file_logging

    robot_state = RobotState(config=config)
    if args.ui == "console":
        _run_console_ui(robot_state, args.update_interval, args.ui_interval)
    else:
        _run_textual_ui(
            robot_state,
            min(args.update_interval, args.ui_interval),
        )


if __name__ == "__main__":
    main()
