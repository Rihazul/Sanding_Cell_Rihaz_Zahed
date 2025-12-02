from __future__ import annotations

import threading
import time
from queue import Empty, Queue
from typing import Any, Callable, Optional, Sequence

import yaml
from rich.progress import BarColumn, Progress, SpinnerColumn, TextColumn, TimeElapsedColumn, TimeRemainingColumn

from Server_Better_V2 import setup_logger
from modules.CPS import CPSClient

ProgressCallback = Callable[[float, Optional[str]], None]


def load_config(config_path: str = "./configs/config.yaml") -> dict:
    """Load and return YAML config as a dict."""
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


class RobotInteractions:
    """Minimal wrapper around CPS client setup and connection, plus helpers."""

    def __init__(self, config: dict | None = None, cps_client: CPSClient | None = None):
        self.config = config or load_config()
        settings = self.config.setdefault("settings", {})
        enable_console = settings.get("debug", False)
        enable_file_logging = settings.get("file_logging", True)
        log_file_name = settings.get("log_file_name", "app.log")
        logger_name = settings.get("logger_name", "DualOutputLogger")
        self.logger = setup_logger(enable_console, enable_file_logging, log_file_name, logger_name)
        self.config["logger"] = self.logger

        self.cps = cps_client or CPSClient()
        self.initialize_comms()

    def initialize_comms(self) -> bool:
        """Establish connection to the robot controller."""
        self.logger.info("Initializing communications...")
        ip = self.config["server"]["cpip"]
        port = self.config["server"]["cps"]
        ret = self.cps.HRIF_Connect(0, ip, port)

        if ret != 0:
            self.logger.error(f"Error {ret}, failed to connect to robot.")
            return False

        self.logger.info("Successfully connected to robot.")
        return True

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------
    def move_l(
        self,
        *,
        point: Sequence[float],
        raw_acs: Sequence[float] | None = None,
        tcp: str | None = None,
        ucs: str | None = None,
        speed: float | None = None,
        acc: float | None = None,
        radius: float | None = None,
        seek: bool = False,
        seek_bit: int = 0,
        seek_state: int = 0,
        cmd_id: str = "0",
        wait: bool = True,
        wait_timeout: float | None = 60.0,
        box_id: int = 0,
        robot_id: int = 0,
        progress_callback: ProgressCallback | None = None,
        show_progress: bool = False,
        progress_label: str | None = "MoveL",
    ) -> bool:
        """Execute a linear move with optional seeking and wait-for-completion."""

        coords = self.config.get("coords", {})
        tcp_name = tcp or coords.get("tcpDefault")
        ucs_name = ucs or coords.get("ucsDefault")
        speed_val = speed if speed is not None else coords.get("roboVelocity", 200.0)
        acc_val = acc if acc is not None else coords.get("roboAcceleration", 200.0)
        radius_val = radius if radius is not None else coords.get("transitionRadius", 0.0)
        raw_acs_val = list(raw_acs) if raw_acs is not None else [0.0] * 6
        point_val = list(point)

        if tcp_name is None:
            self.logger.error("TCP name is required (provide --tcp or set coords.tcpDefault in config).")
            return False
        if ucs_name is None:
            self.logger.error("UCS name is required (provide --ucs or set coords.ucsDefault in config).")
            return False
        if len(raw_acs_val) != 6:
            self.logger.error("Raw joint pose must contain 6 values.")
            return False
        if len(point_val) != 6:
            self.logger.error("Target cartesian pose must contain 6 values.")
            return False

        self.logger.info("Setting frames before MoveL...")
        if progress_callback:
            self._update_progress(progress_callback, 0.0, "Configuring frames")
        if not self._configure_frames(box_id, robot_id, ucs_name, tcp_name):
            return False

        ret = self.cps.HRIF_MoveL(
            box_id,
            robot_id,
            point_val,
            raw_acs_val,
            tcp_name,
            ucs_name,
            speed_val,
            acc_val,
            radius_val,
            1 if seek else 0,
            seek_bit,
            seek_state,
            cmd_id,
        )
        if ret != 0:
            self.logger.error(f"HRIF_MoveL failed with code {ret}.")
            self._describe_error(box_id, ret)
            self._log_robot_state(box_id)
            return False

        if wait:
            timeout_val = wait_timeout if wait_timeout is not None and wait_timeout > 0 else None
            if not self._wait_for_motion_done(
                box_id,
                robot_id,
                timeout_val,
                progress_callback=progress_callback,
                show_progress=show_progress,
                progress_label=progress_label or "Move",
            ):
                return False

        self._update_progress(progress_callback, 100.0, "MoveL completed")
        self.logger.info("MoveL completed successfully.")
        return True

    def grp_reset(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Reset the robot group."""
        return self._run_group_command("HRIF_GrpReset", box_id, robot_id)

    def grp_enable(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Enable the robot group."""
        return self._run_group_command("HRIF_GrpEnable", box_id, robot_id)

    def grp_disable(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Disable the robot group."""
        return self._run_group_command("HRIF_GrpDisable", box_id, robot_id)

    def grp_stop(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Stop robot motion immediately."""
        return self._run_group_command("HRIF_GrpStop", box_id, robot_id)

    def grp_interrupt(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Pause robot motion."""
        return self._run_group_command("HRIF_GrpInterrupt", box_id, robot_id)

    def grp_continue(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Resume robot motion after an interrupt."""
        return self._run_group_command("HRIF_GrpContinue", box_id, robot_id)

    def grp_close_free_drive(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Close/disable free drive mode."""
        return self._run_group_command("HRIF_GrpCloseFreeDriver", box_id, robot_id)

    def grp_open_free_drive(self, box_id: int = 0, robot_id: int = 0) -> bool:
        """Open/enable free drive mode."""
        return self._run_group_command("HRIF_GrpOpenFreeDriver", box_id, robot_id)

    def grprst(self) -> bool:
        """Backward-compatible alias for group reset."""
        return self.grp_reset(0, 0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _run_group_command(self, method_name: str, box_id: int, robot_id: int) -> bool:
        fn = getattr(self.cps, method_name, None)
        if not callable(fn):
            self.logger.error(f"CPS client missing method {method_name}")
            return False
        ret = fn(box_id, robot_id)
        if ret != 0:
            self.logger.error(f"{method_name} failed with code {ret}.")
            self._describe_error(box_id, ret)
            self._log_robot_state(box_id)
            return False
        self.logger.info(f"{method_name} succeeded for box {box_id}, robot {robot_id}.")
        return True

    def _configure_frames(self, box_id: int, robot_id: int, ucs_name: str, tcp_name: str) -> bool:
        current_ucs: list[str] = []
        target_ucs: list[str] = []
        ret = self.cps.HRIF_ReadCurUCS(box_id, robot_id, current_ucs)
        if ret != 0:
            self.logger.error(f"Failed to read current UCS (code {ret}).")
            return False
        ret = self.cps.HRIF_ReadUCSByName(box_id, robot_id, ucs_name, target_ucs)
        if ret != 0:
            self.logger.error(f"Failed to read UCS '{ucs_name}' (code {ret}).")
            return False
        if current_ucs != target_ucs:
            ret = self.cps.HRIF_SetUCSByName(box_id, robot_id, ucs_name)
            if ret != 0:
                self.logger.error(f"Could not set UCS to '{ucs_name}' (code {ret}).")
                return False
            time.sleep(0.1)

        current_tcp: list[str] = []
        target_tcp: list[str] = []
        ret = self.cps.HRIF_ReadCurTCP(box_id, robot_id, current_tcp)
        if ret != 0:
            self.logger.error(f"Failed to read current TCP (code {ret}).")
            return False
        ret = self.cps.HRIF_ReadTCPByName(box_id, robot_id, tcp_name, target_tcp)
        if ret != 0:
            self.logger.error(f"Failed to read TCP '{tcp_name}' (code {ret}).")
            return False
        if current_tcp != target_tcp:
            ret = self.cps.HRIF_SetTCPByName(box_id, robot_id, tcp_name)
            if ret != 0:
                self.logger.error(f"Could not set TCP to '{tcp_name}' (code {ret}).")
                return False
            time.sleep(0.1)

        return True

    def _update_progress(
        self,
        progress_callback: ProgressCallback | None,
        value: float,
        message: str | None,
        progress: Progress | None = None,
        task_id: int | None = None,
        label: str | None = None,
    ) -> None:
        if progress_callback:
            try:
                progress_callback(value, message)
            except Exception:
                self.logger.debug("Progress callback raised an exception", exc_info=True)
        if progress is not None and task_id is not None:
            progress.update(task_id, completed=value, description=message or label or "")

    def _wait_for_motion_done(
        self,
        box_id: int,
        robot_id: int,
        timeout: float | None,
        *,
        progress_callback: ProgressCallback | None = None,
        show_progress: bool = False,
        progress_label: str | None = "Motion",
    ) -> bool:
        start = time.monotonic()
        last_progress = 0.0

        progress: Progress | None = None
        task_id: int | None = None
        if show_progress:
            progress = Progress(
                SpinnerColumn(),
                BarColumn(),
                TextColumn("{task.completed:>5.1f}%"),
                TimeElapsedColumn(),
                TimeRemainingColumn(),
                TextColumn(progress_label or "Motion"),
                transient=True,
            )
            progress.__enter__()
            task_id = progress.add_task(progress_label or "Motion", total=100.0)

        try:
            while True:
                state: list[str] = []
                ret = self.cps.HRIF_ReadRobotState(box_id, robot_id, state)
                if ret != 0:
                    self.logger.error(f"Failed to read robot state (code {ret}).")
                    self._update_progress(progress_callback, last_progress, "State read failed", progress, task_id, progress_label)
                    return False
                if len(state) > 11 and state[11] == "1":
                    self._update_progress(progress_callback, 100.0, "Motion complete", progress, task_id, progress_label)
                    return True
                elapsed = time.monotonic() - start
                if timeout is not None and timeout > 0 and elapsed > timeout:
                    self.logger.error("Timed out waiting for motion completion.")
                    self._update_progress(progress_callback, last_progress, "Timed out", progress, task_id, progress_label)
                    return False
                if timeout is not None and timeout > 0:
                    guess = min(95.0, (elapsed / timeout) * 100.0)
                else:
                    guess = min(95.0, last_progress + 1.0)
                if guess > last_progress:
                    last_progress = guess
                    self._update_progress(progress_callback, guess, "In progress", progress, task_id, progress_label)
                time.sleep(0.1)
        finally:
            if progress is not None:
                progress.__exit__(None, None, None)

    def _describe_error(self, box_id: int, code: int) -> None:
        """Fetch and log a human-readable error string when possible."""
        result: list[str] = []
        ret = self.cps.HRIF_GetErrorCodeStr(box_id, code, result)
        if ret == 0 and result:
            self.logger.error(f"Controller description for error {code}: {result[0]}")
        else:
            self.logger.error(f"Unable to fetch error description for {code} (ret={ret}, raw={result})")

    def _log_robot_state(self, box_id: int) -> None:
        """Log current robot state flags for quick diagnosis."""
        result: list[str] = []
        ret = self.cps.HRIF_ReadRobotState(box_id, 0, result)
        if ret != 0:
            self.logger.error(f"Failed to read robot state (code {ret}).")
            return
        if len(result) < 13:
            self.logger.error(f"Unexpected robot state payload: {result}")
            return
        flags = {
            "in_motion": result[0] == "1",
            "enabled": result[1] == "1",
            "has_error": result[2] == "1",
            "error_code": result[3],
            "error_axis": result[4],
            "lock_state": result[5] == "1",
            "paused": result[6] == "1",
            "emergency_stop": result[7] == "1",
            "light_screen": result[8] == "1",
            "powered_on": result[9] == "1",
            "ebox_connected": result[10] == "1",
            "point_motion_done": result[11] == "1",
            "in_position": result[12] == "1",
        }
        self.logger.error(f"Robot state flags: {flags}")


class RobotInteractionsAsync:
    """Async runner that executes robot commands on a worker thread."""

    def __init__(self, robot: RobotInteractions) -> None:
        self.robot = robot
        self._queue: Queue[tuple[str, Callable[..., bool]]] = Queue()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="robot-async-worker",
            daemon=False,
        )
        self._worker.start()

    def submit_move_l(self, **kwargs: Any) -> None:
        def runner() -> bool:
            return self.robot.move_l(progress_callback=None, show_progress=False, **kwargs)

        self._queue.put(("MoveL", runner))

    def submit_callable(self, name: str, func: Callable[[], bool]) -> None:
        self._queue.put((name, func))

    def wait_for_all(self, timeout: float | None = None) -> None:
        self._queue.join()
        if timeout:
            self._worker.join(timeout=timeout)

    def stop(self) -> None:
        self._stop_event.set()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                name, func = self._queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                func()
            except Exception as exc:  # pragma: no cover - defensive logging
                self.robot.logger.exception(f"Async task {name} failed: {exc}")
            finally:
                self._queue.task_done()
