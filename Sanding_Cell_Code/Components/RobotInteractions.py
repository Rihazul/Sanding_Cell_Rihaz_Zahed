from __future__ import annotations

import yaml
from Server_Better_V2 import setup_logger
from modules.CPS import CPSClient


def load_config(config_path: str = "./configs/config.yaml") -> dict:
    """Load and return YAML config as a dict."""
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


class RobotInteractions:
    """Minimal wrapper around CPS client setup and connection."""

    def __init__(self, config: dict | None = None, cps_client: CPSClient | None = None):
        self.config = config or load_config()
        settings = self.config.setdefault("settings", {})
        enable_console = settings.get("debug", False)
        enable_file_logging = settings.get("file_logging", True)
        self.logger = setup_logger(enable_console, enable_file_logging)
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
