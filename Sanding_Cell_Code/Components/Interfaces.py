from __future__ import annotations

from typing import Optional

from Components.RobotInteractions import RobotInteractions


class IRobotOperation(RobotInteractions):
    def __init__(
        self,
        name: str,
        *,
        config: Optional[dict] = None,
        cps_client: object = None,
    ):
        super().__init__(config=config)
        self.name = name
        print("Initialized IRobotOperation Instance with name:", self.name)

    def invoke(self) -> None:
        """Run the operation."""
        raise NotImplementedError("Subclasses must implement the invoke method.")
