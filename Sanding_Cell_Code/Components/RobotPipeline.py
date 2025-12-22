"""Simple pipeline runner that sequences robot stages using a shared CPS client."""

from __future__ import annotations
from typing import TYPE_CHECKING, Protocol
from Interfaces import IOperation
from RobotInteractions import RobotInteractions
from modules.CPS import CPSClient


class RobotPipeline(IRobotOperation):
    def __init__(
        self,
        name: str,
        *,
        config: dict | None = None,
        cps_client: CPSClient | None = None,
    ):
        super().__init__(config=config, cps_client=cps_client)
        self.name = name
        self.stages: list[Stage] = []

    def add_stage(self, stage: Stage) -> None:
        """Append a stage that implements ``run()``."""
        self.stages.append(stage)

    def invoke(self) -> None:
        """Run the operation."""
        for stage in self.stages:
            stage.run()
