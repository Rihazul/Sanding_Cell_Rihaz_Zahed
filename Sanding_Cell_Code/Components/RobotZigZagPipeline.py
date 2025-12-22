from __future__ import annotations

from typing import Optional

from Components.RobotPipeline import RobotPipeline


class RobotZigZagPipeline(RobotPipeline):
    def __init__(
        self,
        name: str,
        *,
        config: Optional[dict] = None,
        cps_client: "CPSClient" = None,
    ):
        """
        Initialize the ZigZag pipeline with a name, optional config, and CPS client.
        """

        super().__init__(name, config=config, cps_client=cps_client)
