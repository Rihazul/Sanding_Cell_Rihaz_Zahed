from RobotPipeline import RobotPipeline


class RobotZigZagPipeline(RobotPipeline):
    def __init__(
        self,
        name: str,
        *,
        config: dict | None = None,
        cps_client: "CPSClient" | None = None,
    ):
        """
        Initialize the ZigZag pipeline with a name, optional config, and CPS client.
        """

        super().__init__(name, config=config, cps_client=cps_client)
