from Components.RobotInteractions import RobotInteractions


class IRobotOperation(RobotInteractions):
    def __init__(
        self, name: str, *, config: dict | None = None, cps_client: object | None = None
    ):
        super().__init__(config=config, cps_client=cps_client)
        self.name = name
        print("Initialized IRobotOperation Instance with name:", self.name)

    def invoke(self) -> None:
        """Run the operation."""
        raise NotImplementedError("Subclasses must implement the invoke method.")
