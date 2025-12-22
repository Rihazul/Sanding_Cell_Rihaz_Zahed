from Components.RobotPipeline import RobotPipeline
from Components.RobotZigzagOperations import (
    RobotZigzagStateManager,
    RobotZigzagUserSettingManager,
    RobotZigzagPathPlanning,
    RobotZigzagPathExecution,
)


class RobotZigZagPipeline(RobotPipeline):
    def __init__(
        self,
        name: str,
        *,
        config: dict | None = None,
        cps_client: "CPSClient" | None = None,
        door: int | None = None,
        z: float | None = None,
        force: float | None = None,
        plan_paths: bool = True,
    ):
        """
        Initialize the ZigZag pipeline with a name, optional config, and CPS client.
        """

        super().__init__(name, config=config, cps_client=cps_client)
        self.door = door
        self.z = z
        self.force = force
        self.plan_paths = plan_paths

    def invoke(self) -> None:
        """Run the zigzag pipeline end-to-end."""
        if self.door is None or self.z is None or self.force is None:
            raise ValueError("door, z, and force must be provided to run the pipeline.")

        state_manager = RobotZigzagStateManager(
            self.door, self.z, config=self.config, cps_client=self.cps
        )
        state_manager.invoke()
        if not state_manager.has_scan_data():
            print("No door data available - skipping zigzag pipeline")
            return

        settings_manager = RobotZigzagUserSettingManager(
            state_manager, config=self.config, cps_client=self.cps
        )
        settings_manager.invoke()

        planner = None
        if self.plan_paths:
            planner = RobotZigzagPathPlanning(config=self.config, cps_client=self.cps)
            planner.invoke()

        executor = RobotZigzagPathExecution(
            state_manager,
            settings_manager,
            force=self.force,
            planner=planner,
            config=self.config,
            cps_client=self.cps,
        )
        executor.invoke()
