from Components.interfaces import IRobotOperation
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
    get_x_values,
    get_y_values,
)


class RobotZigzagClassifier(IRobotOperation):
    def __init__(
        self,
    ):
        """
        Initialize the Zigzag Classifier.
        """
        super().__init__("Classifier")
        self.classification = {
            "Door1": "Empty Spot",
            "Door1": "Empty Spot",
            "Door1": "Empty Spot",
            "Door1": "Empty Spot",
        }

    def _classify_door(self):
        ylen_data = get_y_values(1, default_on_error=True)
        ylen = ylen_data["ylen"]

        print("ylen:", ylen)

        # Check conditions
        if ylen == "null":
            print("No door data available - skipping operations")
        elif isinstance(ylen, (int, float)):  # Ensure it's numeric
            self.classification["size"] = "Big" if ylen > 600 else "Small"
        else:
            print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

        return

    def invoke(self) -> None:
        """Run the operation."""
        self.classify_door()


class RobotZigzagUserSettingManager(IRobotOperation):
    """
    Manages user settings for zigzag operations.
    """

    def __init__(
        self,
    ):
        """ """
        super().__init__("UserSettingManager")

    def fetch_user_settings(self):
        # Placeholder for fetching user settings logic
        print("Fetching user settings for zigzag operations...")
        # Implement actual fetching logic here

    def invoke(self) -> None:
        """Run the operation."""
        self.fetch_user_settings()


class RobotZigzagPathPlanning(IRobotOperation):
    """
    Plans the zigzag path for the robot.
    """

    def __init__(
        self,
    ):
        """ """
        super().__init__("PathPlanning")

    def plan_zigzag_path(self):
        # Placeholder for zigzag path planning logic
        print("Planning zigzag path for the robot...")
        # Implement actual path planning logic here

    def invoke(self) -> None:
        """Run the operation."""
        self.plan_zigzag_path()
