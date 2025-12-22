"""

This file contains Each operation for the ZigZag pipeline as a separate class.

Each class inherits from IRobotOperation and implements the invoke method.

The classes included are:
    - RobotZigzagClassifier: Classifies doors based on scan data.
    - RobotZigzagUserSettingManager: Manages user settings for zigzag operations.
    - RobotZigzagPathPlanning: Plans the zigzag path for the robot.
    - RobotZigzagPathExecution: Executes the zigzag path for the robot.

"""

from Interfaces import IRobotOperation
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


class RobotZigzagStateManager(IRobotOperation):
    """
    Manages the state of the zigzag operations.

    THIS CLASS HOLDS ALL THE IMPORATN DATA STRUCTURES REQUIED TO PROCESS THE ZIGZAG

    """

    def __init__(
        self,
    ):
        """
        Initialize the Zigzag Classifier.
        """
        super().__init__("Classifier")

        self.classification = {
            "Door1": {"Empty Spot", "Empty Type"},
            "Door2": {"Empty Spot", "Empty Type"},
            "Door3": {"Empty Spot", "Empty Type"},
            "Door4": {"Empty Spot", "Empty Type"},
        }

    def _init_from_scan(self):
        """Initialize state from scan data."""
        return

    def _set_corner_points(self):
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6 = get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        # 7th axis postion
        x1 = p8[0] + get_door_position(1)
        print("x1:", x1)
        # prehoming
        prehoming = [0, 200, 50, 0, 0, 0]

        distance = p6[0] - p8[0]
        print("distance:", distance)
        # Points calculation
        point5 = [-p5[0], p5[1], z, -0.034, 0.556, 0.251]
        print("point5:", point5)
        point6 = [-p6[0], p6[1], z, -0.034, 0.556, 0.251]
        print("point6:", point6)
        point7 = [-p7[0], p7[1], z, -0.034, 0.556, 0.251]
        print("point7:", point7)
        point8 = [-p8[0], p8[1], z, -0.034, 0.556, 0.251]
        print("point8:", point8)

        # Final Points
        point5u = [-distance, point5[1], point5[2], point5[3], point5[4], point5[5]]
        print("point5u:", point5u)
        point6u = [-distance, point6[1], point6[2], point6[3], point6[4], point6[5]]
        print("point6u:", point6u)
        point7u = [0, point7[1], point7[2], point7[3], point7[4], point7[5]]
        print("point7u:", point7u)
        point8u = [0, point8[1], point8[2], point8[3], point8[4], point8[5]]
        print("point8u:", point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
        y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
        z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

        print("x_coords1 ", x_coords1)
        print("y_coords2 ", y_coords1)
        print("z_coords3", z_coords1)

    def invoke(self) -> None:
        """Run the operation."""


class RobotZigzagUserSettingManager(IRobotOperation):
    """
    Manages user settings for zigzag operations.

    Call this as an updater in the flask_apppy to fetch user settings.

    """

    def __init__(
        self,
        RobotZigzagStateManager: RobotZigzagStateManager,
    ):
        """ """
        self.RobotZigzagStateManager = RobotZigzagStateManager
        super().__init__("UserSettingManager")

        self.regular_setting = {
            overlap: 0.2,
            speed: 200,
        }

        self.spiral_setting = {
            radius: 5,
            max_points: 5000,
            turns: 10,
        }

        # Straight Movement or Spiral
        self.spiral: bool = True

    def fetch_user_settings(self):
        """
        Fetch user settings for zigzag operations.
        All the importatnt user settings are fetched here:
        - Config
        """

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


class RobotZigzagPathExecution(IRobotOperation):
    """
    Executes the zigzag path for the robot.
    """

    def __init__(
        self,
    ):
        """ """
        super().__init__("PathExecution")

    def execute_zigzag_path(self):
        # Placeholder for zigzag path execution logic
        print("Executing zigzag path for the robot...")
        # Implement actual path execution logic here

    def invoke(self) -> None:
        """Run the operation."""
        self.execute_zigzag_path()
