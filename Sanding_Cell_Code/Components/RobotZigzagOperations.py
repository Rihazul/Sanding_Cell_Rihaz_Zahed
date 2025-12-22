"""
This file contains each operation for the ZigZag pipeline as a separate class.

Each class inherits from IRobotOperation and implements the invoke method.

The classes included are:
    - RobotZigzagStateManager: Holds the data structures required for zigzag.
    - RobotZigzagUserSettingManager: Manages user settings for zigzag operations.
    - RobotZigzagPathPlanning: Plans the zigzag path for the robot.
    - RobotZigzagPathExecution: Executes the zigzag path for the robot.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

from Components.Interfaces import IRobotOperation
from Components.RobotState import RobotState
from Server_Better_V2 import (
    communicate,
    waitForBlending,
    turn_vibration_on,
    turn_vibration_off,
    putForceZminus,
    releaseForce,
)
from smallTable.scancord import (
    get_door_position,
    get_inner_corner_point,
    get_y_values,
)


class RobotZigzagStateManager(IRobotOperation):
    """
    Manages the state of the zigzag operations.

    THIS CLASS HOLDS ALL THE IMPORTANT DATA STRUCTURES REQUIRED TO PROCESS THE ZIGZAG.
    """

    def __init__(
        self,
        door: int,
        z: float,
        *,
        config: dict | None = None,
        cps_client: object | None = None,
    ):
        super().__init__("ZigzagStateManager", config=config, cps_client=cps_client)
        self.door = door
        self.z = z
        self.prehoming = [0, 200, 50, 0, 0, 0]

        self.ylen_data: dict | None = None
        self.ylen: object | None = None
        self.has_data = False

        self.p8: list | None = None
        self.p7: list | None = None
        self.p6: list | None = None
        self.p5: list | None = None
        self.x1: float | None = None
        self.distance: float | None = None

        self.point5: list | None = None
        self.point6: list | None = None
        self.point7: list | None = None
        self.point8: list | None = None

        self.point5u: list | None = None
        self.point6u: list | None = None
        self.point7u: list | None = None
        self.point8u: list | None = None

        self.x_coords1: list = []
        self.y_coords1: list = []
        self.z_coords1: list = []

        self._load_scan_meta()
        if self.has_data:
            self._set_corner_points()

    def _load_scan_meta(self) -> None:
        self.ylen_data = get_y_values(self.door, default_on_error=True)
        self.ylen = self.ylen_data.get("ylen") if self.ylen_data else None
        self.has_data = isinstance(self.ylen, (int, float))

    def has_scan_data(self) -> bool:
        return self.has_data

    def _set_corner_points(self) -> None:
        # Top Left
        self.p8 = get_inner_corner_point(self.door, 0)
        print("p8:", self.p8)
        # Bottom Left
        self.p7 = get_inner_corner_point(self.door, 1)
        print("p7:", self.p7)
        # Bottom Right
        self.p6 = get_inner_corner_point(self.door, 2)
        print("p6:", self.p6)
        # Up Right
        self.p5 = get_inner_corner_point(self.door, 3)
        print("p5:", self.p5)

        # 7th axis position
        self.x1 = self.p8[0] + get_door_position(self.door)
        print("x1:", self.x1)

        self.distance = self.p6[0] - self.p8[0]
        print("distance:", self.distance)

        # Points calculation
        self.point5 = [-self.p5[0], self.p5[1], self.z, -0.034, 0.556, 0.251]
        print("point5:", self.point5)
        self.point6 = [-self.p6[0], self.p6[1], self.z, -0.034, 0.556, 0.251]
        print("point6:", self.point6)
        self.point7 = [-self.p7[0], self.p7[1], self.z, -0.034, 0.556, 0.251]
        print("point7:", self.point7)
        self.point8 = [-self.p8[0], self.p8[1], self.z, -0.034, 0.556, 0.251]
        print("point8:", self.point8)

        # Final Points
        self.point5u = [
            -self.distance,
            self.point5[1],
            self.point5[2],
            self.point5[3],
            self.point5[4],
            self.point5[5],
        ]
        print("point5u:", self.point5u)
        self.point6u = [
            -self.distance,
            self.point6[1],
            self.point6[2],
            self.point6[3],
            self.point6[4],
            self.point6[5],
        ]
        print("point6u:", self.point6u)
        self.point7u = [
            0,
            self.point7[1],
            self.point7[2],
            self.point7[3],
            self.point7[4],
            self.point7[5],
        ]
        print("point7u:", self.point7u)
        self.point8u = [
            0,
            self.point8[1],
            self.point8[2],
            self.point8[3],
            self.point8[4],
            self.point8[5],
        ]
        print("point8u:", self.point8u)

        # 1) Collect boundary coordinates as [x, y, z]
        self.x_coords1 = [
            self.point5u[0],
            self.point6u[0],
            self.point7u[0],
            self.point8u[0],
        ]
        self.y_coords1 = [
            self.point5u[1],
            self.point6u[1],
            self.point7u[1],
            self.point8u[1],
        ]
        self.z_coords1 = [
            self.point5u[2],
            self.point6u[2],
            self.point7u[2],
            self.point8u[2],
        ]

        print("x_coords1 ", self.x_coords1)
        print("y_coords1 ", self.y_coords1)
        print("z_coords1", self.z_coords1)

    def invoke(self) -> None:
        """Run the operation."""
        if not self.has_data:
            print("No door data available - skipping state setup")
            return
        if self.point5 is None:
            self._set_corner_points()


class RobotZigzagUserSettingManager(IRobotOperation):
    """
    Manages user settings for zigzag operations.

    Call this as an updater in the flask_app to fetch user settings.
    """

    def __init__(
        self,
        state_manager: RobotZigzagStateManager,
        *,
        config: dict | None = None,
        cps_client: object | None = None,
    ):
        super().__init__(
            "ZigzagUserSettingManager", config=config, cps_client=cps_client
        )
        self.state_manager = state_manager

        self.regular_setting = {
            "overlap": 0.2,
            "speed": 200,
        }
        self.spiral_setting = {
            "radius": 5,
            "max_points": 5000,
            "turns": 10,
        }

        # Straight Movement or Spiral
        self.spiral: bool = True

        self.inner_offset = 17
        self.inner_offset_x = 17
        self.inner_sanding_offset = (
            30 if self.state_manager.door == 4 else 50
        )

        self.zigzag_path: list[list[float]] = []
        self.prepoint: Optional[list[float]] = None

    def fetch_user_settings(self) -> None:
        """
        Fetch user settings for zigzag operations.
        All the important user settings are fetched here.
        """
        zigzag_cfg = self.config.get("zigzag", {}) if isinstance(self.config, dict) else {}
        self.spiral = zigzag_cfg.get("spiral", self.spiral)
        self.inner_offset = zigzag_cfg.get("innerOffset", self.inner_offset)
        self.inner_offset_x = zigzag_cfg.get("innerOffsetX", self.inner_offset_x)
        self.inner_sanding_offset = zigzag_cfg.get(
            "innerSandingOffset", self.inner_sanding_offset
        )

    def generate_zigzag_path(
        self,
        x_coords: list,
        y_coords: list,
        z_coords: list,
        *,
        inner_offset: float,
        inner_offset_x: float,
        inner_sanding_offset: float,
    ):
        prepoint = None
        zigzag_coords = []

        # Parameters (adjust as needed)
        tool3y = 50.8  # Tool offset in Y
        tool3x = 38.1  # Tool offset in X
        xframe_1 = 0
        xframe_2 = 0

        # 1) Collect boundary coordinates as [x, y, z]
        boundary_coords = []
        for i in range(len(x_coords)):
            boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

        # Close the loop by duplicating the first point at the end
        if boundary_coords:
            boundary_coords.append(boundary_coords[0][:])  # copy for safety

        # 2) Compute the zigzag path (offset corners + zigzag)
        zigzag_coords = []

        # Ensure we have valid coordinates
        if x_coords and y_coords and z_coords:
            # We'll assume the pocket's Z-level is the same as the first boundary point
            z_zigzag = boundary_coords[0][2]

            # For Pocket4, corners (P13, P14, P15, P16):
            modified_Point2 = [
                (x_coords[1]) / 1 + tool3x + inner_offset_x,
                y_coords[1] - tool3y - (inner_offset * 0.5),
            ]
            modified_Point3 = [
                x_coords[2] - tool3x - inner_offset,
                y_coords[2] - tool3y - inner_offset,
            ]
            modified_Point1 = [
                (x_coords[0]) / 1 + tool3x + inner_offset_x,
                y_coords[0] + tool3y + inner_offset,
            ]
            modified_Point4 = [
                x_coords[3] - tool3x - inner_offset,
                y_coords[3] + tool3y + inner_offset,
            ]
            print("modified_Point1:", modified_Point1)
            print("modified_Point2:", modified_Point2)
            print("modified_Point3:", modified_Point3)
            print("modified_Point4:", modified_Point4)

            # Calculate available horizontal dimension
            xlen1 = abs(modified_Point3[0] - modified_Point1[0])
            xinner = xlen1 - xframe_1 - xframe_2
            print("xinner=", xinner)

            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / inner_sanding_offset)
                adjusted_step = xinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag path from left to right
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    row_points = [
                        [
                            modified_Point1[0] + offset,
                            modified_Point1[1],
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],
                        [
                            modified_Point2[0] + offset,
                            modified_Point2[1],
                            z_zigzag,
                            0,
                            0,
                            0,
                        ],
                    ]
                    # Reverse every other row to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle

            # Update only the y coordinate to its absolute value
            for point in zigzag_coords:
                point[1] = abs(point[1])
                point[0] = abs(point[0])

            prepoint = [
                abs(modified_Point1[0]) + 0.5,
                modified_Point1[1],
                z_zigzag,
                0,
                0,
                0,
            ]
        return zigzag_coords, prepoint

    def invoke(self) -> None:
        """Run the operation."""
        self.fetch_user_settings()
        if not self.state_manager.has_scan_data():
            print("No door data available - skipping zigzag settings")
            return
        self.zigzag_path, self.prepoint = self.generate_zigzag_path(
            x_coords=self.state_manager.x_coords1,
            y_coords=self.state_manager.y_coords1,
            z_coords=self.state_manager.z_coords1,
            inner_offset=self.inner_offset,
            inner_offset_x=self.inner_offset_x,
            inner_sanding_offset=self.inner_sanding_offset,
        )
        print("zigzag_pathp=", self.zigzag_path)
        print("prepointp:", self.prepoint)


class RobotZigzagPathPlanning(IRobotOperation):
    """
    Plans the zigzag path for the robot.
    """

    def __init__(
        self,
        *,
        config: dict | None = None,
        cps_client: object | None = None,
    ):
        super().__init__("PathPlanning", config=config, cps_client=cps_client)
        self.prempted_planned_path_names = {
            "ZigZagNormalDoor1": "not_ready",
            "ZigZagNormalDoor2": "not_ready",
            "ZigZagNormalDoor3": "not_ready",
            "ZigZagNormalDoor4": "not_ready",
            "ZigZagSpiralDoor1": "not_ready",
            "ZigZagSpiralDoor2": "not_ready",
            "ZigZagSpiralDoor3": "not_ready",
            "ZigZagSpiralDoor4": "not_ready",
            "SquareZigZagDoor1": "not_ready",
            "SquareZigZagDoor2": "not_ready",
            "SquareZigZagDoor3": "not_ready",
            "SquareZigZagDoor4": "not_ready",
        }
        self._status_lock = threading.Lock()

    def _set_path_status(self, track_name: str, status: str) -> None:
        with self._status_lock:
            self.prempted_planned_path_names[track_name] = status

    def _zigzag_config(self) -> dict:
        if isinstance(self.config, dict):
            return self.config.get("zigzag", {}) or {}
        return {}

    def _get_planned_path_names(self) -> list[str]:
        zigzag_cfg = self._zigzag_config()
        planned = zigzag_cfg.get("planned_path_names")
        if planned is None and isinstance(self.config, dict):
            planned = self.config.get("planned_path_names")
        if planned is None:
            planned = list(self.prempted_planned_path_names.keys())
        if isinstance(planned, str):
            planned = [planned]
        if not isinstance(planned, (list, tuple)):
            return list(self.prempted_planned_path_names.keys())
        names = [name for name in planned if isinstance(name, str)]
        return names or list(self.prempted_planned_path_names.keys())

    def _parse_door_number(self, track_name: str) -> int | None:
        if "Door" not in track_name:
            return None
        suffix = track_name.rsplit("Door", 1)[-1]
        if not suffix.isdigit():
            return None
        return int(suffix)

    def _resolve_z_for_door(self, door: int) -> float:
        zigzag_cfg = self._zigzag_config()
        z_value = None
        z_by_door = zigzag_cfg.get("z_by_door")
        if isinstance(z_by_door, dict):
            z_value = z_by_door.get(door, z_by_door.get(str(door)))
        if z_value is None:
            z_value = zigzag_cfg.get("z")
        if z_value is None:
            try:
                scan_point = get_inner_corner_point(door, 0, default_on_error=True)
            except TypeError:
                scan_point = get_inner_corner_point(door, 0)
            if (
                isinstance(scan_point, (list, tuple))
                and len(scan_point) > 2
                and isinstance(scan_point[2], (int, float))
            ):
                z_value = scan_point[2]
        if z_value is None:
            z_value = 0.0
        return float(z_value)

    @staticmethod
    def _flatten_points(points: list[list[float]]) -> list[float]:
        return [coord for point in points for coord in point]

    def _build_square_path_points(
        self, state_manager: "RobotZigzagStateManager"
    ) -> list[list[float]]:
        if not (
            state_manager.x_coords1
            and state_manager.y_coords1
            and state_manager.z_coords1
        ):
            return []
        points: list[list[float]] = []
        for x_val, y_val, z_val in zip(
            state_manager.x_coords1, state_manager.y_coords1, state_manager.z_coords1
        ):
            points.append([abs(x_val), abs(y_val), z_val, 0, 0, 0])
        if points:
            points.append(points[0][:])
        return points

    def _build_spiral_path_points(self, points: list[list[float]]) -> list[float]:
        if len(points) < 2:
            return []
        zigzag_cfg = self._zigzag_config()
        turns = int(zigzag_cfg.get("spiral_turns", 10))
        radius = float(zigzag_cfg.get("spiral_radius", 12.0))
        angle_step_deg = float(zigzag_cfg.get("spiral_angle_step_deg", 45.0))
        flat_points: list[float] = []
        for index in range(len(points) - 1):
            segment = RobotZigzagPathExecution.generate_spiral_between_points(
                start_pose=points[index],
                end_pose=points[index + 1],
                turns=turns,
                radius=radius,
                angle_step_deg=angle_step_deg,
            )
            if index > 0:
                segment = segment[6:]
            flat_points.extend(segment)
        return flat_points

    def _build_points_for_track(self, track_name: str) -> list[float] | None:
        door = self._parse_door_number(track_name)
        if door is None:
            return None

        z_value = self._resolve_z_for_door(door)
        state_manager = RobotZigzagStateManager(
            door, z_value, config=self.config, cps_client=self.cps
        )
        state_manager.invoke()
        if not state_manager.has_scan_data():
            return None

        settings_manager = RobotZigzagUserSettingManager(
            state_manager, config=self.config, cps_client=self.cps
        )
        settings_manager.invoke()

        if track_name.startswith("ZigZagSpiralDoor"):
            return self._build_spiral_path_points(settings_manager.zigzag_path)
        if track_name.startswith("ZigZagNormalDoor"):
            return self._flatten_points(settings_manager.zigzag_path)
        if track_name.startswith("SquareZigZagDoor"):
            square_points = self._build_square_path_points(state_manager)
            return self._flatten_points(square_points)
        return None

    def _preempt_path(self, track_name: str) -> None:
        coords = self.config.get("coords", {}) if isinstance(self.config, dict) else {}
        velocity = coords.get("roboVelocity", 300.0)
        accel = coords.get("roboAcceleration", 500.0)
        jerk = coords.get("roboJerk", 10000.0)
        ucs_name = coords.get("ucsTable1") or coords.get("ucsDefault")
        tcp_name = coords.get("tcptool1plane1") or coords.get("tcpDefault")

        ret = self.cps.HRIF_InitMovePathL(
            0,
            0,
            track_name,
            velocity,
            accel,
            jerk,
            ucs_name,
            tcp_name,
        )
        if ret != 0:
            self._set_path_status(track_name, "error")
            if hasattr(self, "logger"):
                self.logger.error(
                    f"[PathPlanning] InitMovePathL failed for {track_name}: {ret}"
                )
            return

        flat_points = self._build_points_for_track(track_name)
        if not flat_points:
            self._set_path_status(track_name, "error")
            if hasattr(self, "logger"):
                self.logger.error(
                    f"[PathPlanning] No points generated for {track_name}."
                )
            return
        if len(flat_points) % 6 != 0:
            self._set_path_status(track_name, "error")
            if hasattr(self, "logger"):
                self.logger.error(
                    f"[PathPlanning] Point list misaligned for {track_name}."
                )
            return
        push_ret = self.cps.HRIF_PushPathPoints(0, 0, track_name, flat_points)
        if push_ret != 0:
            self._set_path_status(track_name, "error")
            if hasattr(self, "logger"):
                self.logger.error(
                    f"[PathPlanning] PushPathPoints failed for {track_name}: {push_ret}"
                )
            return

        end_ret = self.cps.HRIF_EndPushPathPoints(0, 0, track_name)
        if end_ret != 0:
            self._set_path_status(track_name, "error")
            if hasattr(self, "logger"):
                self.logger.error(
                    f"[PathPlanning] EndPushPathPoints failed for {track_name}: {end_ret}"
                )
            return

        self._set_path_status(track_name, "ready")

    def plan_zigzag_path(self) -> None:
        # Placeholder for zigzag path planning logic
        print("Planning zigzag path for the robot...")
        # Implement actual path planning logic here
        # This should ideally be done right after the zigzag planning phase
        self.pre_empt_all_paths()

    def pre_empt_all_paths(self) -> None:
        """Preempt all existing paths."""
        planned_paths = list(dict.fromkeys(self._get_planned_path_names()))
        if not planned_paths:
            print("No planned paths provided - skipping path preemption.")
            return
        print("Prempted all the paths in planned_path_names below...")
        # Call everything from INit_MovePathL to EndPushPathL here in separate threads so it loads faster.
        # As soon as Possible
        threads = []
        for track_name in planned_paths:
            if track_name not in self.prempted_planned_path_names:
                self.prempted_planned_path_names[track_name] = "not_ready"
            thread = threading.Thread(
                target=self._preempt_path, args=(track_name,), daemon=True
            )
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()

    def invoke(self) -> None:
        """Run the operation."""
        self.plan_zigzag_path()


class RobotZigzagPathExecution(IRobotOperation):
    """
    Executes the zigzag path for the robot.
    """

    def __init__(
        self,
        state_manager: RobotZigzagStateManager,
        settings_manager: RobotZigzagUserSettingManager,
        *,
        force: float,
        planner: RobotZigzagPathPlanning | None = None,
        config: dict | None = None,
        cps_client: object | None = None,
    ):
        super().__init__("PathExecution", config=config, cps_client=cps_client)
        self.state_manager = state_manager
        self.settings_manager = settings_manager
        self.force = force
        self.planner = planner

    @staticmethod
    def compute_timeout(
        *,
        total_points: Optional[int] = None,
        cap: float = 72.0,
        velocity: float = 300.0,
    ) -> float:
        """
        Estimate a completion timeout based on total points.

        - Uses a simple model: time = (points / vmax) + overhead
        - Caps the timeout to avoid excessive waits.
        """
        if total_points is None:
            return cap

        distance_per_point = 4.64
        time_factor = float(total_points * distance_per_point / (velocity))

        timeout = float(time_factor)
        print(
            f"[Timeout] total_points={total_points}, est={time_factor:.1f}s timeout={timeout:.1f}s"
        )
        return timeout

    @staticmethod
    def generate_spiral_between_points(
        start_pose,
        end_pose,
        turns: int = 22,
        radius: float = 12.0,
        angle_step_deg: float = 45.0,
    ):
        """
        Build a spiral path between two cartesian poses (X, Y, Z, Rx, Ry, Rz).
        Keeps orientation from start_pose, interpolates center XY between poses,
        and adds radial offsets for each step.
        """
        x0, y0, z0, rx, ry, rz = start_pose[:6]
        x1, y1, _, _, _, _ = end_pose[:6]
        if y0 == y1:
            turns = 3

        total_steps = int(turns * (360.0 / angle_step_deg))
        points = list(start_pose[:6])  # start point

        for step in range(1, total_steps + 1):
            t = step / total_steps
            cx = x0 + (x1 - x0) * t
            cy = y0 + (y1 - y0) * t
            theta_deg = step * angle_step_deg
            theta = math.radians(theta_deg)

            px = cx + radius * math.cos(theta)
            py = cy + radius * math.sin(theta)
            pz = z0  # hold Z while spiraling along Y

            points.extend([px, py, pz, rx, ry, rz])

        return points

    def run_spiral_between_points(
        self,
        start_pose,
        end_pose,
        *,
        turns: int = 30,
        radius: float = 12.0,
        angle_step_deg: float = 10.0,
        tcp: str | None = None,
        ucs: str | None = None,
        track_name: str | None = None,
        velocity: float = 300.0,
        accel: float = 500.0,
        jerk: float = 10000.0,
        box_id: int = 0,
        robot_id: int = 0,
        init_path: bool = True,
    ):
        """Push a spiral MovePathL between two poses."""
        base_name = track_name or "spiral_path"
        track_name = track_name or base_name
        print(f"[Spiral] Using track name: {track_name}")
        tcp_name = tcp or self.config["coords"].get("tcptool1plane1")
        ucs_name = ucs or self.config["coords"].get("ucsTable1")

        all_points = self.generate_spiral_between_points(
            start_pose=start_pose,
            end_pose=end_pose,
            turns=turns,
            radius=radius,
            angle_step_deg=angle_step_deg,
        )

        if len(all_points) % 6 != 0:
            print("[Spiral] Point list misaligned (not divisible by 6).")
            return False, None

        count = len(all_points) // 6
        print(f"[Spiral] Total points = {count}")

        if init_path:
            ret = self.cps.HRIF_InitMovePathL(
                box_id,
                robot_id,
                track_name,
                velocity,
                accel,
                jerk,
                ucs_name,
                tcp_name,
            )
            print("[Spiral][Init] ret =", ret)
            if ret != 0:
                return False, None

        ret = self.cps.HRIF_PushMovePaths(
            box_id,
            robot_id,
            track_name,
            1,
            count,
            all_points,
        )
        print("[Spiral][Push] ret =", ret)
        if ret != 0:
            return False, None

        return True, count

    def finalize_spiral_path(
        self,
        track_name: str,
        *,
        box_id: int = 0,
        robot_id: int = 0,
        completion_timeout: float = 76.0,
    ) -> bool:
        """End push, wait for readiness, execute MovePathL, and wait for completion."""
        ret = self.cps.HRIF_EndPushPathPoints(box_id, robot_id, track_name)
        print("[Spiral][EndPush] ret =", ret)
        if ret != 0:
            return False

        # Create a lightweight state helper for motion monitoring.
        robot_state = RobotState(config=self.config, cps_client=self.cps)

        # Wait for PATH_READY
        start = time.time()
        while True:
            st = []
            ret = self.cps.HRIF_ReadPathState(box_id, robot_id, track_name, st)
            if ret == 0 and len(st) > 3 and st[2] == "3" and st[3] == "0":
                break
            elapsed = time.time() - start
            if elapsed > 120.0:
                print(
                    f"[Spiral] Timeout waiting for PATH_READY after {elapsed:.1f}s:",
                    st,
                )
                return False
            time.sleep(0.1)
            print(f"Waiting for PATH_READY... t={elapsed:.1f}s state={st}\r", end="")

        print("[Spiral][Move] Starting MovePathL...")
        ret = self.cps.HRIF_MovePathL(box_id, robot_id, track_name)
        print("[Spiral][Move] ret =", ret)
        if ret != 0:
            return False

        # Wait for completion (track path state + robot flags)
        start = time.time()
        while True:
            pstate = []
            pret = self.cps.HRIF_ReadPathState(box_id, robot_id, track_name, pstate)
            if pret != 0:
                print("HRIF_ReadPathState failed:", pret)
                return False
            if len(pstate) > 3 and pstate[3] != "0":
                print(f"[Spiral] Path reported error: {pstate}")
                return False

            # Break early if the robot stays still for 0.5s.
            if robot_state.wait_until_still(still_seconds=0.2, poll_seconds=0.05):
                print("[Spiral] No motion detected for 0.5s; exiting wait loop.")
                break

        return True

    def _resolve_track_name(self) -> str:
        if self.settings_manager.spiral:
            return f"ZigZagSpiralDoor{self.state_manager.door}"
        return f"ZigZagNormalDoor{self.state_manager.door}"

    def perform_process_top(self, points1):
        # Force Control Activated
        putForceZminus(
            cps=self.cps,
            force=self.force,
            tcp=self.config["coords"]["tcptool1plane1"],
            ucs=self.config["coords"]["ucsTable1"],
            config=self.config,
        )
        turn_vibration_on(self.cps)
        print("Turned Vibration On")

        spiral_track_name = self._resolve_track_name()
        path_initialized = False
        push_failed = False
        total_count = 0

        for index, _ in enumerate(points1):
            point_A = points1[index]
            if index + 1 >= len(points1):
                break
            point_B = points1[index + 1]

            print("Spiral move from A to B:", point_A, "->", point_B)
            success, count = self.run_spiral_between_points(
                start_pose=point_A,
                end_pose=point_B,
                turns=10,
                radius=12.0,
                angle_step_deg=45.0,
                track_name=spiral_track_name,
                velocity=300.0,
                accel=500.0,
                jerk=10000.0,
                init_path=not path_initialized,
            )
            if not success:
                push_failed = True
                break

            path_initialized = True
            total_count += count

        if path_initialized and not push_failed:
            timeout = self.compute_timeout(
                total_points=total_count, velocity=300.0 * 10.0 / 45.0
            )
            self.finalize_spiral_path(
                spiral_track_name,
                box_id=0,
                robot_id=0,
                completion_timeout=timeout,
            )

        if self.state_manager.door in (3, 4):
            waitForBlending(cps=self.cps, config=self.config)
        turn_vibration_off(self.cps)
        releaseForce(cps=self.cps, config=self.config)

    def execute_zigzag_path(self) -> None:
        # Placeholder for zigzag path execution logic
        print("Executing zigzag path for the robot...")
        # Implement actual path execution logic here
        # Only call the MovePathL here.
        # Whenever the command is set.

        if not self.state_manager.has_scan_data():
            print("No door data available - skipping zigzag execution")
            return

        if not self.settings_manager.zigzag_path or not self.settings_manager.prepoint:
            print("No zigzag path/prepoint available - skipping zigzag execution")
            return

        # Original sequence with dynamic variables
        communicate(
            cps=self.cps,
            config=self.config,
            seventh=self.state_manager.x1,
            tcp=self.config["coords"]["tcptool1plane1"],
            ucs=self.config["coords"]["ucsTable1"],
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=self.cps,
            config=self.config,
            point=self.state_manager.prehoming,
            tcp=self.config["coords"]["tcptool1plane1"],
            ucs=self.config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        communicate(
            cps=self.cps,
            config=self.config,
            point=self.settings_manager.prepoint,
            tcp=self.config["coords"]["tcptool1plane1"],
            ucs=self.config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )
        self.perform_process_top(points1=self.settings_manager.zigzag_path)
        communicate(
            cps=self.cps,
            config=self.config,
            point=self.state_manager.prehoming,
            tcp=self.config["coords"]["tcptool1plane1"],
            ucs=self.config["coords"]["ucsTable1"],
            seventh=-1,
            speed=0.2,
            wait=True,
        )

    def invoke(self) -> None:
        """Run the operation."""
        self.execute_zigzag_path()
