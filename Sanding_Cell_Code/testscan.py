from Components.RobotState import RobotState


def main():
    robot_state = RobotState()
    while True:
        if not robot_state.fetch_and_update_pos():
            break
        robot_state.print_status()


if __name__ == "__main__":
    main()
