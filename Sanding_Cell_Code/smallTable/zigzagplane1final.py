import os
import sys
import yaml

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Components.RobotZigZagPipeline import RobotZigZagPipeline


def load_config():
    """Loads configuration from config.yaml."""
    with open("./configs/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    return config


def _run_zigzag_pipeline(door: int, *, force: float, z: float, cps):
    pipeline = RobotZigZagPipeline(
        name=f"ZigZagDoor{door}",
        config=load_config(),
        cps_client=cps,
        door=door,
        z=z,
        force=force,
    )
    pipeline.invoke()


def smalldoor1zizag(force, z, cps):
    _run_zigzag_pipeline(1, force=force, z=z, cps=cps)


def smalldoor2zizag(force, z, cps):
    _run_zigzag_pipeline(2, force=force, z=z, cps=cps)


def smalldoor3zizag(force, z, cps):
    _run_zigzag_pipeline(3, force=force, z=z, cps=cps)


def smalldoor4zizag(force, z, cps):
    _run_zigzag_pipeline(4, force=force, z=z, cps=cps)


if __name__ == "__main__":
    print("Use RobotZigZagPipeline to run zigzag operations.")
