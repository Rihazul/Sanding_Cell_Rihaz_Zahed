# testing force control
import time
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config


def main():
    
