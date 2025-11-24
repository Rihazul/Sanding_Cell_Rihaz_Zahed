from modules.CPS import CPSClient

def compute(config_path):
    """A textual description of the compute function.

    Inputs:
        in1 (all): Textual description of in1
        in2 (all): Textual description of in2

    Outputs:
        out1 (all): Textual description of out1
        out2 (all): Textual description of out2

    Requirements:
    """
    config = None
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    # Load configuration with console logging enabled
    
    config['logger'] = setup_logger(config['settings']['debug'])
    cps = CPSClient()
    
    print("Initializing communications...")
        # Connect to robot
    IP = self.config['server']['cpip']
    port = self.config['server']['cps']
    ret = self.cps.HRIF_Connect(0, IP, port)

    if ret != 0:
        res = f"Error {ret}, Failed to connect to robot."
    else:
        res ="Successfully connected to robot."
    return {"status": res }


def test():
    """Test the compute function."""

    print("Running test")
