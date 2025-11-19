# testcommu.py

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    # Establish connection with robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    #Points
    #p13_out = [410.5745, 167.27549999959996, -1.5, 180, 0, 0]
    #p14_out = [410.5745, 886.8245000009999, -1.5, 180, 0, 0]
    shome=[150,291.182,-150.483,180,0,0]
    # spoint=[27.575499999999863, 0, 0, 180, 0, 0]
    # p15_out = [27.575499999999863, 381, -1, 180, 0, 0]
    # p16_out = [27.575499999999863, 1, -1, 180, 0, 0]
    spoint1pre=[27.9, 28, -10, 180, 0, 0]
    spoint1=[27.9, 28, -1, 180, 0, 0]
    spoint=[30, 28, -1, 180, 0, 0]
    p14_middile = [100, 28, -1, 180, 0, 0]
    p15_middile = [350, 28, -1, 180, 0, 0]
    p16_out = [550, 28, -1, 180, 0, 0]
    p16_outpre = [550, 28, -10, 180, 0, 0]
    p16s=[95.507,250.170,9.0,180,0,0]
    p15s=[95.507,815.018,9.0,180,0,0]
    point=[spoint1pre,spoint1,spoint,p14_middile,p15_middile,p16_out,p16_outpre]
    x=0

    # Call the communicate function with parameters from config
    
    communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.6,wait=True)
    communicate(cps=cps,config=config,point=shome,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    communicate(cps=cps,config=config,point=spoint1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    putForceZplus(
            cps=cps,
            force=30,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
    print("force Executed Rafat",putForceZplus)
    turn_vibration_on(cps)
    print("Vibration Executed",turn_vibration_on)
    communicate(cps=cps,config=config,seventh=200,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.6,wait=True)
    print("Seventh Movement to 50 Executed Rafat",communicate)
    releaseForce(cps=cps, config=config)
    turn_vibration_off(cps)
    print("Release force Executed Rafat",releaseForce)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    # putForceZplus(cps=cps, force=10, tcp=config['coords']['tcpReal'], ucs=config['coords']['ucsTable2'], config=config)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=p16_out,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    # #communicate(cps=cps,config=config,point=p15_out,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    # releaseForce(cps=cps, config=config)
    # communicate(cps=cps,config=config,point=shome,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.6,wait=True)
    # turn_vibration_off(cps)
    # def perform_process_middle(cps, config, points1):
    #     # Vibration on
        
        
    #     # Force Control Activated
        # putForceZplus(
        #     cps=cps,
        #     force=10,
        #     tcp=config['coords']['tcpReal'],
        #     ucs=config['coords']['ucsTable2'],
        #     config=config
        # )
        
        
    #     # Communicate to each point in points1
    #     for point in points1:
    #         if point==spoint1:turn_vibration_on(cps)
        
    #         if point==p14_middile:turn_vibration_off(cps)
    #         if point==p15_middile:turn_vibration_on(cps)
    #         # cps=cps,
    #         # force=10,
    #         # tcp=config['coords']['tcpReal'],
    #         # ucs=config['coords']['ucsTable2'],
    #         # config=config
    #         # )
    #         communicate(
    #             cps=cps,
    #             config=config,
    #             point=point,
    #             tcp=config['coords']['tcpReal'],
    #             ucs=config['coords']['ucsTable2'],
    #             seventh=-1,
    #             speed=0.6,
    #             wait=False
    #         )
        
    #     # Wait for blending and turn off vibration
    #     waitForBlending(cps=cps, config=config)
    #     turn_vibration_off(cps)
        
    #     # Release Force Control
    #     releaseForce(cps=cps, config=config)
    # perform_process_middle(cps, config, points1=point)


if __name__ == "__main__":
    main()