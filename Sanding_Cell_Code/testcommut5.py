# Side Sanding with 1st tool
import time
import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
# Define points
    spoint=[307.855,291.182,-50,180,0,0]
    p5_out = [1293.2244999999998, 167.2754999999999, -1.5, 180, 0, 0]
    p6_out = [1293.2244999999998, 886.8245000009999, -1.5, 180, 0, 0]
    p7_out = [910.2255, 886.8245000009999, -1.5, 180, 0, 0]
    p8_out = [910.2255, 167.2754999999999, -1.5, 180, 0, 0]
    p9_out = [851.8995, 167.2754999999999, -1.5, 180, 0, 0]
    p10_out = [851.8995, 886.8245000009999, -1.5, 180, 0, 0]
    p11_out = [468.9004999999999, 886.8245000009999, -1.5, 180, 0, 0]
    p12_out = [468.9004999999999, 167.2754999999999, -1.5, 180, 0, 0]
    p13_out = [410.5745, 167.27549999959996, -1.5, 180, 0, 0]
    p14_out = [410.5745, 886.8245000009999, -1.5, 180, 0, 0]
    p15_out = [27.575499999999863, 886.8245000009999, -1.5, 180, 0, 0]
    p16_out = [27.575499999999863, 167.27549999959996, -1.5, 180, 0, 0]

    # Conveyer Position
    x = p16_out[0]
    x1 = p12_out[0]
    x2 = p7_out[0]


    # Robot position
    p16 = [0, p16_out[1], p16_out[2], p16_out[3], p16_out[4], p16_out[5]]
    p15 = [0, p15_out[1], p15_out[2], p15_out[3], p15_out[4], p15_out[5]]

    disr = p13_out[0] - p15_out[0]
    dishalf = disr / 2

    p13 = [disr, p13_out[1], p13_out[2], p13_out[3], p13_out[4], p13_out[5]]
    p14 = [disr, p14_out[1], p14_out[2], p14_out[3], p14_out[4], p14_out[5]]

    p16mid = [dishalf, p16_out[1], p16_out[2], p16_out[3], p16_out[4], p16_out[5]]
    p15mid = [dishalf, p15_out[1], p15_out[2], p15_out[3], p15_out[4], p15_out[5]]
    p15midair = [dishalf, p15_out[1], -30, p15_out[3], p15_out[4], p15_out[5]]
    p16midair = [dishalf, p16_out[1], -30, p16_out[3], p16_out[4], p16_out[5]]
    points = [p16mid, p16, p15, p15mid, p15midair]
    points1= [p15, p15mid, p16mid, p16, p16midair]

    #converyer half
    xhalf=dishalf
    xh=x+xhalf
    x1h=x1+xhalf
    x2h=x2+xhalf

    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)

    # Initial robot movement (without conveyor)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_int,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point1upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=smpoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x_2nd,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point2upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    
    communicate(cps=cps,config=config,seventh=x,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.7,wait=True)
    turn_vibration_on(cps)
    for point in points:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=xh,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points1:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=x1h,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points1:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=x2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,seventh=x2h,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points1:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpReal'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.7,wait=True)
    turn_vibration_off(cps)
    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)
if __name__ == "__main__":
    main()