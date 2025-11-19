# InnerSideCycle
import time
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,turn_vibration_on,turn_vibration_off,waitForBlending  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Define points and conveyor positions
    offset=5
    spoint=[150,291.182,-100.483,180,0,0]
    p13=[344.74,245.709,7.5,180,0,0]
    p14=[344.74,808.39,7.5,180,0,0]
    p15=[93.409,808.39,7.5,180,0,0]
    p16=[93.409,245.709,7.5,180,0,0]
    x16=p16[0]
    x14=p14[0]
    dis164= x14 - x16
    p16u= [0+offset, p16[1]+offset,p16[2],p16[3],p16[4],p16[5]]
    p15u= [0+offset, p15[1]-offset,p15[2],p15[3],p15[4],p15[5]]
    p13u= [dis164-offset, p13[1]+offset,p13[2],p13[3],p13[4],p13[5]]
    p14u= [dis164-offset, p14[1]-offset,p14[2],p14[3],p14[4],p14[5]]
    #For the second pocket
    p9=[786.0649999999999, 245.70999999999992, 4, 180, 0, 0]
    p10=[786.0649999999999, 808.390000001, 4, 180, 0, 0]
    p11=[534.7349999999999, 808.390000001, 4, 180, 0, 0]
    p12=[534.7349999999999, 245.70999999999992, 4, 180, 0, 0]
    x12=p12[0]
    #For the third pocket
    p5=[1227.3899999999999, 245.70999999999992, 4, 180, 0, 0]
    p6=[1227.3899999999999, 808.390000001, 4, 180, 0, 0]
    p7=[976.06, 808.390000001, 4, 180, 0, 0]
    p8=[976.06, 245.70999999999992, 4, 180, 0, 0]
    x8=p8[0]
    points = [p16u, p15u, p14u, p13u, p16u,spoint]
    print('x12',x12)
    print(p13)
    print(p14)
    print(p15)
    print(p16)
    print(x16)
    print(x14)
    print(dis164)
    print(p16u)
    print(p15u)
    print(p13u)
    print(p14u)

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
    #communicate(cps=cps,config=config,seventh=x_2nd,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point12middleupy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point2upy,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    ##New movement
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=x1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=point1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    communicate(cps=cps,config=config,seventh=x16,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
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
    
    communicate(cps=cps,config=config,seventh=x12,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
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

    communicate(cps=cps,config=config,seventh=x8,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
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
    turn_vibration_off(cps)
    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=config['door']['homingSpeed'], wait=True)
    
    
if __name__ == "__main__":
    main()