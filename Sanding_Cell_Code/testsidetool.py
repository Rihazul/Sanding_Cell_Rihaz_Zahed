# testcommu.py

import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading

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

    p1= [1320.8, 0, 0, 180, 0, 0]
    p2= [1320.8, 917.575, 0, 180, 0, 0]
    p3= [0.0, 917.575, 0, 180, 0, 0]
    p4= [0.0, 0, 0, 180, 0, 0]

    #point with offset
    ofsx=17
    p4of=[p4[0]+ofsx,p4[1],p4[2],p4[3],p4[4],p4[5]]
    p3of=[p3[0]+ofsx,p3[1],p3[2],p3[3],p3[4],p3[5]]


    #conveyer
    cx1=p4[0]
    cxbuff=-20
    cx1n=cx1+cxbuff

    # points for bottom
    balanceoffset=-1.5 
    step=3
    offbottom= -17
    zbottom=45
    length=(p1[0]-p4[0])/step
    p4bottom=[p4[0],p4[1]-balanceoffset,zbottom,0,0,-90]
    p1bottom=[length,p1[1]-balanceoffset,zbottom,0,0,-90]
    pbottomair=[p4[0],offbottom,p4[2],0,0,-90]

    #points for the left
    lightoffsetleft=0
    offleft=17
    p4left=[p4[0]+lightoffsetleft,p4[1],zbottom,0,0,0]
    p3left=[p3[0]+lightoffsetleft,p3[1],zbottom,0,0,0]
    pleftair=[offleft,p4[1],p4[2],0,0,0]

    #Converyerbottom
    conbx=p4[0]
    convbl=(p1[0]-p4[0])/step
    conbx2=convbl
    conbx3=convbl*2
    conbx4=convbl*3

    #points for the Top
    lightoffsetop=0 
    steptop=6
    offbottom= -10
    zbottom=45
    length=(p1[0]-p4[0])/steptop
    p2air=[length/2,p2[1]-offbottom +20,p4[2]-10,0,0,90]
    p3air3=[0,p2[1]-offbottom,p4[2]-10,0,0,90]

    p2top=[length,p2[1]+lightoffsetop,zbottom,0,0,90]
    p3top=[p3[0],p3[1]+lightoffsetop,zbottom,0,0,90]
    p2topminus=[-length,p2[1]+lightoffsetop,zbottom,0,0,90]

    #Conveyertop
    convxtop=p4[0]
    convxtoplen=(p1[0]-p4[0])/steptop
    convx2top=convxtoplen
    convx3top=convxtoplen*3
    convx5top=convxtoplen*5

    #points for the right
    zbottomr=45
    lightoffsetright=1.5
    bufferright=10
    offbottomright= 10
    pleright3=[p3[0]-lightoffsetright,p3[1],zbottomr,0,0,-180]
    pleright4=[p4[0]-lightoffsetright,p4[1],zbottomr,0,0,-180]
    plrightair=[p3[0]-lightoffsetright-bufferright,p3[1]+offbottomright,p3[2],0,0,-180]
    safehome=[p4[0]-lightoffsetright,p4[1],-100,0,0,-180]
    


    points= [pbottomair,p4bottom,p1bottom]
    points1= [p4bottom,p1bottom]
    pointsleft=[pleftair,p4left,p3left]
    pointstop=[p2top,p2topminus]
    pointright=[pleright3,pleright4]

    print("p4of:", p4of)
    print("p3of:", p3of)
    print("cx1n:", cx1n)

        # Define wrapper functions for the two commands you want to run concurrently.
    def run_robot_movement():
        communicate(cps=cps, config=config, point=pbottomair, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement():
        communicate(cps=cps, config=config, seventh=conbx2, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    #Second Thread
    def run_robot_movement1():
        communicate(cps=cps, config=config, point=pbottomair, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement1():
        communicate(cps=cps, config=config, seventh=conbx3, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    
    #Third Thread
    def run_robot_movement2():
        communicate(cps=cps, config=config, point=pbottomair, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement2():
        communicate(cps=cps, config=config, seventh=conbx4, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    
    #Fourth Thread
    def run_robot_movement3():
        communicate(cps=cps, config=config, point=p2air, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement3():
        communicate(cps=cps, config=config, seventh=convx3top, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    #Fifth Thread
    def run_robot_movement4():
        communicate(cps=cps, config=config, point=p2air, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], seventh=-1, speed=0.8, wait=True)

    def run_axis_movement4():
        communicate(cps=cps, config=config, seventh=convx2top, tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'], speed=0.5, wait=True)
    
    
    robot_thread = threading.Thread(target=run_robot_movement)
    axis_thread = threading.Thread(target=run_axis_movement)

    #Second Thread

    robot_thread1 = threading.Thread(target=run_robot_movement1)
    axis_thread1 = threading.Thread(target=run_axis_movement1)

    #Third Thread

    robot_thread2 = threading.Thread(target=run_robot_movement2)
    axis_thread2 = threading.Thread(target=run_axis_movement2)
    
    #Fourth Thread
    robot_thread3 = threading.Thread(target=run_robot_movement3)
    axis_thread3 = threading.Thread(target=run_axis_movement3)

    #Fifth Thread
    robot_thread4 = threading.Thread(target=run_robot_movement4)
    axis_thread4 = threading.Thread(target=run_axis_movement4)



    # Call the communicate function with parameters from config
    #communicate(cps=cps,config=config,seventh=cx1n,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p4,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #moveOnlyJ6r(cps, -90, config, wait=True)
    #communicate(cps=cps,config=config,point=p4of,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p3of,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    moveOnlyJ6r(cps, -180, config, wait=True)
    communicate(cps=cps,config=config,seventh=conbx,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    turn_vibration_on(cps)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p4bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=p1bottom,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,point=pbottomair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    for point in points:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    axis_thread.start()
    robot_thread.start()
    # Wait for both threads to complete
    robot_thread.join()
    axis_thread.join()
    #communicate(cps=cps,config=config,seventh=conbx2,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points1:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    axis_thread1.start()
    robot_thread1.start()
    # Wait for both threads to complete
    robot_thread1.join()
    axis_thread1.join()
    #communicate(cps=cps,config=config,seventh=conbx3,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in points1:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    axis_thread2.start()
    robot_thread2.start()
    # Wait for both threads to complete
    robot_thread2.join()
    axis_thread2.join()
    #communicate(cps=cps,config=config,seventh=conbx4,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    #Left Side
    for point in pointsleft:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.5,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    #Top
    #axis_thread3.start()
    #robot_thread3.start()
    # Wait for both threads to complete
    #robot_thread3.join()
    #axis_thread3.join()
    communicate(cps=cps,config=config,point=p3air3,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=convx5top,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    for point in pointstop:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    #Combination movement ontop
    axis_thread3.start()
    robot_thread3.start()
    # Wait for both threads to complete
    robot_thread3.join()
    axis_thread3.join()
    #communicate(cps=cps,config=config,point=p2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=convx3top,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in pointstop:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    #communicate(cps=cps,config=config,point=p2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    #communicate(cps=cps,config=config,seventh=convx2top,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    axis_thread4.start()
    robot_thread4.start()
    # Wait for both threads to complete
    robot_thread4.join()
    axis_thread4.join()
    for point in pointstop:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    #Top right
    
    communicate(cps=cps,config=config,point=p2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=convxtop,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.5,wait=True)
    communicate(cps=cps,config=config,point=plrightair,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    
    for point in pointright:
        communicate(
        cps=cps,
        config=config,
        point=point,
        tcp=config['coords']['tcpSideTool'],
        ucs=config['coords']['ucsTable2'],
        seventh=-1,
        speed=0.3,
        wait=False
    )
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    communicate(cps=cps,config=config,seventh=cxbuff,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    communicate(cps=cps,config=config,point=safehome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    


if __name__ == "__main__":
    main()