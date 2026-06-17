# testcommu.py

import sys
import os
import json
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceYplus1,putForceXminus,putForceYminus1,putForceXplus,moveOnlyJ6r
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from Table2Model.exportpointsmodule import exported_points
from model2cycle.testmodel2tool2sideb import testmodel2tool2sidesbigfunction


def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config





def testmodel2tool2sidesrun(force,cps,section=None,return_home=True,pass_index=None):

    def testmodel2tool2sidesmallfunction(force,cps,section=None,return_home=True,pass_index=None):
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Main Points
        p1 = exported_points["p1"]
        p2 = exported_points["p2"]
        p3 = exported_points["p3"]
        p4 = exported_points["p4"]
        p5 = exported_points["p5"]
        p6 = exported_points["p6"]
        p7 = exported_points["p7"]
        p8 = exported_points["p8"]

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        sanding_speed = float(json_config.get('sandingSpeed', speeed))
        if sanding_speed <= 0:
            sanding_speed = speeed
        print(speeed)
    

        print("p1=",p1)
        print("p2=",p2)
        print("p3=",p3)
        print("p4=",p4)
        print("p5=",p5)
        print("p6=",p6)
        print("p7=",p7)
        print("p8=",p8)
    

        #Frame of Outside
        frameOutside=p1[0]-p3[0]
        print("frameOutside=",frameOutside)

        #Bottom Points
        pointhome=[p4[0],p4[1],-150,0,0,-90]
        print("pointhome=",pointhome)
        point1pre=[p4[0],p4[1]-5,p4[2]+45,0,0,-90]
        print("point1pre=",point1pre)
        point1=[p4[0],p4[1]+0-1.5,p4[2]+45,0,0,-90]
        print("point1=",point1)
        point12=[p4[0]+1+1,p4[1]+0-1.5,p4[2]+45,0,0,-90]
        print("point12=",point12)
        point2=[p1[0]/2,p1[1]+0-1.5,p1[2]+45,0,0,-90]
        print("point2=",point2)
        point2pre=[p1[0]/2,p1[1]-5-10,p1[2]+45,0,0,-90]
        print("point2pre=",point2pre)
        point2air=[p1[0]/2,p1[1]-20,p1[2]+45,0,0,-90]
        print("point2air=",point2air)
        point1Combo=[0,p1[1]-20,p1[2]+45,0,0,-90]
        print("point1Combo=",point1Combo)

        #Bottom Extra point
        point2bottomextra=[p1[0]/2+30,p1[1]-20,p1[2]+45,0,0,0]
        print("point2bottomextra=",point2bottomextra)

        #Left Points
        pointleft1=[(p1[0]/2)+3.5,p4[1],p1[2]+45,0,0,0]
        print("pointleft1=",pointleft1)
        pointleft1pre=[(p1[0]/2)+2+5,p4[1],p1[2]+45,0,0,0]
        print("pointleft1pre=",pointleft1pre)
        pointleft12=[(p1[0]/2)+3.5,p4[1]+1+1,p1[2]+45,0,0,0]
        print("pointleft12=",pointleft12)
        pointleft2=[(p1[0]/2)+3.5,p2[1],p1[2]+45,0,0,0]
        print("pointleft2=",pointleft2)
        pointleft2pre=[(p1[0]/2)+2+5+10,p2[1],p1[2]+45,0,0,0]
        print("pointleft2pre=",pointleft2pre)
        #Left Extra Points
        pointLeftExtra=[(p1[0]/2)+21,p2[1]+10,p1[2]+15,0,0,90]
        print("pointLeftExtra=",pointLeftExtra)

        
        #Top Cycle Points
        pointtop1=[(p1[0]/2),p2[1]+4,p1[2]+45,0,0,90]
        print("pointtop1=",pointtop1)
        pointtop1pre=[(p1[0]/2),p2[1]+2+8,p1[2]+45,0,0,90]
        print("pointtop1pre=",pointtop1pre)
        pointtop12=[(p1[0]/2)-1-1,p2[1]+4,p1[2]+45,0,0,90]
        print("pointtop12=",pointtop12)
        pointtop2=[(p4[0]/2),p2[1]+4,p1[2]+45,0,0,90]
        print("pointtop2=",pointtop2)
        pointtop2pre=[(p4[0]/2),p2[1]+2+8+10,p1[2]+45,0,0,90]
        print("pointtop2pre =",pointtop2pre)
        pointtop2air=[(p4[0]/2),p2[1]+2+3+10,p1[2]+45,0,0,90]
        print("pointtop2air =",pointtop2air)
        pointtop1Combo=[(p1[0]/2),p2[1]+2+3+10,p1[2]+45,0,0,90]
        print("pointtop1Combo =",pointtop1Combo)

        #TopMain Minus Points
        pointtop3=[-(p1[0]/2),p2[1]+3.5,p1[2]+45,0,0,90]
        print("pointtop3=",pointtop3)
        pointtop3pre=[-(p1[0]/2),p2[1]+2+8+10,p1[2]+45,0,0,90]
        print("pointtop3pre =",pointtop3pre)
        pointtop3air=[-(p1[0]/2),p2[1]+2+3+10,p1[2]+45,0,0,90]
        print("pointtop3air =",pointtop3air)

        
        #Top Cycle Points Extra
        pointtopExtra=[-(p1[0]/2)-6,p2[1]+2+3+10,p1[2]+15,0,0,180]
        print("pointtopExtra =",pointtopExtra)

        #RightCycle
        pointright1=[(p4[0]/2)-4,p2[1],p1[2]+45,0,0,180]
        print("pointright1 =",pointright1)
        pointright1pre=[(p4[0]/2)-1-4,p2[1],p1[2]+45,0,0,180]
        print("pointright1pre =",pointright1pre)
        pointright12=[(p4[0]/2)-4,p2[1]-1-1,p1[2]+45,0,0,180]
        print("pointright12 =",pointright12)
        pointright2=[(p4[0]/2)-4,p4[0],p1[2]+45,0,0,180]
        print("pointright2 =",pointright2)
        pointright2pre=[(p4[0]/2)-1-4-10,p4[0],p1[2]+45,0,0,180]
        print("pointright2pre =",pointright2pre)
        homelast=[0,0,-50,0,0,180]
        print("homelast =",homelast)


        #Converyer Points for X axis
        cx=p4[0]
        print("cx=",cx)
        incDistance=(p1[0]-p4[0])/2
        print("incDistance=",incDistance)
        cx1=incDistance
        print("cx1=",cx1)
        cx2=incDistance*2
        print("cx2=",cx2)
        cx3=incDistance*3
        print("cx3=",cx3)
        cx4=incDistance*4
        print("cx4=",cx4)
        cx5=incDistance*5
        print("cx5=",cx5)
        cx6=incDistance*6
        print("cx6=",cx6)
        cx7=incDistance*7
        print("cx7=",cx7)
        cx8=incDistance*8
        print("cx8=",cx8)

        #Conveyer for the TOP
        #Converyer Points for X axis
        tcx=p4[0]
        print("tcx=",tcx)
        tincDistance=(p1[0]-p4[0])/2
        print("tincDistance=",tincDistance)
        tcx1=tincDistance
        print("tcx1=",tcx1)
        tcx2=tincDistance*2
        print("tcx2=",tcx2)
        tcx3=tincDistance*3
        print("tcx3=",tcx3)
        tcx4=tincDistance*4
        print("tcx4=",tcx4)
        tcx5=tincDistance*5
        print("tcx5=",tcx5)
        tcx6=tincDistance*6
        print("tcx6=",tcx6)
        tcx7=tincDistance*7
        print("tcx7=",tcx7)
        tcx8=tincDistance*8
        print("tcx8=",tcx8)
        tcx9=tincDistance*9
        print("tcx9=",tcx9)
        tcx10=tincDistance*10
        print("tcx10=",tcx10)
        tcx11=tincDistance*11
        print("tcx11=",tcx11)
        tcx12=tincDistance*12
        print("tcx12=",tcx12)




        #Array of points
        pointsb=[point1pre,point1,point12,point2,point2pre]
        print("pointsb=",pointsb)
        pointsleft=[pointleft1pre,pointleft1,pointleft12,pointleft2,pointleft2pre]
        print("pointsleft=",pointsleft)
        #pointstop=[pointtop1pre,pointtop1,pointtop12,pointtop2,pointtop2pre]
        #print("pointstop=",pointstop)
        pointstopmain=[pointtop1pre,pointtop1,pointtop12,pointtop3,pointtop3pre]
        print("pointstopmain=",pointstopmain)
        pointsright=[pointright1pre,pointright1,pointright12,pointright2,pointright2pre]
        print("pointsright=",pointsright)

        def perform_process_bottom(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==point12:putForceYplus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==point2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sanding",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        def perform_process_left(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointleft12:putForceXminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointleft2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sanding",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointtop12:putForceYminus1(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointtop3:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sanding",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)
        
        
        def perform_process_right(cps, config, points1,force):
            # Vibration on
            # turn_vibration_on(cps)
            
            # Force Control Activated
            #putForceZplus(
                #cps=cps,
                #force=15,
                #tcp=config['coords']['tcpReal'],
                #ucs=config['coords']['ucsTable2'],
                #config=config
            #)
            
            # Communicate to each point in points1
            for point in points1:
                if point==pointright12:putForceXplus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                config=config
                )
                if point==pointright2:turn_vibration_on(cps)
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcpSideTool'],
                    ucs=config['coords']['ucsTable2'],
                    seventh=-1,
                    speed=sanding_speed,
                    velocity_profile="sanding",
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            
            # Release Force Control
            releaseForce(cps=cps, config=config)

        def run_single_movement(robot_point, seventh_axis_point, cps, config):
            # Safety: lift/position tool before asynchronous 7th-axis motion.
            communicate(
                cps=cps,
                config=config,
                point=robot_point,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speeed,
                velocity_profile="robot",
                wait=True
            )
            communicate(
                cps=cps,
                config=config,
                seventh=seventh_axis_point,
                tcp=config['coords']['tcpSideTool'],
                ucs=config['coords']['ucsTable2'],
                speed=speeed,
                velocity_profile="robot",
                wait=False
            )
        selected_section = section.lower() if isinstance(section, str) else None

        if pass_index is not None:
            current_pass_index = int(pass_index)
            if current_pass_index < 0:
                return False
            if selected_section == "bottom":
                bottom_followup_passes = [cx1]
                if current_pass_index == 0:
                    communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                    communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed, velocity_profile="robot",wait=False)
                    perform_process_bottom(cps, config, points1=pointsb,force=force)
                elif current_pass_index <= len(bottom_followup_passes):
                    cx_val = bottom_followup_passes[current_pass_index - 1]
                    communicate(cps=cps,config=config,point=point2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                    run_single_movement(robot_point=point1Combo, seventh_axis_point=cx_val, cps=cps, config=config)
                    perform_process_bottom(cps, config, points1=pointsb,force=force)
                else:
                    return False
            elif selected_section == "left":
                if current_pass_index != 0:
                    return False
                communicate(cps=cps,config=config,point=point2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                if 'tcx3' in locals():
                    communicate(cps=cps,config=config,seventh=tcx3,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed, velocity_profile="robot",wait=False)
                communicate(cps=cps,config=config,point=point2bottomextra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                perform_process_left(cps, config, points1=pointsleft,force=force)
                communicate(cps=cps,config=config,point=pointLeftExtra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
            elif selected_section == "top":
                top_passes = [tcx1]
                if current_pass_index >= len(top_passes):
                    return False
                cx_val = top_passes[current_pass_index]
                communicate(cps=cps,config=config,point=pointtop3air if 'pointtop3air' in locals() else pointtop2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                run_single_movement(robot_point=pointtop1Combo, seventh_axis_point=cx_val, cps=cps, config=config)
                perform_process_top(cps, config, points1=pointstopmain,force=force)
                communicate(cps=cps,config=config,point=pointtop3air if 'pointtop3air' in locals() else pointtop2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                if current_pass_index == len(top_passes) - 1:
                    communicate(cps=cps,config=config,point=pointtopExtra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
            elif selected_section == "right":
                if current_pass_index != 0:
                    return False
                if 'pointsright' in locals():
                    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed, velocity_profile="robot",wait=False)
                    perform_process_right(cps, config, points1=pointsright,force=force)
            else:
                raise ValueError(f"Unknown Tool 2 section: {section}")
            if return_home:
                communicate(cps=cps,config=config,seventh=-19,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed, velocity_profile="robot",wait=False)
                communicate(cps=cps,config=config,point=homelast,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed, velocity_profile="robot",wait=True)
                moveOnlyJ6r(cps, -326, config)
            return True

        if selected_section in (None, "bottom"):
            communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed,velocity_profile="robot",wait=False)
            communicate(cps=cps,config=config,point=pointhome,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)
            perform_process_bottom(cps, config, points1=pointsb,force=force)
            cx_points = [cx1]
            for cx in cx_points:
                communicate(cps=cps, config=config, point=point2air, tcp=config['coords']['tcpSideTool'], ucs=config['coords']['ucsTable2'], seventh=-1, speed=speeed,velocity_profile="robot",wait=True)
                run_single_movement(robot_point=point1Combo, seventh_axis_point=cx, cps=cps, config=config)
                perform_process_bottom(cps, config, points1=pointsb,force=force)

        if selected_section in (None, "left"):
            communicate(cps=cps,config=config,point=point2air,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)
            communicate(cps=cps,config=config,point=point2bottomextra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)
            perform_process_left(cps, config, points1=pointsleft,force=force)
            communicate(cps=cps,config=config,point=pointLeftExtra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)

        if selected_section in (None, "top"):
            cx_points = [tcx1]
            for cx in cx_points:
                run_single_movement(robot_point=pointtop1Combo, seventh_axis_point=cx, cps=cps, config=config)
                perform_process_top(cps, config, points1=pointstopmain,force=force)
                communicate(cps=cps, config=config, point=pointtop3air, tcp=config['coords']['tcpSideTool'], ucs=config['coords']['ucsTable2'], seventh=-1, speed=speeed,velocity_profile="robot",wait=True)
            communicate(cps=cps,config=config,point=pointtopExtra,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)

        if selected_section in (None, "right"):
            communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed,velocity_profile="robot",wait=False)
            perform_process_right(cps, config, points1=pointsright,force=force)

        if return_home:
            communicate(cps=cps,config=config,seventh=-19,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],speed=speeed,velocity_profile="robot",wait=False)
            communicate(cps=cps,config=config,point=homelast,tcp=config['coords']['tcpSideTool'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,velocity_profile="robot",wait=True)
            moveOnlyJ6r(cps, -326, config)
    #Main Cycle
    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 1000:
            return testmodel2tool2sidesbigfunction(force,cps,section=section,return_home=return_home,pass_index=pass_index)
        else:
            return testmodel2tool2sidesmallfunction(force,cps,section=section,return_home=return_home,pass_index=pass_index)
    else:
        print(f"Invalid ylen value type: {type(xlen)} - expected number or 'null'")
    
    


if __name__ == "__main__":
    testmodel2tool2sidesrun(force=5)
    

