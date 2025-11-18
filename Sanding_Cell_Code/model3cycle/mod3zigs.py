# testcommu.py
import sys
import os
import json
import time
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))



import math
import time
import yaml
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForceZplus,releaseForce  # Ensure seventhGoToPos is imported
from modules.CPS import CPSClient
from Table3Model.exportpointsmodule import exported_points

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

def mod3zigsmall(force,innerSandingOffset,cps):


    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    # cps = CPSClient()
    # IP = config['server']['cpip']
    # port = config['server']['cps']
    # ret = cps.HRIF_Connect(0, IP, port)

    #p1 = exported_points["p1"]
    #p2 = exported_points["p2"]
    #p3 = exported_points["p3"]
    #p4 = exported_points["p4"]
    p9 = exported_points["p9"]
    p10 = exported_points["p10"]
    p11 = exported_points["p11"]
    p12= exported_points["p12"]
    
    p5 = exported_points["p5"]
    p6 = exported_points["p6"]
    p7 = exported_points["p7"]
    p8= exported_points["p8"]

    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    print(speeed)

    print("p9:",p9)
    print("p10:",p10)
    print("p11:",p11)
    print("p12:",p12)

    print("p5:",p5)
    print("p6:",p6)
    print("p7:",p7)
    print("p8:",p8)

    # Hard-coded points for Pocket4
    # Format: [x, y, z, rotX, rotY, rotZ]
    #Pocket4
    spoint=[166,213,-167,180,0,0]
    #p9= [704.8500000032, 57.1500000002, -9.525000000039999, 180, 0, 0]
    #p10= [704.8500000032, 323.8500000012, -9.525000000039999, 180, 0, 0]
    #p11= [57.1500000002, 323.8500000012, -9.525000000039999, 180, 0, 0]
    #p12= [57.1500000002, 57.1500000002, -9.525000000039999, 180, 0, 0]

    #Second Pocket Points
    #p5= [1955.800000003, 57.1500000002, -9.525000000039999, 180, 0, 0]
    #p6= [1955.800000003, 323.8500000012, -9.525000000039999, 180, 0, 0]
    #p7= [822.3250000029999, 323.8500000012, -9.525000000039999, 180, 0, 0]
    #p8= [822.3250000029999, 57.1500000002, -9.525000000039999, 180, 0, 0]

    point13=[-p9[0],p9[1],-p9[2]-5,p9[3],p9[4],p9[5]]
    point14=[-p10[0],p10[1],-p10[2]-5,p10[3],p10[4],p10[5]]
    point15=[-p11[0],p11[1],-p11[2]-5,p11[3],p11[4],p11[5]]
    point16=[-p12[0],p12[1],-p12[2]-5,p12[3],p12[4],p12[5]]
    print("point13=",point13)
    print("point14=",point14)
    print("point15=",point15)
    print("point16=",point16)

    #Second Pocket Points
    point5=[-p5[0],p5[1],-p5[2]-5,p5[3],p5[4],p5[5]]
    point6=[-p6[0],p6[1],-p6[2]-5,p6[3],p6[4],p6[5]]
    point7=[-p7[0],p7[1],-p7[2]-5,p7[3],p7[4],p7[5]]
    point8=[-p8[0],p8[1],-p8[2]-5,p8[3],p8[4],p8[5]]
    print("point5=",point5)
    print("point6=",point6)
    print("point7=",point7)
    print("point8=",point8)


    #Sample Pocket4
    dispocket4= point13[0]-point15[0]
    #cx=abs(point15[0])
    point15u= [0,point15[1],point15[2],point15[3],point15[4],point15[5]]
    point16u= [0,point16[1],point16[2],point16[3],point16[4],point16[5]]
    point13u= [dispocket4,point13[1],point13[2],point13[3],point13[4],point13[5]]
    point14u= [dispocket4,point14[1],point14[2],point14[3],point14[4],point14[5]]
    #pointpre=[-dispocket4-0.5,point14[1]-0.5,point14[2],point14[3],point14[4],point14[5]]

    # Sample pocket for Second
    dispocket4= point5[0]-point7[0]
    point5u= [dispocket4,point5[1],point5[2],point5[3],point5[4],point5[5]]
    point6u= [dispocket4,point6[1],point6[2],point6[3],point6[4],point6[5]]
    point7u= [0,point7[1],point7[2],point7[3],point7[4],point7[5]]
    point8u= [0,point8[1],point8[2],point8[3],point8[4],point8[5]]
    print("point5u=",point5u)
    print("point6u=",point6u)
    print("point7u=",point7u)
    print("point8u=",point8u)

    #Conveyer Movement for Robot
    cx=abs(point15[0])
    print("cx=",cx)
    cdistance=abs(point14[0])-abs(point15[0])
    print("cdistance=",cdistance)
    thirdcdistance=cdistance/2
    print("thirdcdistance=",thirdcdistance)
    cx1=cx+thirdcdistance
    print("cx1=",cx1)
    cx2=cx1+thirdcdistance
    print("cx2=",cx2)
    cx3=cx2+thirdcdistance
    print("cx3=",cx3)

    
    #Converyer For ROBOT Movement Pocket 2
    tcx=abs(point8[0])
    print("tcx=",tcx)
    tcdistance=abs(point6[0])-abs(point7[0])
    print("tcdistance=",tcdistance)
    tthirdcdistance=tcdistance/2
    print("tthirdcdistance=",tthirdcdistance)
    tcx1=tcx+tthirdcdistance
    print("tcx1=",tcx1)
    tcx2=tcx1+tthirdcdistance
    print("tcx2=",tcx2)
    tcx3=tcx2+tthirdcdistance
    print("tcx3=",tcx3)


    # Parameters (adjust as needed)
    #tool3y = 50.8   # Tool offset in Y
    #tool3x = 38.1   # Tool offset in X
    #innerOffset = 5  # Inner boundary offset
    #innerSandingOffset = 50  # Step size in X (instead of Y)
    #xframe_1 = 0
    #xframe_2 = 0

    # 1) Collect boundary coordinates as [x, y, z]
    x_coords = [point13u[0], point14u[0], point15u[0], point16u[0]]
    y_coords = [point13u[1], point14u[1], point15u[1], point16u[1]]
    z_coords = [point13u[2], point14u[2], point15u[2], point16u[2]]

    # 1) Collect boundary coordinates as [x, y, z]
    x_coords1 = [point5u[0], point6u[0], point7u[0], point8u[0]]
    y_coords1 = [point5u[1], point6u[1], point7u[1], point8u[1]]
    z_coords1 = [point5u[2], point6u[2], point7u[2], point8u[2]]

    print("x_coords1 ",x_coords1)
    print("y_coords2 ",y_coords1)
    print("z_coords3",z_coords1)

    # Global variables
    #prepoint = None
    #zigzag_coords = []

    def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX,innerSandingOffset):
        prepoint = None
        zigzag_coords = []
        
        # Parameters (adjust as needed)
        tool3y = 50.8   # Tool offset in Y
        tool3x = 38.1   # Tool offset in X
        innerSandingOffset =innerSandingOffset  # Step size in X (instead of Y)
        xframe_1 = 7
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
                (x_coords[1])/2 + tool3x + innerOffsetX,
                y_coords[1] - tool3y - innerOffset,
            ]
            modified_Point3 = [
                x_coords[2] - tool3x - innerOffset,
                y_coords[2] - tool3y - innerOffset,
            ]
            modified_Point1 = [
                (x_coords[0])/2 + tool3x + innerOffsetX,
                y_coords[0] + tool3y + innerOffset*2,
            ]
            modified_Point4 = [
                x_coords[3] - tool3x - innerOffset,
                y_coords[3] + tool3y + innerOffset,
            ]

            # Calculate available horizontal dimension
            xlen1 = abs(modified_Point3[0] - modified_Point1[0])
            xinner = xlen1 - xframe_1 - xframe_2
            print("xinner=", xinner)

            if xinner > 0:
                # Determine how many "columns" in the zigzag
                num_steps = math.ceil(xinner / innerSandingOffset)
                adjusted_step = xinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag path from left to right
                while offset <= xinner + 1e-9:  # small floating-point tolerance
                    row_points = [
                        [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 180, 0, 0],
                        [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 180, 0, 0],
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
        
            prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 180, 0, 0]  
        return zigzag_coords,prepoint
    # #3rd Cycle for Door 1
    # zigzag_path,prepoint= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=5)
    # print("zigzag_path=",zigzag_path)
    # print("Prepoint:", prepoint)
    #2nd Cycle for Door 1
    zigzag_path1,prepoint1= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=5,innerSandingOffset=innerSandingOffset)
    print("zigzag_path=",zigzag_path1)
    print("Prepoint:", prepoint1)
    #1st Cycle for Door 1
    zigzag_path2,prepoint2= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=-10,innerSandingOffset=innerSandingOffset)
    print("zigzag_path2=",zigzag_path2)
    print("Prepoint2:", prepoint2)

    #Second Pocket 1st Cycle
    zigzag_pathp,prepointp= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10,innerSandingOffset=innerSandingOffset)
    print("zigzag_pathp=",zigzag_pathp)
    print("prepointp:", prepointp)
    #Second Pocket 2nd Cycle
    zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=5,innerSandingOffset=innerSandingOffset)
    print("zigzag_pathp2=",zigzag_pathp2)
    print("prepointp2:", prepointp2)
    # #Second Pocket 3rd Cycle
    # zigzag_pathp3,prepointp3= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=5)
    # print("zigzag_pathp3=",zigzag_pathp3)
    # print("prepointp3:", prepointp3)

    


    def perform_process_top(cps, config, points1,force):
        # Vibration on
        
        
        # Force Control Activated
        putForceZplus(
            cps=cps,
            force=force,
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            config=config
        )
        turn_vibration_on(cps)
        
        # Communicate to each point in points1
        for point in points1:
            communicate(
                cps=cps,
                config=config,
                point=point,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=0.6,
                wait=False
            )
        
        # Wait for blending and turn off vibration
        waitForBlending(cps=cps, config=config)
        turn_vibration_off(cps)
        #Release force
        releaseForce(cps=cps, config=config)

    # ------------------------------------------------------
    # 3) Move the robot through the zigzag points
    # ------------------------------------------------------
    # Assuming 'cps' and 'config' are defined elsewhere in your code
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # turn_vibration_on(cps)
    communicate(cps=cps,config=config,point=prepoint2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    #turn_vibration_on(cps)
    perform_process_top(cps, config, points1=zigzag_path2)
    #turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    #Second Cycle for First Pocket
    communicate(cps=cps,config=config,seventh=cx1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # turn_vibration_on(cps)
    communicate(cps=cps,config=config,point=prepoint1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    #turn_vibration_on(cps)
    perform_process_top(cps, config, points1=zigzag_path1)
    #turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # #Third Cycle of First Pocket
    # communicate(cps=cps,config=config,seventh=cx2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_path)
    #turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)

    #Second Pocket
    #Second Pocket First Cycle
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    communicate(cps=cps,config=config,seventh=tcx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # turn_vibration_on(cps)
    communicate(cps=cps,config=config,point=prepointp,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    #turn_vibration_on(cps)
    perform_process_top(cps, config, points1=zigzag_pathp)
    #turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)

    #Second Pocket Second Cycle
    #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    communicate(cps=cps,config=config,seventh=tcx1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # turn_vibration_on(cps)
    communicate(cps=cps,config=config,point=prepointp2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    #turn_vibration_on(cps)
    perform_process_top(cps, config, points1=zigzag_pathp2)
    #turn_vibration_off(cps)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)

    # #Second Pocket Third Cycle
    # #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # communicate(cps=cps,config=config,seventh=tcx2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # #turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepointp3,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_pathp3)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=speeed,wait=True)

    #Home Position
    communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)

    
if __name__ == "__main__":
    mod3zigsmall()