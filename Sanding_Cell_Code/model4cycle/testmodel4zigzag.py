import sys
import os
import json
import time

# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import math
import time
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,moveOnlyJ6r,putForceYplus1,putForceXminus,putForceYminus1,putForceZplus
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
import threading
from Table4Model.exportpointsmodule import exported_points

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

def testmodel4zigzagfunction(force,innerSandingOffset,cps):


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
    # p9 = exported_points["p9"]
    # p10 = exported_points["p10"]
    # p11 = exported_points["p11"]
    # p12= exported_points["p12"]
    
    p5 = exported_points["p5"]
    p6 = exported_points["p6"]
    p7 = exported_points["p7"]
    p8= exported_points["p8"]

    # print("p9:",p9)
    # print("p10:",p10)
    # print("p11:",p11)
    # print("p12:",p12)

    print("p5:",p5)
    print("p6:",p6)
    print("p7:",p7)
    print("p8:",p8)
    json_config = load_json_config()
    speeed = float(json_config['robotSpeed'])
    pocketzigzag_cfg = json_config.get("TableB", {}).get("pocketzigzag", {})
    zigzag_orientation = str(pocketzigzag_cfg.get("orientation", "vertical")).lower()
    if pocketzigzag_cfg.get("horizontalSpiral"):
        zigzag_orientation = "horizontal"
    elif pocketzigzag_cfg.get("verticalSpiral"):
        zigzag_orientation = "vertical"
    if zigzag_orientation not in ("horizontal", "vertical"):
        zigzag_orientation = "vertical"
    edge_coverage = bool(pocketzigzag_cfg.get("edgeCoverage", pocketzigzag_cfg.get("edge", False)))
    edge_offset = float(pocketzigzag_cfg.get("edgeOffset", 1.75))
    print(speeed)


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

    # point13=[-p9[0],p9[1],-p9[2]-2.52,p9[3],p9[4],p9[5]]
    # point14=[-p10[0],p10[1],-p10[2]-2.52,p10[3],p10[4],p10[5]]
    # point15=[-p11[0],p11[1],-p11[2]-2.52,p11[3],p11[4],p11[5]]
    # point16=[-p12[0],p12[1],-p12[2]-2.52,p12[3],p12[4],p12[5]]
    # print("point13=",point13)
    # print("point14=",point14)
    # print("point15=",point15)
    # print("point16=",point16)

    #Second Pocket Points
    point5=[-p5[0],p5[1],p5[2]-4,p5[3],p5[4],p5[5]]
    point6=[-p6[0],p6[1],p6[2]-4,p6[3],p6[4],p6[5]]
    point7=[-p7[0],p7[1],p7[2]-4,p7[3],p7[4],p7[5]]
    point8=[-p8[0],p8[1],p8[2]-4,p8[3],p8[4],p8[5]]
    print("point5=",point5)
    print("point6=",point6)
    print("point7=",point7)
    print("point8=",point8)


    # #Sample Pocket4
    # dispocket4= point13[0]-point15[0]
    # #cx=abs(point15[0])
    # point15u= [0,point15[1],point15[2],point15[3],point15[4],point15[5]]
    # point16u= [0,point16[1],point16[2],point16[3],point16[4],point16[5]]
    # point13u= [dispocket4,point13[1],point13[2],point13[3],point13[4],point13[5]]
    # point14u= [dispocket4,point14[1],point14[2],point14[3],point14[4],point14[5]]
    # #pointpre=[-dispocket4-0.5,point14[1]-0.5,point14[2],point14[3],point14[4],point14[5]]

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

    # #Conveyer Movement for Robot
    # cx=abs(point15[0])
    # print("cx=",cx)
    # cdistance=abs(point14[0])-abs(point15[0])
    # print("cdistance=",cdistance)
    # thirdcdistance=cdistance/3
    # print("thirdcdistance=",thirdcdistance)
    # cx1=cx+thirdcdistance
    # print("cx1=",cx1)
    # cx2=cx1+thirdcdistance
    # print("cx2=",cx2)
    # cx3=cx2+thirdcdistance
    # print("cx3=",cx3)

    
    #Converyer For ROBOT Movement Pocket 2
    tcx0=abs(point8[0])
    print("tcx=",tcx0)
    tcdistance=abs(point6[0])-abs(point7[0])
    print("tcdistance=",tcdistance)
    tthirdcdistance=tcdistance/5
    print("tthirdcdistance=",tthirdcdistance)
    tcx1=tcx0+tthirdcdistance
    print("tcx1=",tcx1)
    tcx2=tcx1+tthirdcdistance
    print("tcx2=",tcx2)
    tcx3=tcx2+tthirdcdistance
    print("tcx3=",tcx3)
    tcx4=tcx3+tthirdcdistance
    print("tcx4=",tcx4)
    tcx5=tcx4+tthirdcdistance
    print("tcx5=",tcx5)


    # Parameters (adjust as needed)
    #tool3y = 50.8   # Tool offset in Y
    #tool3x = 38.1   # Tool offset in X
    #innerOffset = 5  # Inner boundary offset
    #innerSandingOffset = 50  # Step size in X (instead of Y)
    #xframe_1 = 0
    #xframe_2 = 0

    # # 1) Collect boundary coordinates as [x, y, z]
    # x_coords = [point13u[0], point14u[0], point15u[0], point16u[0]]
    # y_coords = [point13u[1], point14u[1], point15u[1], point16u[1]]
    # z_coords = [point13u[2], point14u[2], point15u[2], point16u[2]]

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

    def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX,innerSandingOffset, orientation="vertical"):
        prepoint = None
        zigzag_coords = []
        
        # Parameters (adjust as needed)
        tool3y = 50.8   # Tool offset in Y
        tool3x = 38.1   # Tool offset in X
        innerSandingOffset = innerSandingOffset  # Step size in X (instead of Y)
        xframe_1 = 0
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
                (x_coords[1])/5 + tool3x + innerOffsetX,
                y_coords[1] - tool3y - innerOffset,
            ]
            modified_Point3 = [
                x_coords[2] - tool3x - innerOffset,
                y_coords[2] - tool3y - innerOffset,
            ]
            modified_Point1 = [
                (x_coords[0])/5 + tool3x + innerOffsetX,
                y_coords[0] + tool3y + innerOffset,
            ]
            modified_Point4 = [
                x_coords[3] - tool3x - innerOffset,
                y_coords[3] + tool3y + innerOffset,
            ]

            x_min = min(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
            x_max = max(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
            y_min = min(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])
            y_max = max(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])

            orientation_mode = (orientation or "vertical").lower()
            if orientation_mode not in ("horizontal", "vertical"):
                orientation_mode = "vertical"

            if orientation_mode == "horizontal":
                y_span = abs(y_max - y_min)
                num_steps = max(1, math.ceil(y_span / innerSandingOffset))
                adjusted_step = y_span / num_steps

                offset = 0.0
                toggle = 0
                while offset <= y_span + 1e-9:
                    y_val = y_max - offset
                    row_points = [
                        [x_min, y_val, z_zigzag, 180, 0, 0],
                        [x_max, y_val, z_zigzag, 180, 0, 0],
                    ]
                    if toggle:
                        row_points.reverse()
                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                xinner = abs(x_max - x_min) - xframe_1 - xframe_2
                print("xinner=", xinner)

                if xinner > 0:
                    # Determine how many "columns" in the zigzag
                    num_steps = math.ceil(xinner / innerSandingOffset)
                    adjusted_step = xinner / num_steps

                    offset = 0.0
                    toggle = 0

                    # Build zigzag path from left to right
                    while offset <= xinner + 1e-9:  # small floating-point tolerance
                        x_val = x_min + offset
                        row_points = [
                            [x_val, y_min, z_zigzag, 180, 0, 0],
                            [x_val, y_max, z_zigzag, 180, 0, 0],
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
        
            if zigzag_coords:
                start_point = zigzag_coords[0]
                prepoint = [abs(start_point[0]) + 0.5, start_point[1], z_zigzag, 180, 0, 0]
        return zigzag_coords,prepoint

    def generate_edge_paths(x_coords, y_coords, z_coords, edge_offset=1.75):
        if not (x_coords and y_coords and z_coords):
            return ([], None), ([], None)

        tool3y = 50.8
        tool3x = 38.1
        z_edge = z_coords[0]
        rx, ry, rz = 180, 0, 0

        edge_Point2 = [
            (x_coords[1]) / 5 + tool3x + edge_offset,
            y_coords[1] - tool3y - edge_offset,
        ]
        edge_Point3 = [
            x_coords[2] - tool3x - edge_offset,
            y_coords[2] - tool3y - edge_offset,
        ]
        edge_Point1 = [
            (x_coords[0]) / 5 + tool3x + edge_offset,
            y_coords[0] + tool3y + edge_offset,
        ]
        edge_Point4 = [
            x_coords[3] - tool3x - edge_offset,
            y_coords[3] + tool3y + edge_offset,
        ]

        x_min = min(edge_Point1[0], edge_Point2[0], edge_Point3[0], edge_Point4[0])
        x_max = max(edge_Point1[0], edge_Point2[0], edge_Point3[0], edge_Point4[0])
        y_min = min(edge_Point1[1], edge_Point2[1], edge_Point3[1], edge_Point4[1])
        y_max = max(edge_Point1[1], edge_Point2[1], edge_Point3[1], edge_Point4[1])

        x_mid = (x_min + x_max) / 2.0

        top_left = [x_min, y_max, z_edge, rx, ry, rz]
        top_right = [x_max, y_max, z_edge, rx, ry, rz]
        bottom_left = [x_min, y_min, z_edge, rx, ry, rz]
        bottom_right = [x_max, y_min, z_edge, rx, ry, rz]
        top_mid = [x_mid, y_max, z_edge, rx, ry, rz]
        bottom_mid = [x_mid, y_min, z_edge, rx, ry, rz]

        edge_pass1 = [top_mid, top_right, bottom_right, bottom_mid]
        edge_pass2 = [bottom_mid, bottom_left, top_left, top_mid]

        for point in edge_pass1 + edge_pass2:
            point[0] = abs(point[0])
            point[1] = abs(point[1])

        start1 = edge_pass1[0] if edge_pass1 else None
        start2 = edge_pass2[0] if edge_pass2 else None
        return (edge_pass1, start1), (edge_pass2, start2)
    #3rd Cycle for Door 1
    # zigzag_path,prepoint= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=5)
    # print("zigzag_path=",zigzag_path)
    # print("Prepoint:", prepoint)
    # #2nd Cycle for Door 1
    # zigzag_path1,prepoint1= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=-10)
    # print("zigzag_path=",zigzag_path1)
    # print("Prepoint:", prepoint1)
    # #1st Cycle for Door 1
    # zigzag_path2,prepoint2= generate_zigzag_path(x_coords=x_coords, y_coords=y_coords, z_coords=z_coords, innerOffset=5,innerOffsetX=-10)
    # print("zigzag_path2=",zigzag_path2)
    # print("Prepoint2:", prepoint2)

    #Second Pocket 1st Cycle
    zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=15,innerSandingOffset=innerSandingOffset, orientation=zigzag_orientation)
    print("zigzag_pathp=",zigzag_pathp1)
    print("prepointp:", prepointp1)
    #Second Pocket 2nd Cycle
    zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=15,innerSandingOffset=innerSandingOffset, orientation=zigzag_orientation)
    print("zigzag_pathp2=",zigzag_pathp2)
    print("prepointp2:", prepointp2)
    #Second Pocket 3rd Cycle
    zigzag_pathp3,prepointp3= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=15,innerSandingOffset=innerSandingOffset, orientation=zigzag_orientation)
    print("zigzag_pathp3=",zigzag_pathp3)
    print("prepointp3:", prepointp3)
    #Second Pocket 4th Cycle
    zigzag_pathp4,prepointp4= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=15,innerSandingOffset=innerSandingOffset, orientation=zigzag_orientation)
    print("zigzag_pathp4=",zigzag_pathp4)
    print("prepointp4:", prepointp4)
    #Second Pocket 5th Cycle
    zigzag_pathp5,prepointp5= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=15,innerSandingOffset=innerSandingOffset, orientation=zigzag_orientation)
    print("zigzag_pathp5=",zigzag_pathp5)
    print("prepointp5:", prepointp5)
    #Second Pocket 6th Cycle
    # zigzag_pathp6,prepointp6= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=5,innerSandingOffset=innerSandingOffset)
    # print("zigzag_pathp6=",zigzag_pathp6)
    # print("prepointp6:", prepointp6)
    (edge_pathp1, edge_startp1), (edge_pathp2, edge_startp2) = generate_edge_paths(
        x_coords=x_coords1,
        y_coords=y_coords1,
        z_coords=z_coords1,
        edge_offset=edge_offset,
    )


    def perform_process_top(cps, config, points1):
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
    
    for i in range(1, 6):  # Loop from 1 to 5 (for p1-p5)
    # Get the current tcx (0-indexed)
        current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
        
        # Get the current prepoint and zigzag path (1-indexed)
        current_prepoint = eval(f"prepointp{i}")
        current_zigzag = eval(f"zigzag_pathp{i}")

        # Apply overlap offsets on middle passes to hide seam lines
        seam_offset = 0.0
        if 1 < i < 5:
            seam_offset = -25.0 if i % 2 == 0 else 25.0

        if seam_offset and current_prepoint:
            current_prepoint = [
                current_prepoint[0] + seam_offset,
                current_prepoint[1],
                current_prepoint[2],
                current_prepoint[3],
                current_prepoint[4],
                current_prepoint[5],
            ]

        if seam_offset and current_zigzag:
            current_zigzag = [
                [pt[0] + seam_offset, pt[1], pt[2], pt[3], pt[4], pt[5]]
                for pt in current_zigzag
            ]

        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=current_tcx, 
            tcp=config['coords']['tcpReal'], 
            ucs=config['coords']['ucsTable2'], 
            speed=speeed, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=spoint, 
            tcp=config['coords']['tcpReal'], 
            ucs=config['coords']['ucsTable2'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)
        edge_pathp = None
        edge_startp = None
        if i == 1:
            edge_pathp = edge_pathp1
            edge_startp = edge_startp1
        elif i == 5:
            edge_pathp = edge_pathp2
            edge_startp = edge_startp2

        if edge_coverage and edge_startp:
            edge_prepoint = [
                edge_startp[0] + 0.5,
                edge_startp[1],
                edge_startp[2],
                edge_startp[3],
                edge_startp[4],
                edge_startp[5],
            ]
            communicate(
                cps=cps,
                config=config,
                point=edge_prepoint,
                tcp=config['coords']['tcpReal'],
                ucs=config['coords']['ucsTable2'],
                seventh=-1,
                speed=speeed,
                wait=True,
            )
            perform_process_top(cps, config, points1=edge_pathp)
        communicate(
            cps=cps, config=config, 
            point=current_prepoint,  # Dynamic prepoint
            tcp=config['coords']['tcpReal'], 
            ucs=config['coords']['ucsTable2'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=current_zigzag)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=spoint, 
            tcp=config['coords']['tcpReal'], 
            ucs=config['coords']['ucsTable2'], 
            seventh=-1, 
            speed=speeed, wait=True
        )

    # # ------------------------------------------------------
    # # 3) Move the robot through the zigzag points
    # # ------------------------------------------------------
    # # Assuming 'cps' and 'config' are defined elsewhere in your code
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.8,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepoint2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_path2)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #Second Cycle for First Pocket
    # communicate(cps=cps,config=config,seventh=cx1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.8,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepoint1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_path1)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #Third Cycle of First Pocket
    # communicate(cps=cps,config=config,seventh=cx2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.8,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_path)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)

    # #Second Pocket
    # #Second Pocket First Cycle
    # #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # communicate(cps=cps,config=config,seventh=tcx0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.2,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepointp1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_pathp1)
    # # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)

    # #Second Pocket Second Cycle
    # #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # communicate(cps=cps,config=config,seventh=tcx1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.8,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # # turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepointp2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_pathp2)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)

    # #Second Pocket Third Cycle
    # #communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.2,wait=True)
    # communicate(cps=cps,config=config,seventh=tcx2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.8,wait=True)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # communicate(cps=cps,config=config,point=prepointp3,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)
    # #turn_vibration_on(cps)
    # perform_process_top(cps, config, points1=zigzag_pathp3)
    # #turn_vibration_off(cps)
    # communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.8,wait=True)

    # #Home Position
    # communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.2,wait=True)

    
if __name__ == "__main__":
    testmodel4zigzagfunction(force=5,innerSandingOffset=50)
