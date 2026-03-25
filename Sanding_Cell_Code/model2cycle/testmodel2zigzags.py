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
from Table2Model.exportpointsmodule import exported_points
from model2cycle.testmodel2zigzagb import model2zigzagbig

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

def model2zigzagrun(force, innerSandingOffset,cps):
    def testmodel2zigzagsmallfunction(force,innerSandingOffset,cps):


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
        pocketzigzag_cfg = json_config.get('TableB', {}).get('pocketzigzag', {})
        zigzag_orientation = str(pocketzigzag_cfg.get("orientation", "vertical")).lower()
        if pocketzigzag_cfg.get("horizontalSpiral"):
            zigzag_orientation = "horizontal"
        elif pocketzigzag_cfg.get("verticalSpiral"):
            zigzag_orientation = "vertical"
        if zigzag_orientation not in ("horizontal", "vertical"):
            zigzag_orientation = "vertical"
        edge_coverage = bool(pocketzigzag_cfg.get('edgeCoverage', pocketzigzag_cfg.get('edge', False)))
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
        point5=[-p5[0],p5[1],p5[2]-2.52,p5[3],p5[4],p5[5]]
        point6=[-p6[0],p6[1],p6[2]-2.52,p6[3],p6[4],p6[5]]
        point7=[-p7[0],p7[1],p7[2]-2.52,p7[3],p7[4],p7[5]]
        point8=[-p8[0],p8[1],p8[2]-2.52,p8[3],p8[4],p8[5]]
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
        tthirdcdistance=tcdistance/2
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
        tcx6=tcx5+tthirdcdistance
        print("tcx6=",tcx6)


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

        def generate_zigzag_edge_path(
            x_coords,
            y_coords,
            z_coords,
            innerOffset,
            innerOffsetX,
            innerSandingOffset,
            orientation="vertical",
            edge_coverage=False,
            edge_offset=1.75,
        ):
            if not (x_coords and y_coords and z_coords):
                return [], [], None

            tool3y = 50.8
            tool3x = 38.1
            z_zigzag = z_coords[0]
            rx, ry, rz = 180, 0, 0

            modified_Point2 = [
                (x_coords[1]) + tool3x + innerOffsetX,
                y_coords[1] - tool3y - innerOffset,
            ]
            modified_Point3 = [
                x_coords[2] - tool3x - innerOffset,
                y_coords[2] - tool3y - innerOffset,
            ]
            modified_Point1 = [
                (x_coords[0]) + tool3x + innerOffsetX,
                y_coords[0] + tool3y + innerOffset,
            ]
            modified_Point4 = [
                x_coords[3] - tool3x - innerOffset,
                y_coords[3] + tool3y + innerOffset,
            ]

            x_min = min(
                modified_Point1[0],
                modified_Point2[0],
                modified_Point3[0],
                modified_Point4[0],
            )
            x_max = max(
                modified_Point1[0],
                modified_Point2[0],
                modified_Point3[0],
                modified_Point4[0],
            )
            y_min = min(
                modified_Point1[1],
                modified_Point2[1],
                modified_Point3[1],
                modified_Point4[1],
            )
            y_max = max(
                modified_Point1[1],
                modified_Point2[1],
                modified_Point3[1],
                modified_Point4[1],
            )

            orientation_mode = (orientation or "vertical").lower()
            if orientation_mode not in ("horizontal", "vertical"):
                orientation_mode = "vertical"

            edge_Point2 = [
                (x_coords[1]) + tool3x + edge_offset,
                y_coords[1] - tool3y - edge_offset,
            ]
            edge_Point3 = [
                x_coords[2] - tool3x - edge_offset,
                y_coords[2] - tool3y - edge_offset,
            ]
            edge_Point1 = [
                (x_coords[0]) + tool3x + edge_offset,
                y_coords[0] + tool3y + edge_offset,
            ]
            edge_Point4 = [
                x_coords[3] - tool3x - edge_offset,
                y_coords[3] + tool3y + edge_offset,
            ]

            x_min_edge = min(
                edge_Point1[0], edge_Point2[0], edge_Point3[0], edge_Point4[0]
            )
            x_max_edge = max(
                edge_Point1[0], edge_Point2[0], edge_Point3[0], edge_Point4[0]
            )
            y_min_edge = min(
                edge_Point1[1], edge_Point2[1], edge_Point3[1], edge_Point4[1]
            )
            y_max_edge = max(
                edge_Point1[1], edge_Point2[1], edge_Point3[1], edge_Point4[1]
            )

            edge_points = []
            if edge_coverage:
                if orientation_mode == "horizontal":
                    edge_points = [
                        [x_min_edge, y_max_edge, z_zigzag, rx, ry, rz],
                        [x_max_edge, y_max_edge, z_zigzag, rx, ry, rz],
                        [x_max_edge, y_min_edge, z_zigzag, rx, ry, rz],
                        [x_min_edge, y_min_edge, z_zigzag, rx, ry, rz],
                        [x_min_edge, y_max_edge, z_zigzag, rx, ry, rz],
                    ]
                else:
                    edge_points = [
                        [x_max_edge, y_min_edge, z_zigzag, rx, ry, rz],
                        [x_min_edge, y_min_edge, z_zigzag, rx, ry, rz],
                        [x_min_edge, y_max_edge, z_zigzag, rx, ry, rz],
                        [x_max_edge, y_max_edge, z_zigzag, rx, ry, rz],
                        [x_max_edge, y_min_edge, z_zigzag, rx, ry, rz],
                    ]

            zigzag_points = []
            inner_step = max(1e-6, float(innerSandingOffset))
            toggle = 0

            if orientation_mode == "horizontal":
                span = abs(y_max - y_min)
                num_steps = max(1, math.ceil(span / inner_step))
                step = span / num_steps
                offset = 0.0
                while offset <= span + 1e-9:
                    y_val = y_max - offset
                    row = [
                        [x_min, y_val, z_zigzag, rx, ry, rz],
                        [x_max, y_val, z_zigzag, rx, ry, rz],
                    ]
                    if toggle:
                        row.reverse()
                    zigzag_points.extend(row)
                    offset += step
                    toggle = 1 - toggle
            else:
                span = abs(x_max - x_min)
                num_steps = max(1, math.ceil(span / inner_step))
                step = span / num_steps
                offset = 0.0
                while offset <= span + 1e-9:
                    x_val = x_min + offset
                    row = [
                        [x_val, y_min, z_zigzag, rx, ry, rz],
                        [x_val, y_max, z_zigzag, rx, ry, rz],
                    ]
                    if toggle:
                        row.reverse()
                    zigzag_points.extend(row)
                    offset += step
                    toggle = 1 - toggle

            for points in (edge_points, zigzag_points):
                for point in points:
                    point[0] = abs(point[0])
                    point[1] = abs(point[1])

            start_point = None
            if edge_points:
                start_point = edge_points[0]
            elif zigzag_points:
                start_point = zigzag_points[0]
            if start_point is None:
                return edge_points, zigzag_points, None

            prepoint = [
                abs(start_point[0]) + 0.5,
                start_point[1],
                start_point[2],
                start_point[3],
                start_point[4],
                start_point[5],
            ]
            return edge_points, zigzag_points, prepoint
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
        edge_pathp1, zigzag_pathp1, prepointp1 = generate_zigzag_edge_path(
            x_coords=x_coords1,
            y_coords=y_coords1,
            z_coords=z_coords1,
            innerOffset=15,
            innerOffsetX=15,
            innerSandingOffset=innerSandingOffset,
            orientation=zigzag_orientation,
            edge_coverage=edge_coverage,
            edge_offset=edge_offset,
        )
        print("edge_pathp1=", edge_pathp1)
        print("zigzag_pathp=", zigzag_pathp1)
        print("prepointp:", prepointp1)
        # Single pass only for pocket sanding (one zigzag path)
        # #Second Pocket 3rd Cycle
        # zigzag_pathp3,prepointp3= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        # print("zigzag_pathp3=",zigzag_pathp3)
        # print("prepointp3:", prepointp3)
        # #Second Pocket 4th Cycle
        # zigzag_pathp4,prepointp4= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        # print("zigzag_pathp4=",zigzag_pathp4)
        # print("prepointp4:", prepointp4)
        # #Second Pocket 5th Cycle
        # zigzag_pathp5,prepointp5= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        # print("zigzag_pathp5=",zigzag_pathp5)
        # print("prepointp5:", prepointp5)
        # #Second Pocket 6th Cycle
        # zigzag_pathp6,prepointp6= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=5)
        # print("zigzag_pathp6=",zigzag_pathp6)
        # print("prepointp6:", prepointp6)

        


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
        
        for i in range(1, 2):  # Single pass
        # Get the current tcx (0-indexed)
            current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
            
            # Get the current prepoint and zigzag path (1-indexed)
            current_prepoint = eval(f"prepointp{i}")
            current_zigzag = eval(f"zigzag_pathp{i}")
            current_edge = eval(f"edge_pathp{i}") if edge_coverage else []

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
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcpReal'], 
                ucs=config['coords']['ucsTable2'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            # # turn_vibration_on(cps)
            if current_edge:
                perform_process_top(cps, config, points1=current_edge,force=force)
            perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps, config=config, 
                point=spoint, 
                tcp=config['coords']['tcpReal'], 
                ucs=config['coords']['ucsTable2'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
        communicate(cps=cps,config=config,seventh=0,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=speeed,wait=True)
    #Main Cycle
    p1 = exported_points["p1"]
    xlen = p1[0]
    print("xlen:", xlen)
    # Check conditions
    if xlen == "null":
        print("No door data available - skipping operations")
    elif isinstance(xlen, (int, float)):  # Ensure it's numeric
        if xlen > 600:
            model2zigzagbig(force,cps)
        else:
            testmodel2zigzagsmallfunction(force, innerSandingOffset,cps)
    else:
        print(f"Invalid xlen value type: {type(xlen)} - expected number or 'null'")
    

    
if __name__ == "__main__":
    model2zigzagrun(force=30,innerSandingOffset=50)  # Example force and innerSandingOffset values
