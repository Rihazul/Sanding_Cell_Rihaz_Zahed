# testcommu.py

import sys
import math
import os
import time
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import config
import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus, setUCS_TCP
from modules.CPS import CPSClient  # Ensure CPSClient is properly defined
from smallTable.scancord import (
    read_scan_results,
    get_door_position,
    get_frame_point,
    get_inner_corner_point,
    get_outer_corner_point,
    get_pocket_point,
    get_x_values,
    get_y_values
)


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

def smalldoor1zizag(force,z,cps):
    
    def smalldoor1zizagsmall(force,z,cps):
        
        # Load configuration from YAML
        config = load_config()
        json_config = load_json_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(1)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        

        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 20  # Step size in X (instead of Y)
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
                    (x_coords[1])/1 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - (innerOffset*0.5),
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/1 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
                ]
                modified_Point4 = [
                    x_coords[3] - tool3x - innerOffset,
                    y_coords[3] + tool3y + innerOffset,
                ]
                print("modified_Point1:", modified_Point1)
                print("modified_Point2:", modified_Point2)
                print("modified_Point3:", modified_Point3)
                print("modified_Point4:", modified_Point4)

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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=10,innerOffsetX=10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            print("Turned Vibration On")
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=float(json_config['sandingSpeed']),
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            # waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor1zizagbig(force,z,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(1, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(1, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(1, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(1, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(1)
        print("tcx0:", tcx0)
        tcx1=tcx0+((p6[0]-p7[0])/2)
        print("tcx1:", tcx1)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 50  # Step size in X (instead of Y)
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
                
                modified_Point1 = [
                    (x_coords[0])/2 + tool3x + innerOffsetX ,
                    y_coords[0] + tool3y + innerOffset ,
                ]
                
                modified_Point2 = [
                    (x_coords[1])/2 + tool3x + innerOffsetX ,
                    y_coords[1] - tool3y - innerOffset ,
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset ,
                    y_coords[2] - tool3y - innerOffset ,
                ]
                
                modified_Point4 = [
                    x_coords[3] - tool3x - innerOffset ,
                    y_coords[3] + tool3y + innerOffset ,
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
                            [modified_Point1[0] + offset, modified_Point1[1] , z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1] , z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=-10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=15,innerOffsetX=4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force, enable_spiral=False, spiral_params=None):
            # Vibration on
            
            # Default spiral parameters
            if spiral_params is None:
                spiral_params = {
                    "dSpiralIncrement": 1.0,     # Rad80ius increment per revolution (mm)
                    "dSpiralDiameter": 5.0,     # Final spiral diameter (mm)
                    "dVelocity": 120.0,          # Speed (deg/s) - FAST
                    "dAcc": 200.0,               # Acceleration - FAST
                    "dRadius": 5.0,              # Transition radius (mm)
                    "spacing": 10.0,             # Distance between spiral points (mm)
                }     
                
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
          #  turn_vibration_on(cps)
           
           # Set TCP/UCS before weave motion
            setUCS_TCP(cps=cps, tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], config=config)
                   
            if enable_spiral and len(points1) >= 2:
                spiral_count = 0
                tcp_name = config['coords']['tcptool1plane1']
                ucs_name = config['coords']['ucsTable1']
                
                # Simple move helper - no customMoveL overhead
                def simple_move(point):
                    RawACSpoints = [0, 0, 0, 0, 0, 0]
                    cps.HRIF_MoveL(0, 0, point, RawACSpoints, tcp_name, ucs_name,
                                   config['coords']['roboVelocity'],
                                   config['coords']['roboAcceleration'],
                                   config['coords']['transitionRadius'],
                                   0, 0, 0, f"move{spiral_count}")
                    
                # For each segment between zigzag waypoints
                for seg_idx in range(len(points1) - 1):
                    # Generate evenly spaced points along this segment
                    segment_points = generate_points_along_segment(
                        points1[seg_idx], 
                        points1[seg_idx + 1],
                        spacing=spiral_params["spacing"]
                    )
                    
                    # At each point: move there + do HRIF_MoveS spiral continuously
                    for pt in segment_points:
                        simple_move(pt)
                        waitForBlending(cps=cps, config=config)

                        # Do HRIF_MoveS spiral at this point
                        cps.HRIF_MoveS(
                            0, 0,
                            spiral_params["dSpiralIncrement"],
                            spiral_params["dSpiralDiameter"],
                            spiral_params["dVelocity"],
                            spiral_params["dAcc"],
                            spiral_params["dRadius"],
                            tcp_name,
                            ucs_name,
                            f"spiral{spiral_count}"
                        )
                        waitForBlending(cps=cps, config=config)
                        spiral_count += 1   
                    
            else:
                # Normal linear motion (original behavior)
                for point in points1:
                    communicate(
                        cps=cps,
                        config=config,
                        point=point,
                        tcp=config['coords']['tcptool1plane1'],
                        ucs=config['coords']['ucsTable1'],
                        seventh=-1,
                        speed=0.6,
                        wait=False
                    )
                    waitForBlending(cps=cps, config=config)
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
           # turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        for i in range(1, 3):  # Loop from 1 to 6 (for p1-p6)
        # Get the current tcx (0-indexed)
            current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
            
            # Get the current prepoint and zigzag path (1-indexed)
            current_prepoint = eval(f"prepointp{i}")
            current_zigzag = eval(f"zigzag_pathp{i}")

            # Original sequence with dynamic variables
            communicate(
                cps=cps, config=config, 
                seventh=current_tcx, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=0.7, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            perform_process_top(cps, config, points1=current_zigzag,force=force, enable_spiral=True)  # Dynamic zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
    
    ylen_data = get_y_values(1, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor1zizagbig(force,z,cps)
        else:
            smalldoor1zizagsmall(force,z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor2zizag(force,z,cps):
    
    def smalldoor2zizagsmall(force,z,cps):
        
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
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(2)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 50  # Step size in X (instead of Y)
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
                    (x_coords[1])/1 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - (innerOffset*0.5),
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/1 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
                ]
                modified_Point4 = [
                    x_coords[3] - tool3x - innerOffset,
                    y_coords[3] + tool3y + innerOffset,
                ]
                print("modified_Point1:", modified_Point1)
                print("modified_Point2:", modified_Point2)
                print("modified_Point3:", modified_Point3)
                print("modified_Point4:", modified_Point4)

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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=10,innerOffsetX=10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor2zizagbig(force,z,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(2, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(2, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(2, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(2, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(2)
        print("tcx0:", tcx0)
        tcx1=tcx0+((p6[0]-p7[0])/2)
        print("tcx1:", tcx1)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 80  # Step size in X (instead of Y)
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
                    (x_coords[1])/2 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - innerOffset,
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/2 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        for i in range(1, 3):  # Loop from 1 to 6 (for p1-p6)
        # Get the current tcx (0-indexed)
            current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
            
            # Get the current prepoint and zigzag path (1-indexed)
            current_prepoint = eval(f"prepointp{i}")
            current_zigzag = eval(f"zigzag_pathp{i}")

            # Original sequence with dynamic variables
            communicate(
                cps=cps, config=config, 
                seventh=current_tcx, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=0.7, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
    
    ylen_data = get_y_values(2, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor2zizagbig(force,z,cps)
        else:
            smalldoor2zizagsmall(force,z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

    
def smalldoor3zizag(force,z,cps):
    
    def smalldoor3zizagsmall(force,z,cps):
        
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
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(3)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 50  # Step size in X (instead of Y)
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
                    (x_coords[1])/1 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - (innerOffset*0.5),
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/1 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
                ]
                modified_Point4 = [
                    x_coords[3] - tool3x - innerOffset,
                    y_coords[3] + tool3y + innerOffset,
                ]
                print("modified_Point1:", modified_Point1)
                print("modified_Point2:", modified_Point2)
                print("modified_Point3:", modified_Point3)
                print("modified_Point4:", modified_Point4)

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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=10,innerOffsetX=10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor3zizagbig(force,z,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(3, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(3, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(3, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(3, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(3)
        print("tcx0:", tcx0)
        tcx1=tcx0+((p6[0]-p7[0])/2)
        print("tcx1:", tcx1)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 80  # Step size in X (instead of Y)
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
                    (x_coords[1])/2 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - innerOffset,
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/2 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        for i in range(1, 3):  # Loop from 1 to 6 (for p1-p6)
        # Get the current tcx (0-indexed)
            current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
            
            # Get the current prepoint and zigzag path (1-indexed)
            current_prepoint = eval(f"prepointp{i}")
            current_zigzag = eval(f"zigzag_pathp{i}")

            # Original sequence with dynamic variables
            communicate(
                cps=cps, config=config, 
                seventh=current_tcx, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=0.7, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
    
    ylen_data = get_y_values(3, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor3zizagbig(force,z,cps)
        else:
            smalldoor3zizagsmall(force,z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")

def smalldoor4zizag(force,z,cps):
    
    def smalldoor4zizagsmall(force,z,cps):
        
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
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)
        #7th axis postion
        x1=p8[0]+get_door_position(4)
        print("x1:", x1)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 30  # Step size in X (instead of Y)
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
                    (x_coords[1])/1 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - (innerOffset*0.5),
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/1 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
                ]
                modified_Point4 = [
                    x_coords[3] - tool3x - innerOffset,
                    y_coords[3] + tool3y + innerOffset,
                ]
                print("modified_Point1:", modified_Point1)
                print("modified_Point2:", modified_Point2)
                print("modified_Point3:", modified_Point3)
                print("modified_Point4:", modified_Point4)

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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint


        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=10,innerOffsetX=10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)


        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=0.2, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
        # # turn_vibration_on(cps)
        perform_process_top(cps, config, points1=zigzag_pathp1,force=force)  # Dynamic zigzag path
        # # turn_vibration_off(cps)
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=0.2, wait=True
        )
    
    def smalldoor4zizagbig(force,z,cps):


        # Load configuration
        config = load_config()
        config['logger'] = setup_logger(config['settings']['debug'])

        # Connect to robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)
        
        #Main Points
        p8 = get_inner_corner_point(4, 0)
        print("p8:", p8)
        p7 = get_inner_corner_point(4, 1)
        print("p7:", p7)
        p6= get_inner_corner_point(4, 2)
        print("p6:", p6)
        p5 = get_inner_corner_point(4, 3)
        print("p5:", p5)

        #7th axis position
        tcx0=p8[0]+get_door_position(4)
        print("tcx0:", tcx0)
        tcx1=tcx0+((p6[0]-p7[0])/2)
        print("tcx1:", tcx1)

        #prehoming
        prehoming=[0,200,50,0,0,0]
        #Depth of the poocket z
        #z=-6.5
        #zigzag points
        distance=p6[0]-p8[0]
        print("distance:", distance)
        #Points calculation
        point5=[-p5[0],p5[1],z,-0.034,0.556,0.251]
        print("point5:", point5)
        point6=[-p6[0],p6[1],z,-0.034,0.556,0.251]
        print("point6:", point6)
        point7=[-p7[0],p7[1],z,-0.034,0.556,0.251]
        print("point7:", point7)
        point8=[-p8[0],p8[1],z,-0.034,0.556,0.251]
        print("point8:", point8)

        #Final Points
        point5u=[-distance,point5[1],point5[2],point5[3],point5[4],point5[5]]
        print("point5u:", point5u)
        point6u=[-distance,point6[1],point6[2],point6[3],point6[4],point6[5]]
        print("point6u:", point6u)
        point7u=[0,point7[1],point7[2],point7[3],point7[4],point7[5]]
        print("point7u:", point7u)
        point8u=[0,point8[1],point8[2],point8[3],point8[4],point8[5]]
        print("point8u:", point8u)

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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX):
            prepoint = None
            zigzag_coords = []
            
            # Parameters (adjust as needed)
            tool3y = 50.8   # Tool offset in Y
            tool3x = 38.1   # Tool offset in X
            innerSandingOffset = 80  # Step size in X (instead of Y)
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
                    (x_coords[1])/2 + tool3x + innerOffsetX,
                    y_coords[1] - tool3y - innerOffset,
                ]
                modified_Point3 = [
                    x_coords[2] - tool3x - innerOffset,
                    y_coords[2] - tool3y - innerOffset,
                ]
                modified_Point1 = [
                    (x_coords[0])/2 + tool3x + innerOffsetX,
                    y_coords[0] + tool3y + innerOffset,
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
                            [modified_Point1[0] + offset, modified_Point1[1], z_zigzag, 0, 0, 0],
                            [modified_Point2[0] + offset, modified_Point2[1], z_zigzag, 0, 0, 0],
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
            
                prepoint = [abs(modified_Point1[0])+0.5, modified_Point1[1], z_zigzag, 0, 0, 0]  
            return zigzag_coords,prepoint

        #Second Pocket 1st Cycle
        zigzag_pathp1,prepointp1= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=-10)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        zigzag_pathp2,prepointp2= generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=5,innerOffsetX=4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            # Vibration on
            
            
            # Force Control Activated
            putForceZminus(
                cps=cps,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                config=config
            )
            turn_vibration_on(cps)
            
            # Communicate to each point in points1
            for point in points1:
                communicate(
                    cps=cps,
                    config=config,
                    point=point,
                    tcp=config['coords']['tcptool1plane1'],
                    ucs=config['coords']['ucsTable1'],
                    seventh=-1,
                    speed=0.6,
                    wait=False
                )
            
            # Wait for blending and turn off vibration
            waitForBlending(cps=cps, config=config)
            turn_vibration_off(cps)
            #Release force
            releaseForce(cps=cps, config=config)
        
        for i in range(1, 3):  # Loop from 1 to 6 (for p1-p6)
        # Get the current tcx (0-indexed)
            current_tcx = eval(f"tcx{i-1}")  # Maps tcx0 for i=1, tcx1 for i=2, etc.
            
            # Get the current prepoint and zigzag path (1-indexed)
            current_prepoint = eval(f"prepointp{i}")
            current_zigzag = eval(f"zigzag_pathp{i}")

            # Original sequence with dynamic variables
            communicate(
                cps=cps, config=config, 
                seventh=current_tcx, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=0.7, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
            # # turn_vibration_on(cps)
            perform_process_top(cps, config, points1=current_zigzag,force=force)  # Dynamic zigzag path
            # # turn_vibration_off(cps)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=0.7, wait=True
            )
    
    ylen_data = get_y_values(4, default_on_error=True)
    ylen = ylen_data['ylen']

    print("ylen:", ylen)

    # Check conditions
    if ylen == "null":
        print("No door data available - skipping operations")
    elif isinstance(ylen, (int, float)):  # Ensure it's numeric
        if ylen > 600:
            smalldoor4zizagbig(force,z,cps)
        else:
            smalldoor4zizagsmall(force,z,cps)
    else:
        print(f"Invalid ylen value type: {type(ylen)} - expected number or 'null'")



def generate_points_along_segment(start_point, end_point, spacing=15.0):
    """Generate evenly spaced points along a line segment."""
    import math
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    dz = end_point[2] - start_point[2]
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
            
    if length < spacing:
        return [start_point, end_point]
            
    num_points = max(int(length / spacing), 2)
    points = []
    for i in range(num_points + 1):
        t = i / num_points
        points.append([
            start_point[0] + t * dx,
            start_point[1] + t * dy,
            start_point[2] + t * dz,
            start_point[3], start_point[4], start_point[5],
        ])
    return points


if __name__ == "__main__":
    smalldoor1zizag(force=5,z=-6.5)
    smalldoor2zizag(force=5,z=-6.5)
    smalldoor3zizag(force=5,z=-6.5)
    #smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag