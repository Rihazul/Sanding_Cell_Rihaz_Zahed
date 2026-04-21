# testcommu.py

import sys
import math
import os
import time
import json
# Add parent directory to path so Python can find the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import threading
from Server_Better_V2 import communicate,setup_logger,waitForBlending,turn_vibration_on,turn_vibration_off,putForce,releaseForce,putForceZplus,putForceZminus
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


def _generate_zigzag_path_shared(x_coords, y_coords, z_coords, innerOffset, innerOffsetX, orientation="vertical", movement="zigzag", innerSandingOffset=50, edge_coverage=False, x_length_divisor=1.0):
    prepoint = None
    zigzag_coords = []
    
    # Parameters (adjust as needed)
    tool3y = 50.8   # Tool offset in Y
    tool3x = 38.1   # Tool offset in X
    innerSandingOffset = float(innerSandingOffset)  # Step size in X (instead of Y)
    xframe_1 = 0
    xframe_2 = 0

    # 1) Collect boundary coordinates as [x, y, z]
    boundary_coords = []
    for i in range(len(x_coords)):
        boundary_coords.append([x_coords[i], y_coords[i], z_coords[i]])

    try:
        x_length_divisor = float(x_length_divisor)
        if x_length_divisor == 0:
            x_length_divisor = 1.0
    except (TypeError, ValueError):
        x_length_divisor = 1.0

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
            (x_coords[1]) / x_length_divisor + tool3x + innerOffsetX,
            y_coords[1] - tool3y - (innerOffset),
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            (x_coords[0]) / x_length_divisor + tool3x + innerOffsetX,
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

        # Bounding box for horizontal orientation
        x_min = min(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        x_max = max(modified_Point1[0], modified_Point2[0], modified_Point3[0], modified_Point4[0])
        y_min = min(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])
        y_max = max(modified_Point1[1], modified_Point2[1], modified_Point3[1], modified_Point4[1])

        orientation_mode = (orientation or "vertical").lower()

        # Edge coverage path around the boundary (single pass)
        edge_coverage_coords = []
        edge_prepoint = None
        edge_offset = 1.75
        if edge_coverage:
            edge_Point1 = [
                x_coords[0] + tool3x + edge_offset,
                y_coords[0] + tool3y + edge_offset,
            ]
            edge_Point2 = [
                x_coords[1] + tool3x + edge_offset,
                y_coords[1] - tool3y - edge_offset,
            ]
            edge_Point3 = [
                x_coords[2] - tool3x - edge_offset,
                y_coords[2] - tool3y - edge_offset,
            ]
            edge_Point4 = [
                x_coords[3] - tool3x - edge_offset,
                y_coords[3] + tool3y + edge_offset,
            ]

            if orientation_mode == "horizontal":
                edge_coverage_coords = [
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                    [edge_Point3[0], edge_Point3[1], z_zigzag, 0, 0, 0],
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                    [edge_Point1[0], edge_Point1[1], z_zigzag, 0, 0, 0],
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                ]
            else:
                edge_coverage_coords = [
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                    [edge_Point1[0], edge_Point1[1], z_zigzag, 0, 0, 0],
                    [edge_Point2[0], edge_Point2[1], z_zigzag, 0, 0, 0],
                    [edge_Point3[0], edge_Point3[1], z_zigzag, 0, 0, 0],
                    [edge_Point4[0], edge_Point4[1], z_zigzag, 0, 0, 0],
                ]

        if orientation_mode == "horizontal":
            yinner = abs(y_max - y_min)
            if yinner > 0:
                num_steps = math.floor(yinner / innerSandingOffset)
                if num_steps == 0:
                    num_steps = 1
                adjusted_step = yinner / num_steps

                offset = 0.0
                toggle = 0

                # Build zigzag rows from top (max y) to bottom (min y)
                while offset <= yinner + 1e-9:  # small floating-point tolerance
                    current_y = y_max - offset
                    row_points = [
                        [x_min, current_y, z_zigzag, 0, 0, 0],
                        [x_max, current_y, z_zigzag, 0, 0, 0],
                    ]
                    # Reverse every other row to create a zigzag
                    if toggle:
                        row_points.reverse()

                    zigzag_coords.extend(row_points)
                    offset += adjusted_step
                    toggle = 1 - toggle
            else:
                # Fallback: single row when height is too small
                zigzag_coords.extend(
                    [
                        [x_min, y_max, z_zigzag, 0, 0, 0],
                        [x_max, y_max, z_zigzag, 0, 0, 0],
                    ]
                )
        else:
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

        # Update edge coverage coords to absolute values
        for point in edge_coverage_coords:
            point[1] = abs(point[1])
            point[0] = abs(point[0])

        if edge_coverage_coords:
            edge_prepoint = [
                abs(edge_coverage_coords[0][0]) + 0.5,
                edge_coverage_coords[0][1],
                z_zigzag,
                0,
                0,
                0,
            ]

        if orientation_mode == "horizontal":
            prepoint = [abs(x_min) + 0.5, y_max, z_zigzag, 0, 0, 0]
        else:
            prepoint = [abs(modified_Point1[0]) + 0.5, modified_Point1[1], z_zigzag, 0, 0, 0]
    return edge_coverage_coords, zigzag_coords, prepoint, edge_prepoint


def _perform_process_top_shared(cps, config, points1, force, tcp, ucs, speed=0.6):
    # Vibration on
    
    
    # Force Control Activated
    putForceZminus(
        cps=cps,
        force=force,
        tcp=tcp,
        ucs=ucs,
        config=config
    )
    turn_vibration_on(cps)
    
    # Communicate to each point in points1
    for point in points1:
        communicate(
            cps=cps,
            config=config,
            point=point,
            tcp=tcp,
            ucs=ucs,
            seventh=-1,
            speed=speed,
            wait=False
        )
    
    # Wait for blending and turn off vibration
    waitForBlending(cps=cps, config=config)
    turn_vibration_off(cps)
    #Release force
    releaseForce(cps=cps, config=config)



def smalldoor1zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    
    def smalldoor1zizagsmall(force,z,cps):
        
        # Load configuration from YAML
        config = load_config()

        #Set up logger
        config['logger'] = setup_logger(config['settings']['debug'])

        #Establish connection with robot
        # cps = CPSClient()
        # IP = config['server']['cpip']
        # port = config['server']['cps']
        # ret = cps.HRIF_Connect(0, IP, port)


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(1, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(1, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(1, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(1, 3)
        print("p50:", p50)


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        

        # Processing Points
        frame= 57
        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
        print("p5:", p5)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #7th axis postion
        x1=p8[0]+get_door_position(1)
        print("x1:", x1)
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1,innerOffset=17,innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=speeed, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)

        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
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
            speed=speeed, wait=True
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
        # Using cps from caller; connection handled upstream.

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(1, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(1, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(1, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(1, 3)
        print("p50:",p50)

        # Processing Points
        frame= 67
        

        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
                x_length_divisor=2.0,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        _, zigzag_pathp2, prepointp2, _ = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Edge coverage (single pass)
        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                seventh=tcx0, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
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
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
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
                speed=speeed, wait=True
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

def smalldoor2zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    
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


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(2, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(2, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(2, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(2, 3)
        print("p50:", p50)


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        

        # Processing Points
        frame= 67
        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
        print("p5:", p5)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #7th axis postion
        x1=p8[0]+get_door_position(2)
        print("x1:", x1)
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1,innerOffset=17,innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=speeed, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)

        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
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
            speed=speeed, wait=True
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
        # Using cps from caller; connection handled upstream.

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(2, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(2, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(2, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(2, 3)
        print("p50:",p50)

        # Processing Points
        frame= 67
        

        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
                x_length_divisor=2.0,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        _, zigzag_pathp2, prepointp2, _ = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Edge coverage (single pass)
        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                seventh=tcx0, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
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
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
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
                speed=speeed, wait=True
            )
    
    ylen_data = get_y_values(1, default_on_error=True)
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

def smalldoor3zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    
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


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(3, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(3, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(3, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(3, 3)
        print("p50:", p50)


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        

        # Processing Points
        frame= 67
        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
        print("p5:", p5)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #7th axis postion
        x1=p8[0]+get_door_position(3)
        print("x1:", x1)
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1,innerOffset=17,innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=speeed, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)

        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
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
            speed=speeed, wait=True
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
        # Using cps from caller; connection handled upstream.

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(3, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(3, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(3, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(3, 3)
        print("p50:",p50)

        # Processing Points
        frame= 67
        

        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
                x_length_divisor=2.0,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        _, zigzag_pathp2, prepointp2, _ = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Edge coverage (single pass)
        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                seventh=tcx0, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
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
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
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
                speed=speeed, wait=True
            )
    
    ylen_data = get_y_values(1, default_on_error=True)
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

def smalldoor4zizag(
    force, z, cps, orientation="vertical", movement="zigzag", spiral_settings=None
):
    
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


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(4, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(4, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(4, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(4, 3)
        print("p50:", p50)


        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        

        # Processing Points
        frame= 67
        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
        print("p5:", p5)
        #prehoming
        prehoming=[0,200,50,0,0,0]
        # #Depth of the poocket z
        # z=-6.5
        #7th axis postion
        x1=p8[0]+get_door_position(4)
        print("x1:", x1)
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1,innerOffset=17,innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)

        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Original sequence with dynamic variables
        communicate(
            cps=cps, config=config, 
            seventh=x1, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            speed=speeed, wait=True
        )
        communicate(
            cps=cps, config=config, 
            point=prehoming, 
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
        )
        # # turn_vibration_on(cps)

        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
        communicate(
            cps=cps, config=config, 
            point=prepointp1,  # Dynamic prepoint
            tcp=config['coords']['tcptool1plane1'], 
            ucs=config['coords']['ucsTable1'], 
            seventh=-1, 
            speed=speeed, wait=True
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
            speed=speeed, wait=True
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
        # Using cps from caller; connection handled upstream.

        json_config = load_json_config()
        speeed = float(json_config['robotSpeed'])
        
        #Main Points
        p80 = get_outer_corner_point(4, 0)
        print("p80:", p80)
        p70 = get_outer_corner_point(4, 1)
        print("p70:", p70)
        p60= get_outer_corner_point(4, 2)
        print("p60:", p60)
        p50 = get_outer_corner_point(4, 3)
        print("p50:",p50)

        # Processing Points
        frame= 67
        

        p8 = [p80[0] + frame, p80[1] + frame, p80[2], p80[3], p80[4], 0]
        print("p8:", p8)

        p7 = [p70[0] + frame, p70[1] - frame, p70[2], p70[3], p70[4], 0]
        print("p7:", p7)

        p6 = [p60[0] - frame, p60[1] - frame, p60[2], p60[3], p60[4], 0]
        print("p6:", p6)

        p5 = [p50[0] - frame, p50[1] + frame, p50[2], p50[3], p50[4], 0]
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

        def generate_zigzag_path(x_coords, y_coords, z_coords, innerOffset,innerOffsetX, edge_coverage=False):
            return _generate_zigzag_path_shared(
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                innerOffset=innerOffset,
                innerOffsetX=innerOffsetX,
                orientation=orientation,
                movement=movement,
                innerSandingOffset=67.5,
                edge_coverage=edge_coverage,
                x_length_divisor=2.0,
            )

        #Second Pocket 1st Cycle
        edge_pathp1, zigzag_pathp1, prepointp1, edge_prepointp1 = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4, edge_coverage=True)
        print("zigzag_pathp=",zigzag_pathp1)
        print("prepointp:", prepointp1)
        #Second Pocket 2nd Cycle
        _, zigzag_pathp2, prepointp2, _ = generate_zigzag_path(x_coords=x_coords1, y_coords=y_coords1, z_coords=z_coords1, innerOffset=17, innerOffsetX=29.4)
        print("zigzag_pathp2=",zigzag_pathp2)
        print("prepointp2:", prepointp2)
        
        def perform_process_top(cps, config, points1,force):
            _perform_process_top_shared(
                cps=cps,
                config=config,
                points1=points1,
                force=force,
                tcp=config['coords']['tcptool1plane1'],
                ucs=config['coords']['ucsTable1'],
                speed=0.6,
            )

        # Edge coverage (single pass)
        if edge_pathp1:
            communicate(
                cps=cps, config=config, 
                seventh=tcx0, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=edge_prepointp1,  # Edge prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            perform_process_top(cps, config, points1=edge_pathp1,force=force)
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
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
                speed=speeed, wait=True
            )
            communicate(
                cps=cps, config=config, 
                point=prehoming, 
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
            )
            # # turn_vibration_on(cps)
            communicate(
                cps=cps, config=config, 
                point=current_prepoint,  # Dynamic prepoint
                tcp=config['coords']['tcptool1plane1'], 
                ucs=config['coords']['ucsTable1'], 
                seventh=-1, 
                speed=speeed, wait=True
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
                speed=speeed, wait=True
            )
    
    ylen_data = get_y_values(1, default_on_error=True)
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


if __name__ == "__main__":
    smalldoor1zizag(force=5,z=-6.5)
    # smalldoor2zizag(force=5,z=-6.5)
    # smalldoor3zizag(force=5,z=-6.5)
    #smalldoor4zizag(force=5,z=-6.5)# Uncommented to call smalldoor4zizag
