# testcommu.py
import math
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


    # Load configuration
    config = load_config()
    config['logger'] = setup_logger(config['settings']['debug'])

    # Connect to robot
    cps = CPSClient()
    IP = config['server']['cpip']
    port = config['server']['cps']
    ret = cps.HRIF_Connect(0, IP, port)



    # Hard-coded points for Pocket4
    # Format: [x, y, z, rotX, rotY, rotZ]
    #Pocket4
    spoint=[150,700,-30,180,0,0]
    point13 = [-381.0, 196.84999999959996, 7.5, 180, 0, 0]
    point14 = [-381.0, 857.250000001,  7.5, 180, 0, 0]
    point15 = [-57.149999999999864, 857.250000001,  7.5, 180, 0, 0]
    point16 = [-57.149999999999864, 196.84999999959996,  7.5, 180, 0, 0]
    #Sample Pocket4
    dispocket4= point13[0]-point15[0]
    cx=abs(point15[0])
    point15u= [0,point15[1],point15[2],point15[3],point15[4],point15[5]]
    point16u= [0,point16[1],point16[2],point16[3],point16[4],point16[5]]
    point13u= [dispocket4,point13[1],point13[2],point13[3],point13[4],point13[5]]
    point14u= [dispocket4,point14[1],point14[2],point14[3],point14[4],point14[5]]

     #Pocket3
    point9= [-822.3249999999999, 196.8499999999999, -20, 180, 0, 0]
    point10= [-822.3249999999999, 857.250000001,  -20, 180, 0, 0]
    point11= [-498.4749999999999, 857.250000001,  -20, 180, 0, 0]
    point12= [-498.4749999999999, 196.8499999999999,  -20, 180, 0, 0]
    cx1=abs(point12[0])

    #Pocket2
    point5= [-1263.6499999999999, 196.8499999999999, -20, 180, 0, 0]
    point6= [-1263.649999999999, 857.250000001, -20, 180, 0, 0]
    point7= [-939.8, 857.250000001, -20, 180, 0, 0]
    point8= [-939.8, 196.8499999999999, -20, 180, 0, 0]
    cx2=abs(point8[0])


    

    # Parameters (adjust as needed)
    tool3y             = 50.8   # Tool offset in Y
    tool3x             = 38.1   # Tool offset in X
    innerOffset        = 5      # Inner boundary offset
    innerSandingOffset = 50     # Step size in Y
    yframe_1           = 0
    yframe_2           = 0

    # 1) Collect boundary coordinates as [x, y, z]
    x_coords = [point13u[0], point14u[0], point15u[0], point16u[0]]
    y_coords = [point13u[1], point14u[1], point15u[1], point16u[1]]
    z_coords = [point13u[2], point14u[2], point15u[2], point16u[2]]

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
            x_coords[1] + tool3x + innerOffset,
            y_coords[1] - tool3y - innerOffset,
        ]
        modified_Point3 = [
            x_coords[2] - tool3x - innerOffset,
            y_coords[2] - tool3y - innerOffset,
        ]
        modified_Point1 = [
            x_coords[0] + tool3x + innerOffset,
            y_coords[0] + tool3y + innerOffset,
        ]
        modified_Point4 = [
            x_coords[3] - tool3x - innerOffset,
            y_coords[3] + tool3y + innerOffset,
        ]

        # Calculate available vertical dimension
        ylen1 = abs(modified_Point2[1] - modified_Point1[1])
        yinner = ylen1 - yframe_1 - yframe_2

        if yinner > 0:
            # Determine how many "rows" in the zigzag
            num_steps = math.ceil(yinner / innerSandingOffset)
            adjusted_step = yinner / num_steps

            offset = 0.0
            toggle = 0

            # Build zigzag path from top to bottom
            while offset <= yinner + 1e-9:  # small floating-point tolerance
                row_points = [
                    [modified_Point2[0], modified_Point2[1] - offset, z_zigzag, 180, 0, 0],
                    [modified_Point3[0], modified_Point3[1] - offset, z_zigzag, 180, 0, 0],
                ]
                # Reverse every other row to create a zigzag
                if toggle:
                    row_points.reverse()

                zigzag_coords.extend(row_points)
                offset += adjusted_step
                toggle = 1 - toggle

    # Update only the x coordinate to its absolute value
    for point in zigzag_coords:
        point[0] = abs(point[0])

    print("Zigzag coordinates:", zigzag_coords)

    # ------------------------------------------------------
    # 3) Move the robot through the zigzag points
    # ------------------------------------------------------
    # Assuming 'cps' and 'config' are defined elsewhere in your code
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    turn_vibration_on(cps)
    communicate(cps=cps,config=config,seventh=cx,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in zigzag_coords:
        communicate(
            cps=cps,
            config=config,
            point=point,  # Pass one point at a time
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.3,
            wait=False  # Wait for the robot to reach the point before moving to the next
        )
    waitForBlending(cps=cps, config=config)

    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=cx1,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)

    for point in zigzag_coords:
        communicate(
            cps=cps,
            config=config,
            point=point,  # Pass one point at a time
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.3,
            wait=False  # Wait for the robot to reach the point before moving to the next
        )
    waitForBlending(cps=cps, config=config)

    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.5,wait=True)
    communicate(cps=cps,config=config,seventh=cx2,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],speed=0.3,wait=True)
    for point in zigzag_coords:
        communicate(
            cps=cps,
            config=config,
            point=point,  # Pass one point at a time
            tcp=config['coords']['tcpReal'],
            ucs=config['coords']['ucsTable2'],
            seventh=-1,
            speed=0.3,
            wait=False  # Wait for the robot to reach the point before moving to the next
        )
    waitForBlending(cps=cps, config=config)
    communicate(cps=cps,config=config,point=spoint,tcp=config['coords']['tcpReal'],ucs=config['coords']['ucsTable2'],seventh=-1,speed=0.3,wait=True)
    turn_vibration_off(cps)

    
if __name__ == "__main__":
    main()