import math
import matplotlib.pyplot as plt

spoint=[150,513,-30,180,0,0]
#point13 = [-381.0, 196.84999999959996, 7.5, 180, 0, 0]
#point14 = [-381.0, 857.250000001,  7.5, 180, 0, 0]
#point15 = [-57.149999999999864, 857.250000001,  7.5, 180, 0, 0]
#point16 = [-57.149999999999864, 196.84999999959996,  7.5, 180, 0, 0]
#point13=[-412.968,252.297,7,180,0,0]
#point14=[-412.968,546.604,7,180,0,0]
#point15=[-96.614,546.601,7,180,0,0]
#point16=[-96.614,252.297,7,180,0,0]

p9= [704.8500000032, 57.1500000002, -9.525000000039999, 180, 0, 0]
p10= [704.8500000032, 323.8500000012, -9.525000000039999, 180, 0, 0]
p11= [57.1500000002, 323.8500000012, -9.525000000039999, 180, 0, 0]
p12= [57.1500000002, 57.1500000002, -9.525000000039999, 180, 0, 0]

point13=[-p9[0],p9[1],7,p9[3],p9[4],p9[5]]
point14=[-p10[0],p10[1],7,p10[3],p10[4],p10[5]]
point15=[-p11[0],p11[1],7,p11[3],p11[4],p11[5]]
point16=[-p12[0],p12[1],7,p12[3],p12[4],p12[5]]
print("point13=",point13)
print("point14=",point14)
print("point15=",point15)
print("point16=",point16)


#Sample Pocket4
dispocket4= point13[0]-point15[0]
cx=abs(point15[0])
print("cx=",cx)
cdistance=abs(point14[0])-abs(point15[0])
print("cdistance=",cdistance)
thirdcdistance=cdistance/3
print("thirdcdistance=",thirdcdistance)
cx1=cx+thirdcdistance
print("cx1=",cx1)
cx2=cx1+thirdcdistance
print("cx2=",cx2)
cx3=cx2+thirdcdistance
print("cx3=",cx3)

point15u= [0,point15[1],point15[2],point15[3],point15[4],point15[5]]
point16u= [0,point16[1],point16[2],point16[3],point16[4],point16[5]]
point13u= [dispocket4,point13[1],point13[2],point13[3],point13[4],point13[5]]
point14u= [dispocket4,point14[1],point14[2],point14[3],point14[4],point14[5]]
#pointpre=[-dispocket4-0.5,point14[1]-0.5,point14[2],point14[3],point14[4],point14[5]]


# Parameters (adjust as needed)
tool3y = 50.8   # Tool offset in Y
tool3x = 38.1   # Tool offset in X
innerOffset = 5  # Inner boundary offset
innerSandingOffset = 50  # Step size in X (instead of Y)
xframe_1 = 0
xframe_2 = 0

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
        (x_coords[1])/3 + tool3x + innerOffset,
        y_coords[1] - tool3y - innerOffset,
    ]
    modified_Point3 = [
        x_coords[2] - tool3x - innerOffset,
        y_coords[2] - tool3y - innerOffset,
    ]
    modified_Point1 = [
        (x_coords[0])/3 + tool3x + innerOffset,
        y_coords[0] + tool3y + innerOffset*2,
    ]
    modified_Point4 = [
        x_coords[3] - tool3x - innerOffset,
        y_coords[3] + tool3y + innerOffset,
    ]

    # Calculate available horizontal dimension
    xlen1 = abs(modified_Point3[0] - modified_Point1[0])
    xinner = xlen1 - xframe_1 - xframe_2
    print("xinner=",xinner)

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

prepoint = [abs(modified_Point1[0]), modified_Point1[1], z_zigzag, 180, 0, 0]

print("Zigzag coordinates:", zigzag_coords)
print("modified_Point1:",prepoint)

# Plotting the zigzag path
x_vals = [point[0] for point in zigzag_coords]
y_vals = [point[1] for point in zigzag_coords]

plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, marker='o', linestyle='-', color='b')
plt.title('Zigzag Path from Left to Right')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()
