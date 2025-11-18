from point_utils import get_points_coordinates_as_arrays

# Path to the SQLite database
db_path = "pocket_measurements01.db"

# Manually define the point labels
point_labels = [
    "Point1", "Point2", "Point3", "Point4", "Point5",
    "Point6", "Point7", "Point8", "Point9", "Point10",
    "Point11", "Point12", "Point13", "Point14", "Point15", "Point16"
]

# Fetch coordinates for all points
results = get_points_coordinates_as_arrays(db_path, point_labels)

# Define which points should have z=0
pocket_points = ["Point1", "Point2", "Point3", "Point4"]

# Update each point so that:
# 1) x coordinate is positive
# 2) if in pocket_points, set z=0
# 3) then append [180, 0, 0]
for key, pt in results.items():
    if isinstance(pt, list):
        pt[0] = abs(pt[0])
        if key in pocket_points:
            pt[2] = 0
        pt.extend([180, 0, 0])
    else:
        print(f"Skipping {key} because data is invalid: {pt}")

# Manually assign results to variables
p1 = results.get("Point1", None)
p2 = results.get("Point2", None)
p3 = results.get("Point3", None)
p4 = results.get("Point4", None)
p5 = results.get("Point5", None)
p6 = results.get("Point6", None)
p7 = results.get("Point7", None)
p8 = results.get("Point8", None)
p9 = results.get("Point9", None)
p10 = results.get("Point10", None)
p11 = results.get("Point11", None)
p12 = results.get("Point12", None)
p13 = results.get("Point13", None)
p14 = results.get("Point14", None)
p15 = results.get("Point15", None)
p16 = results.get("Point16", None)

# Print the (original) results
print("p1:", p1)
print("p2:", p2)
print("p3:", p3)
print("p4:", p4)
print("p5:", p5)
print("p6:", p6)
print("p7:", p7)
print("p8:", p8)
print("p9:", p9)
print("p10:", p10)
print("p11:", p11)
print("p12:", p12)
print("p13:", p13)
print("p14:", p14)
print("p15:", p15)
print("p16:", p16)


