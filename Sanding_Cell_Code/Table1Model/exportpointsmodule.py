import sys
from pathlib import Path
import sqlite3

# Add the current directory to Python's module search path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

# Import point_utils after ensuring the path is correct
from Utils.point_utils import get_points_coordinates_as_arrays

# Path to the SQLite database (assuming it's in the same directory)
db_path = "./databases/pocket_measurements01.db"

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

# Process each point:
# 1) Make x coordinate positive
# 2) If in pocket_points, set z=0
# 3) Append [180, 0, 0] to each point
for key, pt in results.items():
    if isinstance(pt, list):
        pt[0] = abs(pt[0])  # Ensure x is positive
        if key in pocket_points:
            pt[2] = 0  # Set z=0 for pocket points
        pt.extend([180, 0, 0])  # Append additional values
    else:
        print(f"Skipping {key} because data is invalid: {pt}")

# Create a dictionary of all processed points for export
exported_points = {
    "p1": results.get("Point1"),
    "p2": results.get("Point2"),
    "p3": results.get("Point3"),
    "p4": results.get("Point4"),
    "p5": results.get("Point5"),
    "p6": results.get("Point6"),
    "p7": results.get("Point7"),
    "p8": results.get("Point8"),
    "p9": results.get("Point9"),
    "p10": results.get("Point10"),
    "p11": results.get("Point11"),
    "p12": results.get("Point12"),
    "p13": results.get("Point13"),
    "p14": results.get("Point14"),
    "p15": results.get("Point15"),
    "p16": results.get("Point16"),

}

# Print all points for verification
if __name__ == "__main__":
    for name, point in exported_points.items():
        print(f"{name}: {point}")

# Make the points available when imported
__all__ = ['exported_points']