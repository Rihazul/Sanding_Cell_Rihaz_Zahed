import cadquery as cq

def load_and_measure_pockets(file_path):
    """
    Load a STEP file, measure the overall bounding box, and extract 4 corner points for each pocket.

    Args:
        file_path (str): Path to the STEP file.

    Returns:
        dict: A dictionary containing measurements and pocket points.
    """
    try:
        # Load the STEP file
        model = cq.importers.importStep(file_path)
        print("STEP file imported successfully!")
        
        # Get solids (assuming multiple solids for pockets)
        solids = model.val().Solids()

        # Prepare measurements for all pockets
        pockets_data = []

        for idx, solid in enumerate(solids, start=1):
            # Extract bounding box for the pocket
            bounding_box = solid.BoundingBox()
            
            # Extract 4 corner points for the top of the pocket
            x_min, x_max = bounding_box.xmin, bounding_box.xmax
            y_min, y_max = bounding_box.ymin, bounding_box.ymax
            z_max = bounding_box.zmax

            top_corners = [
                (x_min, y_min, z_max),  # Bottom Left Front
                (x_min, y_max, z_max),  # Bottom Right Front
                (x_max, y_max, z_max),  # Top Right Front
                (x_max, y_min, z_max)   # Top Left Front
            ]
            
            pocket_data = {
                "Pocket Index": idx,
                "Bounding Box": {
                    "X Min": x_min,
                    "Y Min": y_min,
                    "Z Min": bounding_box.zmin,
                    "X Max": x_max,
                    "Y Max": y_max,
                    "Z Max": z_max,
                    "Width (X)": x_max - x_min,
                    "Depth (Y)": y_max - y_min,
                    "Height (Z)": z_max - bounding_box.zmin,
                },
                "Top Front Corners": top_corners,
                "Volume": solid.Volume(),
                "Surface Area": solid.Area(),
            }
            
            pockets_data.append(pocket_data)

        return pockets_data
    
    except Exception as e:
        print(f"Error loading or measuring STEP file: {e}")
        return None

# Path to your STEP file
file_path = "D:/Job/models/Final_31012025/P02.stp"  # Replace with your STEP file path

# Load and measure the STEP file
pocket_measurements = load_and_measure_pockets(file_path)

# Print pocket measurements
if pocket_measurements:
    print("\nPocket Measurements:")
    for pocket in pocket_measurements:
        print(f"Pocket {pocket['Pocket Index']}:")
        for key, value in pocket.items():
            if isinstance(value, dict):  # For nested dictionaries like Bounding Box
                print(f"  {key}:")
                for sub_key, sub_value in value.items():
                    print(f"    {sub_key}: {sub_value}")
            elif isinstance(value, list):  # For lists like corner points
                print(f"  {key}:")
                for point in value:
                    print(f"    {point}")
            else:
                print(f"  {key}: {value}")
