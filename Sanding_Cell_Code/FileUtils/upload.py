import cadquery as cq
import sqlite3

def table_exists(cursor, table_name):
    """
    Check if a table exists in the SQLite database.
    """
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name=?", (table_name,))
    return cursor.fetchone() is not None

def load_and_measure_pockets(file_path):
    """
    Load a STEP file, extract measurements and pocket data.
    """
    try:
        model = cq.importers.importStep(file_path)
        print("STEP file imported successfully!")
        
        solids = model.val().Solids()
        pockets_data = []

        for idx, solid in enumerate(solids, start=1):
            bounding_box = solid.BoundingBox()
            x_min, x_max = bounding_box.xmin, bounding_box.xmax
            y_min, y_max = bounding_box.ymin, bounding_box.ymax
            z_max = bounding_box.zmax

            top_corners = [
                (x_min, y_min, z_max),
                (x_min, y_max, z_max),
                (x_max, y_max, z_max),
                (x_max, y_min, z_max)
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
            }
            pockets_data.append(pocket_data)
        return pockets_data

    except Exception as e:
        print(f"Error loading or measuring STEP file: {e}")
        return None

def save_to_database(connection, cursor, pockets_data):
    """
    Save the extracted pockets data into the database only if the tables exist.
    """
    try:
        # Check and delete data only if the tables exist
        if table_exists(cursor, "pocket_data"):
            cursor.execute("DELETE FROM pocket_data")
        if table_exists(cursor, "top_corners"):
            cursor.execute("DELETE FROM top_corners")
        
        connection.commit()

        point_counter = 1  # Start labeling points from Point1

        for pocket in pockets_data:
            bounding_box = pocket["Bounding Box"]

            if table_exists(cursor, "pocket_data"):
                cursor.execute("""
                    INSERT INTO pocket_data (
                        pocket_index, x_min, y_min, z_min, x_max, y_max, z_max,
                        width, depth, height
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    pocket["Pocket Index"],
                    bounding_box["X Min"], bounding_box["Y Min"], bounding_box["Z Min"],
                    bounding_box["X Max"], bounding_box["Y Max"], bounding_box["Z Max"],
                    bounding_box["Width (X)"], bounding_box["Depth (Y)"], bounding_box["Height (Z)"]
                ))

            if table_exists(cursor, "top_corners"):
                for corner in pocket["Top Front Corners"]:
                    point_label = f"Point{point_counter}"
                    cursor.execute("""
                        INSERT INTO top_corners (pocket_index, point_label, x, y, z)
                        VALUES (?, ?, ?, ?, ?)
                    """, (pocket["Pocket Index"], point_label, corner[0], corner[1], corner[2]))
                    point_counter += 1

        connection.commit()
        print("Data saved to database successfully!")

    except Exception as e:
        print(f"Error saving to database: {e}")
        connection.rollback()

# File path to STEP file
# file_path ="../Final_31012025/P01.stp"



def upload3DModel(file_path):
    # Connect to SQLite database
    connection = sqlite3.connect("./databases/pocket_measurements01.db")
    cursor = connection.cursor()

    # Ensure necessary tables exist
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS pocket_data (
        pocket_index INTEGER PRIMARY KEY,
        x_min REAL,
        y_min REAL,
        z_min REAL,
        x_max REAL,
        y_max REAL,
        z_max REAL,
        width REAL,
        depth REAL,
        height REAL
    )
    """)

    cursor.execute("""
    CREATE TABLE IF NOT EXISTS top_corners (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        pocket_index INTEGER,
        point_label TEXT,
        x REAL,
        y REAL,
        z REAL,
        FOREIGN KEY (pocket_index) REFERENCES pocket_data (pocket_index)
    )
    """)

    connection.commit()

    # Load the STEP file and extract pocket data
    pocket_measurements = load_and_measure_pockets(file_path)

    # Save data to the database
    if pocket_measurements:
        save_to_database(connection, cursor, pocket_measurements)

    # Close the database connection
    connection.close()
