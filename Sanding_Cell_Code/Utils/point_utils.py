import sqlite3

def get_points_coordinates_as_arrays(db_path, point_labels):
    """
    Fetch coordinates for any set of points as arrays.

    Args:
        db_path (str): Path to the SQLite database.
        point_labels (list): List of point labels to query (e.g., ['Point17', 'Point18']).

    Returns:
        dict: A dictionary mapping point labels to their coordinates (as arrays) or error messages.
    """
    results = {}
    connection = None
    try:
        # Connect to the SQLite database
        connection = sqlite3.connect(db_path)
        cursor = connection.cursor()

        # Query each point label
        for point_label in point_labels:
            query = "SELECT x, y, z FROM top_corners WHERE point_label = ?"
            cursor.execute(query, (point_label,))
            point = cursor.fetchone()

            if point:
                results[point_label] = [point[0], point[1], point[2]]
            else:
                results[point_label] = {"error": f"{point_label} not found in the database."}

    except Exception as e:
        results["error"] = str(e)

    finally:
        # Close the connection
        if connection is not None:
            connection.close()

    return results

def display_points_as_arrays(db_path, point_labels):
    """
    Display coordinates of points as arrays.

    Args:
        db_path (str): Path to the SQLite database.
        point_labels (list): List of point labels to query (e.g., ['Point17', 'Point18']).
    """
    results = get_points_coordinates_as_arrays(db_path, point_labels)

    for point_label, data in results.items():
        if isinstance(data, list):
            print(f"{point_label}: {data}")
        else:
            print(f"{point_label}: {data['error']}")
