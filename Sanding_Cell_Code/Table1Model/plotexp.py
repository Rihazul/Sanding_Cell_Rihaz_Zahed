import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math  # for dynamic step calculation

# Hypothetical function to fetch points from the DB
from Utils.point_utils import get_points_coordinates_as_arrays


def plot_data(inverseOverlapping):
    # Path to the SQLite database
    db_path = "./databases/pocket_measurements01.db"

    # Manually define the point labels
    point_labels = [
        "Point1", "Point2", "Point3", "Point4",
        "Point5", "Point6", "Point7", "Point8",
        "Point9", "Point10", "Point11", "Point12",
        "Point13", "Point14", "Point15", "Point16"
        
    ]

    print("the point lables are: ", point_labels)

    # Fetch coordinates for all points
    results = get_points_coordinates_as_arrays(db_path, point_labels)

    # -------------------------------------------------------------------
    # Print the coordinate value for each point
    print("Individual Point Coordinates:")
    for point_label in point_labels:
        point = results.get(point_label)
        if point is not None:
            print(f"{point_label}: {point}")
        else:
            print(f"{point_label}: not found in the database")

    # -------------------------------------------------------------------
    # Collect points for each pocket
    pockets = {
        "Pocket1": ["Point1", "Point2", "Point3", "Point4"],
        "Pocket2": ["Point5", "Point6", "Point7", "Point8"],
        "Pocket3": ["Point9", "Point10", "Point11", "Point12"],
        "Pocket4": ["Point13", "Point14", "Point15", "Point16"]
    }

    # Define tool offsets and parameters
    tool3y = 50.8   # Tool offset in Y
    tool3x = 38.1   # Tool offset in X
    innerOffset = 5  # Inner boundary offset
    innerSandingOffset = int(inverseOverlapping)  # Step size in X for the zigzag passes
    pocketZigSandCnt = 1      # Number of cycles (not used in this example)
    xframe_1 = 0              # Additional X offset if needed
    xframe_2 = 0
    yframe_1 = 0
    yframe_2 = 0

    # Prepare the plot
    # plt.figure(figsize=(12, 10))
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(bottom=0.2)

    def add_direction_arrows(ax, xs, ys, color, every=2, zorder=3):
        if len(xs) < 2 or len(ys) < 2:
            return
        for i in range(0, min(len(xs), len(ys)) - 1, every):
            ax.annotate(
                "",
                xy=(xs[i + 1], ys[i + 1]),
                xytext=(xs[i], ys[i]),
                arrowprops=dict(arrowstyle="->", color=color, lw=1),
                zorder=zorder,
            )

    def plot_corner_points(ax, x_coords, y_coords, label, color):
        ax.scatter(
            x_coords,
            y_coords,
            s=50,
            marker="s",
            facecolors="none",
            edgecolors=color,
            label=label,
            zorder=4,
        )

    # Loop through all pockets
    for pocket_name, point_names in pockets.items():
        x_coords = []
        y_coords = []
        z_coords = []
        
        missing_points = []

        # Gather data for this pocket
        for pt in point_names:
            point = results.get(pt)
            if isinstance(point, (list, tuple)) and len(point) >= 3:
                x_coords.append(point[0])
                y_coords.append(point[1])
                z_coords.append(point[2])
            else:
                missing_points.append(pt)

        if missing_points:
            print(f"\n{pocket_name}: missing/invalid points -> {missing_points}")
            # Keep flow alive when uploaded geometry has fewer pockets than expected.
            if len(x_coords) < 4:
                print(f"{pocket_name}: insufficient valid points ({len(x_coords)}/4), skipping.")
                continue
        
        # Plot the pocket boundary (if points are valid)
        if x_coords and y_coords:
            # Close the loop with first point
            boundary_line, = ax.plot(
                x_coords + [x_coords[0]],
                y_coords + [y_coords[0]],
                label=f"{pocket_name} Boundary",
                linestyle="--",
                marker="x",
            )
            boundary_color = boundary_line.get_color()
            plot_corner_points(
                ax,
                x_coords,
                y_coords,
                label=f"{pocket_name} Corners",
                color=boundary_color,
            )
            if pocket_name == "Pocket1":
                boundary_x = x_coords + [x_coords[0]]
                boundary_y = y_coords + [y_coords[0]]
                add_direction_arrows(ax, boundary_x, boundary_y, boundary_color, every=1, zorder=2)
            
            # Print the boundary points
            print(f"\n{pocket_name} boundary points:")
            for (xx, yy) in zip(x_coords, y_coords):
                print(f"({xx}, {yy})")
        
        # ONLY generate zigzag for certain pockets
        if pocket_name in ["Pocket2", "Pocket3", "Pocket4"] and x_coords and y_coords:
            # ------------------------------------------------------------
            # 1) Compute bounding box, applying tool + inner offsets
            # ------------------------------------------------------------
            min_x = min(x_coords)
            max_x = max(x_coords)
            min_y = min(y_coords)
            max_y = max(y_coords)
            
            # "Left" and "Right" edges in X
            leftX  = min_x + tool3x + innerOffset + xframe_1
            rightX = max_x - tool3x - innerOffset - xframe_2
            
            # "Bottom" and "Top" edges in Y
            bottomY = min_y + tool3y + innerOffset + yframe_1
            topY    = max_y - tool3y - innerOffset - yframe_2
            
            xinner = abs(rightX - leftX)  # total width
            if xinner <= 0:
                # If there's no valid width left, skip
                continue
            
            # ------------------------------------------------------------
            # 2) Step in X, build zigzag lines from bottom to top
            # ------------------------------------------------------------
            if innerSandingOffset <= 0:
                print("inverseOverlapping must be > 0; using fallback 1.")
                innerSandingOffset = 1

            num_steps = math.ceil(xinner / innerSandingOffset)
            adjusted_step = xinner / num_steps
            
            offset = 0.0
            toggle = False  # We'll flip this each pass to reverse direction
            all_paths = []
            
            while offset <= xinner + 1e-9:  # floating-point margin
                current_x = leftX + offset
                
                # Depending on toggle, go up or go down
                if not toggle:
                    row_points = [
                        [current_x, bottomY],
                        [current_x, topY]
                    ]
                else:
                    row_points = [
                        [current_x, topY],
                        [current_x, bottomY]
                    ]
                
                all_paths.extend(row_points)
                offset += adjusted_step
                toggle = not toggle
            
            # ------------------------------------------------------------
            # 3) Plot the resulting zigzag
            # ------------------------------------------------------------
            zigzag_x = [p[0] for p in all_paths]
            zigzag_y = [p[1] for p in all_paths]
            zigzag_line, = ax.plot(
                zigzag_x, zigzag_y, label=f"{pocket_name} Zigzag Path", marker="o"
            )
            add_direction_arrows(
                ax, zigzag_x, zigzag_y, zigzag_line.get_color(), every=2, zorder=3
            )
            
            # Print the zigzag path points
            print(f"\n{pocket_name} Zigzag path points (left-to-right):")
            for (xx, yy) in all_paths:
                print(f"({xx}, {yy})")

    ax_cancel = plt.axes([0.2, 0.05, 0.1, 0.075])  # [left, bottom, width, height]
    ax_start = plt.axes([0.7, 0.05, 0.1, 0.075])

    # Create button objects
    btn_cancel = Button(ax_cancel, 'Cancel', color='lightcoral')
    btn_start = Button(ax_start, 'Start', color='lightgreen')
    
    result = {'action': None}

    def cancel_clicked(event):
        result['action'] = 'cancel'
        plt.close(fig)

    def start_clicked(event):
        result['action'] = 'start'
        plt.close(fig)
        
    btn_cancel.on_clicked(cancel_clicked)
    btn_start.on_clicked(start_clicked)
    
    # Configure plot settings
    ax.set_title("Combined Plot: Pockets with Left-to-Right Zigzag Paths")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.legend()
    
    plt.show()
    
    if result['action'] is None:
        result['action'] = 'cancel'
    
    return result['action']

if __name__ == "__main__":
    plot_data()
