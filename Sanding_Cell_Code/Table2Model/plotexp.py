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
        "Point5", "Point6", "Point7", "Point8"
    ]

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
        "Pocket2": ["Point5", "Point6", "Point7", "Point8"]
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

    def add_direction_arrows(ax, xs, ys, color, every=1, zorder=3):
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

    def annotate_corner_values(ax, x_coords, y_coords, color):
        for (xx, yy) in zip(x_coords, y_coords):
            ax.text(
                xx + 5,
                yy + 5,
                f"({xx:.1f}, {yy:.1f})",
                fontsize=8,
                color=color,
                zorder=5,
            )

    def compute_frame_tool_path(results):
        required = ["Point5", "Point6", "Point7", "Point8"]
        if any(results.get(k) is None for k in required):
            return []
        p5 = results["Point5"]
        p6 = results["Point6"]
        p7 = results["Point7"]
        p8 = results["Point8"]
        z = -4

        outeroffset = p7[0] / 2.0
        p5o = [p5[0] + outeroffset, p5[1] - outeroffset, z]
        p6o = [p6[0] + outeroffset, p6[1] + outeroffset, z]
        p7o = [p7[0] - outeroffset, p7[1] + outeroffset, z]
        p8o = [p8[0] - outeroffset, p8[1] - outeroffset, z]

        distance = p5o[0] - p8o[0]
        base_x = p8o[0]
        point8 = p8o
        point5 = p5o
        point6 = p6o
        point7 = p7o

        bpoint1st = [base_x + 0, point8[1], z]
        bpoint2nd = [base_x + distance / 2.0, point5[1], z]
        bpointmiddle = [base_x + 0 + 2, point8[1], z]
        bpoints = [bpoint1st, bpointmiddle, bpoint2nd]

        lpoint1st = [base_x + 0, point5[1], z]
        lpoint2nd = [base_x + 0, point6[1], z]
        lpointmiddle = [base_x + 0, point5[1] + 2, z]
        leftpoints = [lpoint1st, lpointmiddle, lpoint2nd]

        tpoint1st = [base_x + 0, point6[1], z]
        topointmiddle = [base_x + 0 + 2, point6[1], z]
        tpoint2nd = [base_x - distance / 2.0, point7[1], z]
        top_mid_to_right = [tpoint1st, topointmiddle, tpoint2nd]
        top_mid_to_left = [tpoint2nd, topointmiddle, tpoint1st]

        rpoint1st = [base_x + 0, point7[1], z]
        rpoint2nd = [base_x + 0, point8[1], z]
        rpointmiddle = [base_x + 0, point7[1] - 2, z]
        rightpoints = [rpoint1st, rpointmiddle, rpoint2nd]

        path = top_mid_to_right + rightpoints + bpoints + bpoints + leftpoints + top_mid_to_left
        return [(p[0], p[1]) for p in path]

    # Loop through all pockets
    for pocket_name, point_names in pockets.items():
        x_coords = []
        y_coords = []
        z_coords = []
        
        # Gather data for this pocket
        for pt in point_names:
            point = results.get(pt)
            if point is not None:
                x_coords.append(point[0])
                y_coords.append(point[1])
                z_coords.append(point[2])
        
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
            annotate_corner_values(ax, x_coords, y_coords, boundary_color)
            if pocket_name == "Pocket1":
                frame_path = compute_frame_tool_path(results)
                if frame_path:
                    fx = [p[0] for p in frame_path]
                    fy = [p[1] for p in frame_path]
                    ax.plot(
                        fx,
                        fy,
                        color=boundary_color,
                        linestyle="-",
                        label="Pocket1 Frame Path",
                        zorder=2,
                    )
                    add_direction_arrows(ax, fx, fy, boundary_color, every=1, zorder=3)
            
            # Print the boundary points
            print(f"\n{pocket_name} boundary points:")
            for (xx, yy) in zip(x_coords, y_coords):
                print(f"({xx}, {yy})")
        
        # ONLY generate zigzag for certain pockets
        if pocket_name in ["Pocket2"] and x_coords and y_coords:
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
                ax, zigzag_x, zigzag_y, zigzag_line.get_color(), every=1, zorder=3
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
