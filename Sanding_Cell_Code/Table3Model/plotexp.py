import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math  # for dynamic step calculation
import json

# Hypothetical function to fetch points from the DB
from Utils.point_utils import get_points_coordinates_as_arrays

def load_json_config():
    """Loads configuration from config.json."""
    with open('./configs/cycleData.json', 'r') as file:
        config = json.load(file)
    return config


def plot_data(inverseOverlapping):
    print("Plotting data....")
    # Path to the SQLite database
    db_path = "./databases/pocket_measurements01.db"

    # Manually define the point labels
    point_labels = [
        "Point1", "Point2", "Point3", "Point4",
        "Point5", "Point6", "Point7", "Point8",
        "Point9", "Point10", "Point11", "Point12"
    ]

    print("the point_labels are: ",point_labels)

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
        required = [
            "Point1",
            "Point2",
            "Point3",
            "Point4",
            "Point9",
            "Point10",
            "Point11",
            "Point12",
        ]
        if any(results.get(k) is None for k in required):
            return []

        p1 = results["Point1"]
        p2 = results["Point2"]
        p3 = results["Point3"]
        p4 = results["Point4"]
        p9 = results["Point9"]
        p10 = results["Point10"]
        p11 = results["Point11"]
        p12 = results["Point12"]

        tool3y = 50.8
        tool3x = 38.1

        point4 = [p4[0] + tool3x - 9.749, p4[1] + tool3y - 19.243, p4[2] - 4]
        point1 = [p1[0] / 2.0, p4[1] + tool3y - 19.243, p4[2] - 4]
        point41 = [p4[0] + tool3x - 9.749 + 2, p4[1] + tool3y - 19.243, p4[2] - 4]
        point4pre = [p4[0] + tool3x - 9.749, p4[1] + tool3y - 19.243, p4[2] - 1 - 3 - 10]
        point1pre = [p1[0] / 2.0, p4[1] + tool3y - 19.243, p4[2] - 1 - 3 - 10]

        pointl2 = [p4[0] - tool3x + 11.138, p4[1] + tool3y - 19.243, p4[2] - 4]
        pointl3 = [p4[0] - tool3x + 11.138, p3[1] - tool3y + 23.769, p4[2] - 4]
        pointl2pre = [p4[0] - tool3x + 11.138, p4[1] + tool3y - 19.243, p4[2] - 1 - 3 - 10]
        pointl3pre = [p4[0] - tool3x + 11.138, p3[1] - tool3y + 23.769, p4[2] - 1 - 3 - 10]
        pointl23 = [p4[0] - tool3x + 11.138, p4[1] + tool3y - 19.243 + 1 + 1, p4[2] - 4]

        pointtop2 = [p2[0] / 2.0 - 32.4, p2[1] - 27.04, p2[2] - 4]
        pointtop21 = [p2[0] / 2.0 - 32.4 - 1 - 1, p2[1] - 27.04, p2[2] - 4]
        pointtop2pre = [p2[0] / 2.0 - 32.4, p2[1] - 27.04, p2[2] - 1 - 3 - 10]
        pointtop1 = [0, p2[1] - 27.04, p2[2] - 4]
        pointtop1pre = [0, p2[1] - 27.04, p2[2] - 1 - 3 - 10]

        point1right = [0 + tool3x - 9.749, p2[1] - tool3y + 23.76, p2[2] - 4]
        point14right = [0 + tool3x - 9.749, p2[1] - tool3y + 23.76 - 1 - 1, p2[2] - 4]
        point1rightpre = [0 + tool3x - 9.749, p2[1] - tool3y + 23.76, p2[2] - 1 - 3 - 10]
        point4right = [0 + tool3x - 9.749, p4[1], p2[2] - 4]
        point4rightpre = [0 + tool3x - 9.749, p4[1], p2[2] - 1 - 3 - 10]

        point1middle = [p12[0] / 2.0, 0 + tool3y - 20.8, -4]
        point1middlepre = [p12[0] / 2.0, 0 + tool3y - 20.8, -1 - 3 - 10]
        point12middle = [p12[0] / 2.0, 0 + tool3y - 20.8 + 1 + 1, -4]
        point2middle = [p12[0] / 2.0, p2[1] - tool3y + 23.76, -4]
        point3middle = [((p12[0] / 2.0) * 3) + 4.28, p2[1] - tool3y + 23.76, -4]
        point4middle = [((p12[0] / 2.0) * 3) + 4.28, 0 + tool3y - 20.8, -4]
        point4middlepre = [((p12[0] / 2.0) * 3) + 4.28, 0 + tool3y - 20.8, -1 - 3 - 10]

        points1 = [point4pre, point4, point41, point1, point1pre]
        pointsleft = [pointl2pre, pointl2, pointl23, pointl3, pointl3pre]
        pointstop = [pointtop2pre, pointtop2, pointtop21, pointtop1, pointtop1pre]
        pointright = [point1rightpre, point1right, point14right, point4right, point4rightpre]
        pointmiddle = [
            point1middlepre,
            point1middle,
            point12middle,
            point2middle,
            point3middle,
            point4middle,
            point4middlepre,
        ]

        def contact_only(points):
            return [p for p in points if p[2] >= -6]

        path = (
            contact_only(points1)
            + contact_only(pointsleft)
            + contact_only(pointstop)
            + contact_only(pointright)
            + contact_only(pointmiddle)
        )
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
        if pocket_name in ["Pocket2", "Pocket3"] and x_coords and y_coords:
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
