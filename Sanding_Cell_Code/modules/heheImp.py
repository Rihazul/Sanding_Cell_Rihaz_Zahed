import numpy as np
import csv
import math

def mm_to_inches(mm):
    inches = mm / 25.4
    return inches

def csv_to_dict_list(file_path):
    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        headers = next(reader)  # Read the headers

        # Convert each row's values to float where possible
        def convert_to_float(value):
            try:
                return float(value)
            except ValueError:
                return value  # Keep the original value if conversion fails

        # Build the list of dictionaries with converted values
        return [
            {header: convert_to_float(value) for header, value in zip(headers, row)}
            for row in reader
        ]
    
def find_x_groups(data, n):
            """
            This will return the groups in which valid heights are not NaN.
            A new group is started if n consecutive NaN values are encountered.

            Args:
                data (list): A list of dictionaries containing 'height' values.
                n (int): The number of consecutive NaN values required to start a new group.

            Returns:
                list: A list of groups (each group is a list of dictionaries).
            """
            grouped_valids = []  # To store lists of valid dictionaries
            current_group = []   # To accumulate valid dictionaries in the current group
            nan_count = 0        # To track consecutive NaN values

            for item in data:
                height_value = item.get('height', float('nan'))  # Get the 'height' value, default to NaN if not found

                if math.isnan(float(height_value)):
                    nan_count += 1  # Increment the NaN counter
                else:
                    nan_count = 0   # Reset the NaN counter if a valid value is found
                    current_group.append(item)  # Add valid items to the current group

                # If we encounter n consecutive NaN values, we close the current group
                if nan_count == n:
                    if current_group:
                        grouped_valids.append(current_group)
                        current_group = []  # Reset the current group for the next batch
                    nan_count = 0  # Reset the NaN counter after splitting

            # If there are any valid entries left in the last group, add them too
            if current_group:
                grouped_valids.append(current_group)
            
            return grouped_valids

import numpy as np
import math

# def identify_gradient_change_points_dynamic(chunks, thresholds):
#     """
#     Identifies the exact points where the gradient starts and stops using start and stop thresholds.

#     Args:
#     - chunks (list): List of dictionaries containing 'dist' and 'height'.
#     - thresholds (tuple): Start and stop thresholds for detecting gradient changes.

#     Returns:
#     - result (dict): Dictionary with calculated frame and pocket distances.
#     """
#     # Filter out chunks with NaN heights
#     chunks = [item for item in chunks if not math.isnan(item['height'])]

#     # Extract distances and heights
#     distances = [item['dist'] for item in chunks]
#     heights = [item['height'] for item in chunks]
#     start_threshold = thresholds[0]
#     stop_threshold = thresholds[1]
#     min_distance = 15  

#     gradient_changes = []
#     gradients = np.diff(heights) / np.diff(distances)

#     positive_start_idx = None

#     for i in range(1, len(gradients)):
#         if distances[i] < min_distance:
#             continue  # Skip points until the distance is at least min_distance

#         # Detect start of positive gradient
#         if abs(gradients[i]) >= start_threshold and positive_start_idx is None:
#             positive_start_idx = i

#         # Detect end of positive gradient
#         elif positive_start_idx is not None and abs(gradients[i]) < stop_threshold:
#             gradient_changes.append({
#                 'type': 'positive_start',
#                 'distance': distances[positive_start_idx],
#                 'gradient': gradients[positive_start_idx]
#             })
#             gradient_changes.append({
#                 'type': 'positive_stop',
#                 'distance': distances[i],
#                 'gradient': gradients[i]
#             })
#             positive_start_idx = None  # Reset

#     # Check for pairs of positive starts and stops with distance < 10
#     for j in range(1, len(gradient_changes) - 1, 2):  # Iterate over pairs
#         if (gradient_changes[j]['type'] == 'positive_stop' and 
#             gradient_changes[j - 1]['type'] == 'positive_start' and 
#             abs(gradient_changes[j]['distance'] - gradient_changes[j - 1]['distance']) < 1):
            
#             # Calculate average gradient between these start and stop points
#             avg_gradient = (gradient_changes[j - 1]['gradient'] + gradient_changes[j]['gradient']) / 2
#             gradient_changes[j - 1]['avg_gradient'] = avg_gradient  # Attach avg_gradient to start
#             gradient_changes[j]['avg_gradient'] = avg_gradient      # Attach avg_gradient to stop

#     # Extract specific points
#     frame1_point1 = float(chunks[0]['dist'])
#     frame1_point2 = gradient_changes[0]['distance']
#     pocket_point1 = gradient_changes[1]['distance']
#     pocket_point2 = gradient_changes[2]['distance']
#     frame2_point1 = gradient_changes[3]['distance']
#     frame2_point2 = float(chunks[-1]['dist'])

#     # Calculate distances as per given structure
#     frame1 = frame1_point2 - frame1_point1
#     pocket = pocket_point2 - pocket_point1
#     threeD1 = pocket_point1 - frame1_point2
#     threeD2 = frame2_point1 - pocket_point2
#     frame2 = frame2_point2 - frame2_point1

#     result = {
#         'frame_1': frame1,
#         'threeD_1': threeD1,
#         'pocket': pocket,
#         'threeD_2': threeD2,
#         'frame_2': frame2
#     }
    
#     # Attach gradient changes for reference
#     # result = gradient_changes
#     return result


def identify_gradient_change_points_dynamic(chunks, thresholds):
        """
        Identifies the exact points where the gradient starts and stops using start and stop thresholds.

        Args:
        - distances (list): List of distances.
        - heights (list): List of corresponding heights.
        - threshold (float): The threshold for detecting the start and stop of a gradient.
        - min_distance (float): Minimum distance after which to detect the first gradient change.

        Returns:
        - gradient_changes (list of dicts): List of points where gradients start and stop with the distance.
        """
        chunks = [item for item in chunks if not math.isnan(item['height'])]

        distances = [item['dist'] for item in chunks]
        heights = [item['height'] for item in chunks]
        start_threshold = thresholds[0]  # Threshold for detecting the start and stop of a gradient
        stop_threshold = thresholds[1]
        min_distance = 15  

        gradient_changes = []
        gradients = np.diff(heights) / np.diff(distances)

        positive_start_idx = None

        # Loop through gradients, starting only after the specified minimum distance
        for i in range(1, len(gradients)):
            if distances[i] < min_distance:
                continue  # Skip points until the distance is at least 10mm

            # Detect start of positive gradient
            if abs(gradients[i]) >= start_threshold and positive_start_idx is None:
                positive_start_idx = i

            # Detect end of positive gradient
            elif positive_start_idx is not None and abs(gradients[i]) < stop_threshold:
                gradient_changes.append({
                    'type': 'positive_start',
                    'distance': distances[positive_start_idx],
                    'gradient': gradients[positive_start_idx]
                })
                gradient_changes.append({
                    'type': 'positive_stop',
                    'distance': distances[i],
                    'gradient': gradients[i]
                })
                positive_start_idx = None  # Reset

        frame1_point1 = float(chunks[0]['dist'])
        frame1_point2 = gradient_changes[0]['distance']
        pocket_point1 = gradient_changes[1]['distance']
        pocket_point2 = gradient_changes[2]['distance']
        frame2_point1 = gradient_changes[3]['distance']
        frame2_point2 = float(chunks[-1]['dist'])

        
        frame1 = frame1_point2 - frame1_point1
        pocket = pocket_point2 - pocket_point1
        threeD1 = pocket_point1 - frame1_point2
        threeD2 = frame2_point1 - pocket_point2
        frame2 = frame2_point2 - frame2_point1

        result = {
            'frame_1': frame1,
            'threeD_1': threeD1,
            'pocket': pocket,
            'threeD_2': threeD2,
            'frame_2': frame2
        }
        return result

def identify_gradient_change_points_simple(chunks, threshold=1, min_stable_distance=0):
    """
    Identifies stable regions where the change in height is below a specified threshold for a minimum distance.

    Args:
    - chunks (list of dicts): Each dict contains 'dist' and 'height'.
    - threshold (float): The threshold for detecting stable regions based on height change.
    - min_stable_distance (float): Minimum distance required for a region to be considered stable.

    Returns:
    - stable_regions (list of dicts): Each dict contains the start and end distances of a stable region.
    """
    # Filter out NaN values in height data
    chunks = [item for item in chunks if not math.isnan(item['height'])]
    compensation = 0
    # Extract distances and heights
    distances = [item['dist'] for item in chunks]
    heights = [item['height'] for item in chunks]
    
    
    stable_start_idx = None  # Track start of stable region
    stable_regions = []

    for i in range(1, len(heights)):
        # Calculate change in height from the previous point
        height_diff = abs(heights[i] - heights[i - 1])

        # Check if the change in height is below the threshold
        if height_diff < threshold:
            if stable_start_idx is None:
                stable_start_idx = i - 1  # Mark the start of a stable region
        else:
            # If height change exceeds the threshold and we were in a stable region
            if stable_start_idx is not None:
                # Check if stable region meets the minimum distance requirement
                start_dist = distances[stable_start_idx]
                end_dist = distances[i - 1]
                # start_dist = distances[stable_start_idx]
                # end_dist = distances[i-1]
                if end_dist - start_dist >= min_stable_distance:
                    stable_regions.append({
                        'start_distance': start_dist,
                        'end_distance': end_dist
                    })
                stable_start_idx = None  # Reset stable start index

    # Check the final region if the data ends in a stable region
    if stable_start_idx is not None:
        start_dist = distances[stable_start_idx]
        end_dist = distances[-1]
        if end_dist - start_dist >= min_stable_distance:
            stable_regions.append({
                'start_distance': start_dist,
                'end_distance': end_dist
            })

    print(stable_regions)
    input()
    
    frame1_point1 = float(chunks[0]['dist'])
    frame1_point2 = stable_regions[0]['end_distance']
    pocket_point1 = stable_regions[0]['end_distance']
    pocket_point2 = stable_regions[-1]['start_distance']
    frame2_point1 = stable_regions[-1]['start_distance']
    frame2_point2 = float(chunks[-1]['dist'])


    frame1 = frame1_point2 - frame1_point1 + compensation
    pocket = pocket_point2 - pocket_point1 + compensation
    threeD1 = pocket_point1 - frame1_point2 + compensation
    threeD2 = frame2_point1 - pocket_point2 + compensation
    frame2 = frame2_point2 - frame2_point1 + compensation

    result = {
            'frame_1': frame1,
            'threeD_1': threeD1,
            'pocket': pocket,
            'threeD_2': threeD2,
            'frame_2': frame2
        }
    
    return result



x_measures = csv_to_dict_list("./static/xmeasures.csv")
allXMeasurements = find_x_groups(x_measures, 4)
# print(len(allXMeasurements))
xVals = []
for xcnt, xmeasurements in enumerate(allXMeasurements):
    ###############################
    ####### point calcu x     #####
    ###############################
    xpos = xmeasurements[0]['dist'] # this is position of 7th axis
    # xlen, xframe_1, xframe_2 = find_constant_height_periods(xmeasurements, threshold=2)
    # results = calculate_for_each_chunk([xmeasurements], thresholds=[xthresholds[xcnt]])
    results = identify_gradient_change_points_simple(xmeasurements, threshold=0.3, min_stable_distance=8)
    # results = identify_gradient_change_points_dynamic(xmeasurements, thresholds=[0.15, 0.1])
    # print(f"Door{xcnt+1}: ")
    # print (results)

    xframe_1, x_td1,  x_pocket, x_td2, xframe_2 = results['frame_1'], results['threeD_1'], results['pocket'], results['threeD_2'], results['frame_2']
    xlen = xframe_1 + x_td1 + x_pocket + x_td2 + xframe_2
    xVals.append({'xlen':xlen, 'xframe_1': xframe_1 + x_td1, 'xframe_2': xframe_2 + x_td2})


for i in range (0, len(xVals)):
    print(f"Door {i+1}", xVals[i])