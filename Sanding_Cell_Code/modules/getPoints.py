import numpy as np

def CoordinatesGenerator(valuesToHave, lowVal, highVal, threshold):
    """
    Generate an array of float values with the specified properties.

    Parameters:
    - n_close_to_5_before (int): Number of values close to 5 before the values close to 10.
    - n_close_to_10 (int): Number of values close to 10.
    - n_close_to_5_after (int): Number of values close to 5 after the values close to 10.
    - threshold (float): The deviation threshold for values around 5 and 10.

    Returns:
    - np.ndarray: Array of float values with the specified properties.
    """
    resultArr = np.array([])
    for i, cnt in enumerate(valuesToHave):
        x = lowVal if i % 2 else highVal
        shortArr = np.random.uniform(x - threshold, x + threshold, cnt)
        resultArr = np.concatenate((resultArr, shortArr), axis=0)
    
    return resultArr

def changeCoordinateFinder(seq_values, float_values, threshold):
    """
    Find the points where the values in float_values suddenly rise or fall.

    Parameters:
    - seq_values (np.ndarray): Array of sequential values (e.g., [1, 2, 3, ..., n]).
    - float_values (np.ndarray): Array of float values.
    - threshold (float): The threshold for detecting a sudden change.

    Returns:
    - np.ndarray: Array of points (from seq_values) where the sudden rise/fall occurs.
    """
    if len(seq_values) != len(float_values):
        raise ValueError("Both input arrays must have the same length.")
    
    sudden_changes = []
    
    for i in range(1, len(float_values)):
        if abs(float_values[i] - float_values[i-1]) > threshold:
            sudden_changes.append(seq_values[i])
    
    return np.array(sudden_changes)

if __name__ == "__main__":
    threshold = 0.1
    lens = [5, 7, 12, 5, 7, 12]
    n =  np.sum(lens)
    seventhAxisValues = np.arange(1, n+1)
    laserSensorValues = CoordinatesGenerator(lens, 5, 10, threshold)

    coordinatesY = changeCoordinateFinder(seventhAxisValues, laserSensorValues, threshold=0.5)
    print(coordinatesY)
    # zipping zippingg
    
    # xy_tuples = list(zip(X, Y))
    # print(xy_tuples)