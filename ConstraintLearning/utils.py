import numpy as np


import numpy as np

def load_coordinates(filename):
    """
    Loads and parses the coordinates from a text file.
    Format should be:

    [(x1,y1,z1), (x2,y2,z2), ..., (xn,yn,zn)]

    [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), ... , (10.0, 11.0, 12.0)]

    , where i=1 reprents the x,y,z coordinates from the ith observation/recording i.e.,
    the max height or curviture in x,z,y from N=n total observations/recordings.
    
    Parameters:
    filename (str): The path to the file containing the coordinates.
    
    Returns:
    tuple of lists: Three lists containing x, y, and z coordinates respectively.
    """
    with open(filename, 'r') as file:
        data = file.read()

    # Extract the coordinates
    data = data.strip()[1:-1]  # Remove the surrounding brackets
    data = data.split('), (')  # Split each tuple

    x_coords = []
    y_coords = []
    z_coords = []

    for item in data:
        x, y, z = map(float, item.replace('(', '').replace(')', '').split(','))
        x_coords.append(x)
        y_coords.append(y)
        z_coords.append(z)
    
    return x_coords, y_coords, z_coords

def compute_mean(filename):
    """
    Computes the mean of x, y, and z coordinates from a file.
    
    Parameters:
    filename (str): The path to the file containing the coordinates.
    
    Returns:
    tuple: The means of the x, y, and z coordinates.
    """
    x_coords, y_coords, z_coords = load_coordinates(filename)
    
    mean_x = np.mean(x_coords)
    mean_y = np.mean(y_coords)
    mean_z = np.mean(z_coords)
    
    return mean_x, mean_y, mean_z

def compute_standard_dev(filename):
    """
    Computes the standard deviation of x, y, and z coordinates from a file.
    
    Parameters:
    filename (str): The path to the file containing the coordinates.
    
    Returns:
    tuple: The standard deviations of the x, y, and z coordinates.
    """
    x_coords, y_coords, z_coords = load_coordinates(filename)
    
    std_x = np.std(x_coords)
    std_y = np.std(y_coords)
    std_z = np.std(z_coords)
    
    return std_x, std_y, std_z

def main():
    filename = 'coordinates_test_data.txt'
    
    # Compute mean
    mean_x, mean_y, mean_z = compute_mean(filename)
    
    # Compute standard deviation
    std_x, std_y, std_z = compute_standard_dev(filename)
    
    # Display the results
    print(f"Mean of x: {mean_x}, y: {mean_y}, z: {mean_z}")
    print(f"Standard deviation of x: {std_x}, y: {std_y}, z: {std_z}")

# Run the main function
if __name__ == "__main__":
    main()
