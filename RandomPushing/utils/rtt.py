import random
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node:
    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent

def random_sample_3d(lower = 0, upper = 10):
    '''
    Randomly samples a 3D point within specified bounds.

    Args:
        lower (int): lower bound of the sampling distribution.
        upper (int): upper bound of the sampling distribution.

    Returns: 3D points (tuple): x,y,z coordinates.
    '''
    return (random.uniform(lower, upper), random.uniform(lower, upper), random.uniform(lower, upper))

def steer_3d(from_points, to_point, step_size=0.5):
    '''
    Steers or moves towards target objects within a given constraint i.e., step size.

    Args:
        from point (tuple): 3d points from the targer. 
        to points (tuple): 3d points towards the target.
        step size (float): constrains the movement from points to target point by step size.
    
    Returns:
        new 3d coordinates (tuple): 3D points.
    '''
     
    new_3dcoords = []
    for i in range(3):
        distance = to_point[i] - from_points[i]
        step = min(step_size, abs(distance))
        new_coordinate = from_points[i] + step * math.copysign(1, distance)
        new_3dcoords.append(new_coordinate)
    return tuple(new_3dcoords)

def calculate_distances(points, obstacle_center):
    distances = [abs(points[i] - obstacle_center[i]) for i in range(3)]
    return distances

def check_collision(distances, obstacle_dimensions):
    '''
    Checks for collision. 

    Args: 
        distances (list): of distances along x,y,z axis
        obstacles dimensions (tuple): x,y,z coordinates 
    Returns:
        True or False (bool): False if colesison detected True if valid path  
    '''
    for i in range(3):
        if distances[i] > obstacle_dimensions[i] / 2.0:
            return False
    return True

def is_valid_path(points, obstacles):
    '''
    Checks for valid path. 

    Args: 
        point (tuple): 3D points.
        obstacles (dictionary of tuples): 3D points. 
    Returns:
        True or False (bool): False if colesison detected True if valid path  
    '''
    for obstacle in obstacles:
        obstacle_center = obstacle['center']
        obstacle_dimensions = obstacle['dimensions']
        distances = calculate_distances(points, obstacle_center)
        if check_collision(distances, obstacle_dimensions):
            return False
    return True 

def distance_3d(points1, points2):
    '''
    Computes eucledian distance between two 3D points.
    
    Args: 
        points1 (tuple): x,y,z of 3D point. 
        points2 (tuple): x,y,z of 3D point.
    
    Returns
        eucledian distance (float): distance between the two 3D points. 

    '''
    squared_sum = 0
    for i in range(3):  
        squared_sum += (points1[i] - points2[i])**2
    return math.sqrt(squared_sum)

def rrt_3d(startpos, goalpos, obstacles, max_iterations=3000, threshold=0.5):
    '''
    Computes path between start and goal pos using rapidly exploring randomg trees (rrt).
    
    Args: 
        startpos (tuple): x,y,z of 3D point. 
        goalpos (tuple): x,y,z of 3D point.
        max_iterations (int): number of iterations
        threshold (float): treshold param to only construct path between two nodes if dist smaller than treshold. 
        obstacles (list of tuples ): list of x,y,z coordinates i.e., [(2,2,2), (5,5,5)]
    
    Returns
        eucledian distance (float): distance between the two 3D points. 

    '''
    
    tree = [Node(startpos)]

    for _ in range(max_iterations):
        random_config = random_sample_3d()
        nearest_node = min(tree, key=lambda node: distance_3d(node.config, random_config))

        new_points = steer_3d(nearest_node.config, random_config)
        if is_valid_path(new_points, obstacles):
            new_node = Node(new_points, parent=nearest_node)
            tree.append(new_node)
            if distance_3d(new_points, goalpos) < threshold:  
                return construct_path(new_node)

    return None

def construct_path(end_node):
    path = []
    node = end_node
    while node is not None:
        path.append(node.config)
        node = node.parent
    return list(reversed(path))

def visualize_environment(path=None, obstacles=[]):
    '''
    Visualises rrt path and obstacles. 
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for obstacle in obstacles:
        center = obstacle['center']
        dimensions = obstacle['dimensions']
        x, y, z = center
        dx, dy, dz = dimensions
        ax.bar3d(x - dx/2, y - dy/2, z - dz/2, dx, dy, dz, color='red', shade=True)

    if path:
        path_coords = list(zip(*path))
        ax.plot(*path_coords, marker='o', color='blue')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

# Example usage and test
start = (0, 0, 0)
goal = (9, 9, 9)
obstacles = [
    {'center': (5, 5, 5), 'dimensions': (2, 2, 2)},
    {'center': (2, 2, 2), 'dimensions': (1, 3, 1)}
    # Add more obstacles here
]
path = rrt_3d(start, goal, obstacles)
if path:
    print("Path found:", path)
    visualize_environment(path, obstacles)
else:
    print("Path not found")
    visualize_environment(obstacles=obstacles)
