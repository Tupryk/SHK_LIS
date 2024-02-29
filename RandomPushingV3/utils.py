import numpy as np
from typing import List


def line_circle_intersection(line_pos, line_vec, circle_pos, circle_radious):

    a = line_vec[0]**2 + line_vec[1]**2
    b = 2*(line_vec[0]*line_pos[0] + line_vec[1]*line_pos[1] - line_vec[0]*circle_pos[0] - line_vec[1]*circle_pos[1])
    c = circle_pos[0]**2 + circle_pos[1]**2 - 2*line_pos[0]*circle_pos[0] - 2*line_pos[1]*circle_pos[1] - circle_radious**2 + line_pos[0]**2 + line_pos[1]**2

    tmp = b**2-4*a*c

    if tmp < 0:
        return []
    if tmp == 0:
        t = -b/(2*a)
        return [np.array([line_vec[0]*t+line_pos[0], line_vec[1]*t+line_pos[1]])]

    t = (-b+np.sqrt(tmp))/(2*a)
    result = [np.array([line_vec[0]*t+line_pos[0], line_vec[1]*t+line_pos[1]])]
    t = (-b-np.sqrt(tmp))/(2*a)
    result.append(np.array([line_vec[0]*t+line_pos[0], line_vec[1]*t+line_pos[1]]))
    return result


def line_rect_intersection(line_pos, line_vec, pos, rect_width, rect_height):
    left_x= -1/2*rect_width+pos[0]
    right_x= 1/2*rect_width+pos[0]
    upper_y= 1/2*rect_height+pos[1]
    lower_y= -1/2*rect_height+pos[1]    # Find the parametric equation of the line: P = position_vector + t * direction_vector
    # where P is any point on the line, and t is a scalar parameter.

    # Define the rectangle
    rectangle_x = (left_x, right_x)  # X-coordinate range of the rectangle
    rectangle_y = (lower_y, upper_y)  # Y-coordinate range of the rectangle

    # Find the parametric equation of the line: P = position_vector + t * direction_vector
    # where P is any point on the line, and t is a scalar parameter

    # Initialize lists to store intersection points
    intersections = []

    # Check intersection with top edge of the rectangle (y = y2)
    if line_vec[1] != 0:
        t = (rectangle_y[1] - line_pos[1]) / line_vec[1]
        intersection = line_pos + t * line_vec
        if rectangle_x[0] <= intersection[0] <= rectangle_x[1]:
            intersections.append(intersection)

    # Check intersection with bottom edge of the rectangle (y = y1)
    if line_vec[1] != 0:
        t = (rectangle_y[0] - line_pos[1]) / line_vec[1]
        intersection = line_pos + t * line_vec
        if rectangle_x[0] <= intersection[0] <= rectangle_x[1]:
            intersections.append(intersection)

    # Check intersection with left edge of the rectangle (x = x1)
    if line_vec[0] != 0:
        t = (rectangle_x[0] - line_pos[0]) / line_vec[0]
        intersection = line_pos + t * line_vec
        if rectangle_y[0] <= intersection[1] <= rectangle_y[1]:
            intersections.append(intersection)

    # Check intersection with right edge of the rectangle (x = x2)
    if line_vec[0] != 0:
        t = (rectangle_x[1] - line_pos[0]) / line_vec[0]
        intersection = line_pos + t * line_vec
        if rectangle_y[0] <= intersection[1] <= rectangle_y[1]:
            intersections.append(intersection)

    # Now, 'intersections' contains all the intersection points within the rectangle.
    return intersections


def segment_line(point1, point2, point_between):
    return [point1 + (point2 - point1) * 0.5 * (1-np.cos(np.pi * i/(point_between-1))) for i in range(point_between)]


def pathLength(waypoints: List[np.ndarray]) -> float:
    sum_ = 0
    for i, _ in enumerate(waypoints[1:]):
        sum_ += np.linalg.norm(waypoints[i]-waypoints[i-1])
    return sum_
    
def extract_position_and_quaternion(pose_matrix):
    # Extract position (translation) from the last column of the pose matrix
    position = pose_matrix[:3, 3]

    # Extract the rotation matrix from the upper-left 3x3 submatrix
    rotation_matrix = pose_matrix[:3, :3]

    # Compute quaternion from rotation matrix
    trace = np.trace(rotation_matrix)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2  # S = 4 * qx
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qx = 0.25 * S
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2  # S = 4 * qy
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qy = 0.25 * S
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2  # S = 4 * qz
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        qz = 0.25 * S
    
    quaternion = np.array([qw, qx, qy, qz])

    return position, quaternion
