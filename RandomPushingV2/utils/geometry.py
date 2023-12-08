import numpy as np

### multiplied out version of line equation (vector form) into circle eq. seperated to a, b, c for final solving with abc-formula

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
