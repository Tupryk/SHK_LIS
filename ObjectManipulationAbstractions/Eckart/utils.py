import numpy as np
import robotic as ry


def get_quaternion_from_euler(roll, pitch, yaw): # Borrowed from https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def x_rotation(vector,theta):
    R = np.array([[1,0,0], [0,np.cos(theta),-np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    return np.dot(R,vector)

def y_rotation(vector,theta):
    R = np.array([[np.cos(theta),0,np.sin(theta)], [0,1,0], [-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(R,vector)

def z_rotation(vector,theta):
    R = np.array([[np.cos(theta), -np.sin(theta),0], [np.sin(theta), np.cos(theta),0], [0,0,1]])
    return np.dot(R,vector)

def visualizeWaypoints(C, waypoints):
    for i, way in enumerate(waypoints):
        C.addFrame(f'way{i}') \
            .setShape(ry.ST.marker, [.1]) \
            .setColor([1., 1., 0.]) \
            .setPosition(way)
