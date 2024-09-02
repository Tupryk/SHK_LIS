import numpy as np
import robotic as ry 
import matplotlib.pyplot as plt

def extract_np_arrays(file_path):
    pos = []
    quat = []

    with open(file_path, 'r') as file:
        for line in file:
            left_str, right_str = line.split('] [')

            left_list = [float(x) for x in left_str.strip('[] \n').split(',')]
            right_list = [float(x) for x in right_str.strip('[] \n').split(',')]

            left_array = np.array(left_list)
            right_array = np.array(right_list)

            pos.append(left_array)
            quat.append(right_array)

    return pos, quat


pos, quat = extract_np_arrays("recording/poses.txt")


def extract_proprioceptives(file_path):
    qs = []
    taus = []
    gripper_widths = []

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split('] [')

            if len(parts) == 2:
                right_str, float_str = parts[1].rsplit('] ', 1)

                qs_list = [float(x) for x in parts[0].strip('[]').split(',')]
                taus_list = [float(x) for x in right_str.strip('[]').split(',')]
                width_value = float(float_str.strip())

                left_array = np.array(qs_list)
                right_array = np.array(taus_list)

                qs.append(left_array)
                taus.append(right_array)
                gripper_widths.append(width_value)
            else:
                print(f"Unexpected line format: {line}")

    return qs, taus, gripper_widths


def find_switch_indices_with_delta(values, delta):
    switch_indices = []
    
    for i in range(1, len(values)):
        if abs(values[i] - values[i - 1]) > delta:
            switch_indices.append(i)
    
    return switch_indices


_, __, gripper_widths = extract_proprioceptives("recording/proprioceptive.txt")


delta = 0.02  # gripper change to detect a waypoint
switch_indices = find_switch_indices_with_delta(gripper_widths, delta)
print(f"switch indices: {switch_indices}")

gripper_switch_waypoints = []
gripper_switch_quats = []


for i in switch_indices:
    gripper_switch_waypoints.append(pos[i])
    gripper_switch_quats.append(quat[i])


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))


#### TODO change height heuristic ####
def find_max_z_coordinate_index(values):
    max_index = max(range(len(values)), key=lambda i: values[i][-1])
    return max_index

def find_min_z_coordinate_index(values):
    max_index = min(range(len(values)), key=lambda i: values[i][-1])
    return max_index

# just extract the max between first and last gripper width change ---- maybe change in future
max_pos = pos[switch_indices[0]:switch_indices[-1]+1]

min_index = find_min_z_coordinate_index(max_pos)

print(min_index+switch_indices[0])

max_index = find_max_z_coordinate_index(max_pos[min_index:])
print(max_index+min_index+switch_indices[0])

print(f"max after min xyz: {pos[switch_indices[0]+max_index+min_index]}")
C.addFrame('max_after_min'). setShape(ry.ST.marker, [.1]) .setPosition(pos[switch_indices[0]+max_index+min_index]) .setColor([64,128,64])


C.view(True, "height waypoint added")


for i, q in enumerate(gripper_switch_waypoints):
    C.addFrame(f'task_way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(q) .setColor([255,0,128]) .setQuaternion(gripper_switch_quats[i])
    
C.view(True, "gripper waypoints")


#### Curvature waypoint only in translation ####
def calculate_curvature(path):
    curvatures = []
    for i in range(1, len(path) - 1):
        v1 = np.array(path[i]) - np.array(path[i-1])
        v2 = np.array(path[i+1]) - np.array(path[i])
        
        # Cross product magnitude
        cross_product_magnitude = np.linalg.norm(np.cross(v1, v2))
        
        # Magnitudes of vectors
        v1_magnitude = np.linalg.norm(v1)
        v2_magnitude = np.linalg.norm(v2)
        
        # Calculate curvature
        curvature = cross_product_magnitude / (v1_magnitude * v2_magnitude)
        curvatures.append(curvature)
    
    return curvatures

def find_max_curvature_timestep(path):
    curvatures = calculate_curvature(path)
    max_curvature_index = np.argmax(curvatures) + 1  # +1 to account for the offset in indexing
    return max_curvature_index, curvatures[max_curvature_index - 1]


curvs = calculate_curvature(pos)
print(find_max_curvature_timestep(pos))

print("test", find_max_curvature_timestep(pos)[1])
C.addFrame('max_curv'). setShape(ry.ST.marker, [.1]) .setPosition(pos[find_max_curvature_timestep[0]]) .setColor([128,78,223]) .setQuaternion(quat[find_max_curvature_timestep[0]])
#print(f"max curce xzy: {pos[find_max_curvature_timestep[0]]}")
C.view(True, "curvs waypoints")

###############################################################################################f