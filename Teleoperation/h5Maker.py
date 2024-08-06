import ast
import h5py
import os
from PIL import Image
import numpy as np

file_path = '/home/eckart/SHK_LIS/Teleoperation/recordings_new/proprioceptive.txt'

qpos_data = []
tau_external_data = []
last_number = []

with open(file_path, 'r') as file:
    for line in file:
        parts = line.split('] [')
        
        first_part = parts[0].strip('[').strip()
        second_part = parts[1].split('] ')[0].strip()
        last_part = parts[1].split('] ')[1].strip()
        
        first_list = ast.literal_eval(f'[{first_part}]')
        second_list = ast.literal_eval(f'[{second_part}]')
        last_value = int(last_part)
        first_list.append(last_value)

        qpos_data.append(first_list)
        tau_external_data.append(second_list)

print("First Column:")
print(qpos_data)
print("\nSecond Column:")
print(tau_external_data)


import os
from PIL import Image
import numpy as np

folder_path = 'recordings_new/images'

cam1_images = []
cam2_images = []

for filename in os.listdir(folder_path):
    if filename.endswith('.jpg') or filename.endswith('.png'):  # Modify the extensions as needed
        if filename.startswith('cam1'):
            image_path = os.path.join(folder_path, filename)
            image = Image.open(image_path).convert('RGB')  # Ensure image is in RGB mode

            image_array = np.array(image)

            cam1_images.append(image_array)
        
        elif filename.startswith('cam2'):
            image_path = os.path.join(folder_path, filename)
            image = Image.open(image_path).convert('RGB')  # Ensure image is in RGB mode

            image_array = np.array(image)

            cam2_images.append(image_array)

print(f'Number of cam1 images: {len(cam1_images)}')
print(f'Number of cam2 images: {len(cam2_images)}')
print(f'Number of tau ext: {len(tau_external_data)}')
print(f'Number of qpos: {len(qpos_data)}')




with h5py.File(f"episode_1.hdf5", "w") as hdf:
    hdf.attrs["sim"] = False
    
    hdf.create_dataset("/action", data=qpos_data[1:])
    
    observations_group = hdf.create_group("/observations")
    observations_group.create_dataset("qpos", data=qpos_data)
    observations_group.create_dataset("qvel", data=[])
    observations_group.create_dataset("tau", data=tau_external_data)

    images_group = observations_group.create_group("images")
    images_group.create_dataset("cam1", data=cam1_images)
    images_group.create_dataset("cam2", data=cam2_images)
    
    hdf.create_dataset("/waypoints", data=[])
