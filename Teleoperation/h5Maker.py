import ast
import h5py
import os
from PIL import Image
import numpy as np

# Set the path to the main directory containing subdirectories
main_directory = '/home/eckart/SHK_LIS/Teleoperation/recordings/'


def process_proprioceptive_data(file_path):
    qpos_data = []
    tau_external_data = []
    
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

    return qpos_data, tau_external_data

# Function to process images from a folder
def process_images(folder_path):
    cam1_images = []
    cam2_images = []

    for filename in os.listdir(folder_path):
        if filename.endswith('.jpg') or filename.endswith('.png'):  # Modify the extensions as needed
            image_path = os.path.join(folder_path, filename)
            image = Image.open(image_path).convert('RGB')  # Ensure image is in RGB mode
            image_array = np.array(image)

            if filename.startswith('cam1'):
                cam1_images.append(image_array)
            elif filename.startswith('cam2'):
                cam2_images.append(image_array)

    return cam1_images, cam2_images


def save_hdf5(filename, qpos_data, tau_external_data, cam1_images, cam2_images):
    with h5py.File(filename, "w") as hdf:
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


for subdir in os.listdir(main_directory):
    subdir_path = os.path.join(main_directory, subdir)
    
    if os.path.isdir(subdir_path):
        data_folder_path = os.path.join(subdir_path, 'images')
        print(f"Processing data folder: {data_folder_path}")
        
        # Process proprioceptive.txt
        txt_file_path = os.path.join(data_folder_path, 'proprioceptive.txt')
        qpos_data, tau_external_data = process_proprioceptive_data(txt_file_path)
        
        # Process images
        images_folder_path = os.path.join(data_folder_path, 'images')
        cam1_images, cam2_images = process_images(images_folder_path)
        
        # Save to HDF5
        output_hdf5_file = os.path.join(main_directory, f"{subdir}.hdf5")
        save_hdf5(output_hdf5_file, qpos_data, tau_external_data, cam1_images, cam2_images)
        
        print(f"Saved HDF5 file: {output_hdf5_file}")

