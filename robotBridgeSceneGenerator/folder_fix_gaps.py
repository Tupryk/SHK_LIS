import os
import re

def shift_files(folder_path):
    # Pattern to match files with digits in their names
    pattern = re.compile(r'(.*?)(\d+)(\..+)$')

    # Get all files in the folder
    files = os.listdir(folder_path)

    # Extract and process files with numeric parts
    files_with_numbers = []
    for file in files:
        match = pattern.match(file)
        if match:
            prefix, number, extension = match.groups()
            files_with_numbers.append((file, prefix, int(number), extension))

    # Sort files numerically
    files_with_numbers.sort(key=lambda x: x[2])

    # Rename files to temporary names to avoid overwriting
    temp_files = {}
    for original_file, prefix, number, extension in files_with_numbers:
        temp_name = f"temp_{number}"
        temp_files[temp_name] = (original_file, prefix, extension)
        os.rename(os.path.join(folder_path, original_file), os.path.join(folder_path, temp_name))

    current_number = 0
    for temp_name, (_, prefix, extension) in sorted(temp_files.items(), key=lambda x: int(x[0].split('_')[1])):
        new_name = f"{prefix}{current_number}{extension}"
        os.rename(os.path.join(folder_path, temp_name), os.path.join(folder_path, new_name))
        current_number += 1



folders = ["block_pos", "init_scene_img", "npy_paths", "paths"]

for folder_path in folders:
    shift_files(folder_path)

