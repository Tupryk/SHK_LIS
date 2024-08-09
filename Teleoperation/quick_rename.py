import os

# Define the path to the directory you want to process
directory_path = '.'

# Initialize an index for naming
index = 1

# Iterate through each item in the directory
for folder_name in os.listdir(directory_path):
    folder_path = os.path.join(directory_path, folder_name)
    
    # Check if it's a folder and if the name starts with 'recordings_'
    if os.path.isdir(folder_path) and folder_name.startswith('recordings_'):
        # Generate a new unique folder name
        new_folder_name = f'recordings_{index}'
        new_folder_path = os.path.join(directory_path, new_folder_name)

        # Increment the index if the target directory already exists
        while os.path.exists(new_folder_path):
            new_folder_name = f'recordings_{index}'
            new_folder_path = os.path.join(directory_path, new_folder_name)

        # Rename the folder
        os.rename(folder_path, new_folder_path)
        print(f'Renamed "{folder_name}" to "{new_folder_name}"')

        # Increment the index for the next folder
        index += 1

print("Renaming completed.")
