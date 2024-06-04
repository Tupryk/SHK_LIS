import pandas as pd
import numpy as np

# Read the CSV file
df = pd.read_csv('red_dot_coordinates.csv')

# Extract the X and Y columns
image_points = df[['X', 'Y']].values.astype(np.float32)

print(image_points)
