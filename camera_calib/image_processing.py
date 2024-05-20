import cv2
import numpy as np
import os
import csv

# Directory containing the images
image_dir = 'photos'
image_prefix = 'image'
image_extension = '.png'
output_file = 'red_dot_coordinates.csv'

# Define the range of the red color in HSV
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])

# Function to find the coordinates of the red dot
def find_red_dot_coordinates(image):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create a mask for the red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Compute the center of the contour
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy)
    return None

# Open the CSV file for writing
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Image', 'X', 'Y'])  # Write the header

    # Process each image
    for i in range(10):
        image_path = os.path.join(image_dir, f'{image_prefix}{i}{image_extension}')
        image = cv2.imread(image_path)
        
        if image is not None:
            coordinates = find_red_dot_coordinates(image)
            if coordinates:
                writer.writerow([f'{image_prefix}{i}{image_extension}', coordinates[0], coordinates[1]])
                print(f'Image {i}: Red dot coordinates are {coordinates}')
            else:
                writer.writerow([f'{image_prefix}{i}{image_extension}', 'Not found', 'Not found'])
                print(f'Image {i}: Red dot not found')
        else:
            writer.writerow([f'{image_prefix}{i}{image_extension}', 'Image not found', 'Image not found'])
            print(f'Image {i}: Image not found')

print(f'Coordinates saved to {output_file}')

# Optional: Display the image with the red dot marked (for verification)
# Uncomment below code to visualize
for i in range(10):
    image_path = os.path.join(image_dir, f'{image_prefix}{i}{image_extension}')
    image = cv2.imread(image_path)
    if image is not None:
        coordinates = find_red_dot_coordinates(image)
        if coordinates:
            cv2.circle(image, coordinates, 5, (0, 255, 0), -1)
        cv2.imshow(f'Image {i}', image)
cv2.waitKey(0)
cv2.destroyAllWindows()