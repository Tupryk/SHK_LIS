import numpy as np

# Functions for PSNR and SSIM calculation
def calculate_psnr(img1, img2):
    mse_value = np.mean((img1 - img2) ** 2)
    if mse_value == 0:  # MSE is zero means no noise is present.
        return float('inf')
    pixel_max = 1.0  # Assuming images are normalized between [0, 1]
    return 20 * np.log10(pixel_max / np.sqrt(mse_value))