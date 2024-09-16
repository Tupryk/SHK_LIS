import cv2 as cv
import numpy as np
import pandas as pd
import random
import os
from PIL import Image

cv.setNumThreads(4)

class AugmentImage:
    @staticmethod
    def normalize(image, mean = [0, 0 , 0], std = [1, 1, 1], max_pixel_val = 255):
        image = image.astype(np.float32)
        image = (image - np.array(mean, dtype=np.float32) * max_pixel_val) / (np.array(std, dtype=np.float32) * max_pixel_val)
        return image

    def posterize(image,  num_bits = 4, p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            levels = 2 ** num_bits
            divisor = 256 // levels
            lut = np.arange(256, dtype=np.uint8) // divisor * divisor
            image = cv.LUT(image, lut)
        return image

    def greyscale(image, p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            image = cv.merge([image, image, image])
        return image

    def blur(image, p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            kernel_width = np.random.randint(3, 7)
            kernel_height = np.random.randint(3, 7)
            kernel_width = kernel_width + 1 if kernel_width % 2 == 0 else kernel_width
            kernel_height = kernel_height + 1 if kernel_height % 2 == 0 else kernel_height
            std_deviation = np.random.uniform(0.2, 1.0)
            image = cv.GaussianBlur(image, (kernel_width, kernel_height), std_deviation)
        return image

    def clahe(image, p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            lab_image = cv.cvtColor(image, cv.COLOR_BGR2LAB)
            l_channel, a_channel, b_channel = cv.split(lab_image)
            
            # Apply CLAHE to the L (lightness) channel
            clahe = cv.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
            l_clahe = clahe.apply(l_channel)
            
            # Merge the CLAHE enhanced L channel back with the original a and b channels
            lab_clahe_image = cv.merge((l_clahe, a_channel, b_channel))
            
            # Convert back to BGR color space
            image = cv.cvtColor(lab_clahe_image, cv.COLOR_LAB2BGR)
        return image


    def gauss_noise(image , mean=0, std=2, p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)
        
        if np.random.rand() <= p:
            # Generate Gaussian noise
            noise = np.random.normal(mean, std, image.shape).astype(np.float32)
    
            # Add the noise to the image and clip the values to stay within [0, 255]
            image = cv.add(image.astype(np.float32), noise)
            image = np.clip(image, 0, 255).astype(np.uint8)
        
        return image

    def hsv_augment(image, hgain = 0.1, sgain=0.9, vgain=0.9, p = 1.0, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            r = np.random.uniform(-1, 1, 3) * [hgain, sgain, vgain] + 1  # random gains
            hue, sat, val = cv.split(cv.cvtColor(image, cv.COLOR_BGR2HSV))
            dtype = image.dtype  # uint8

            x = np.arange(0, 256, dtype=np.int16)
            lut_hue = ((x * r[0]) % 180).astype(dtype)
            lut_sat = np.clip(x * r[1], 0, 255).astype(dtype)
            lut_val = np.clip(x * r[2], 0, 255).astype(dtype)

            image_hsv = cv.merge((cv.LUT(hue, lut_hue), cv.LUT(sat, lut_sat), cv.LUT(val, lut_val))).astype(dtype)
            image = cv.cvtColor(image_hsv, cv.COLOR_HSV2BGR, dst=image)
        return image

    def horizontal_flip(image, p = 0.3, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            image = cv.flip(image, 1)
        return image
    
    def shuffle_channel(image , p = 0.1, seed = None):
        if seed is not None:
            np.random.seed(seed)

        if np.random.rand() <= p:
            b, g, r = cv.split(image)
            # shuffle the channels in a random order
            channel_order = np.random.permutation([0, 1, 2])
            # reorder the channels based on the random order
            image = cv.merge([b, g, r])[..., channel_order]
        return image

    def center_crop(image, crop_height = 360, crop_width=360):
        """
        Crop the center of the image.

        Args:
            image (numpy array): Input image in BGR format (Height, Width, Channels).
            crop_size (tuple): Desired output size (height, width).

        Returns:
            numpy array: Cropped image in BGR format.
        """
        height, width, _ = image.shape

        # Calculate the coordinates of the cropping box
        left = (width - crop_width) // 2
        top = (height - crop_height) // 2
        right = left + crop_width
        bottom = top + crop_height

        # Perform the crop
        image = image[top:bottom, left:right]
        
        return image

    def resize(image, resize_to):
        image = cv.resize(image, (resize_to, resize_to))
        return image

    def rgb_to_bgr(image):
        image = cv.cvtColor(np.array(image), cv.COLOR_RGB2BGR)
        return image
    
    def rgba_to_rgb(image):
        image = cv.cvtColor(np.array(image), cv.COLOR_RGBA2RGB)
        return image
    
        