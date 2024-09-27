import cv2 as cv
import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image
import random
import pandas as pd
from utils.augmentations import AugmentImage as apply
cv.setNumThreads(4)

def augment_data(image,
                  image_size = 256,
                  p_hsv = 1.0, hgain = 0.015, sgain=0.2, vgain=0.2,
                  p_grey = 0.0,
                  p_blur = 0.0,
                  p_clahe = 0.2,
                  p_shuffle = 0.2,
                  p_post = 0.0,
                  p_gauss = 0.2,
                  mode = 'test',
                  seed = None):
    """
    Apply various transformations to an input image.

    Args:

    Returns:
        np.ndarray: Transformed image.
    """

    if mode == 'train':
        # convert rgba to rgb
        image = apply.rgba_to_rgb(image)
        # convert rgb to bgr
        image = apply.rgb_to_bgr(image)

        image = apply.center_crop(image, crop_height=250, crop_width=380)

        # resize image
        image = apply.resize(image, image_size)

        height, width = image.shape[:2]
        
        #image = apply.posterize(image, p = p_post, seed = seed)

        #image = apply.greyscale(image, p = p_grey, seed = seed)

        #image = apply.blur(image, p = p_blur, seed = seed)

        image = apply.clahe(image, p = p_clahe, seed = seed)

        image = apply.hsv_augment(image, hgain = hgain, sgain = sgain, vgain = vgain, p = p_hsv, seed = seed)

        image = apply.gauss_noise(image, mean=0, std = 2, p = p_gauss)

        image = apply.shuffle_channel(image, p = p_shuffle, seed = seed)
        

    elif mode == 'test':
        # convert rgb to bgr
        image = apply.rgb_to_bgr(image)
        image = apply.center_crop(image, crop_height=250, crop_width=380)
        # resize image
        image = apply.resize(image, image_size)
        height, width = image.shape[:2]
        # conver to opencv format for augmentations

    image = apply.resize(image, width)
    # convert brg to rgb
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    # normalise # consistent with ImageNet backbone
    image = apply.normalize(image) #, mean = [0.485, 0.456, 0.406], std = [0.229, 0.244, 0.255])


    return image