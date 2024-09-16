import os
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms
from utils.augmentations import AugmentImage
from utils.transform import augment_data

# Define a custom dataset class
class CustomImageDataset(Dataset):
    def __init__(self, root_dir, image_size = 256, mode = "test"):
        """
        Args:
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied on an image.
        """
        self.root_dir = root_dir
        self.mode = mode
        self.image_size = 256
        self.image_files = [f for f in os.listdir(root_dir) if os.path.isfile(os.path.join(root_dir, f))]

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        img_path = os.path.join(self.root_dir, self.image_files[idx])
        image = Image.open(img_path).convert("RGB") 
        if self.mode == "train":
            image = augment_data(image, image_size = self.image_size, mode = self.mode)
            image = np.transpose(image, (2, 0, 1))
            #image = np.transpose(image, (1, 2, 0))  # From (Channels, Height, Width) to (Height, Width, Channels)
            #plt.imshow(image)
            #plt.axis('off')  # Hide the axes
            #plt.show()

        elif self.mode == "test":
            image = augment_data(image, image_size = self.image_size, mode = self.mode)
            image = np.transpose(image, (2, 0, 1))

        return image

