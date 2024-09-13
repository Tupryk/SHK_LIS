import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader, random_split
import matplotlib.pyplot as plt
from utils.dataloader import CustomImageDataset
from model.vae import ConvVAE
from loss.vae_loss import vae_loss, vae_loss_ssim
import numpy as np 

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Device Name: {torch.cuda.get_device_name(device)}" if device.type == "cuda" else "cpu")

# Hyperparameters
latent_dim = 32
lr = 1e-3
batch_size = 64
epochs = 5000
n_layers = 3
# Define the transformations
transform = transforms.Compose([
    transforms.Resize((256, 256)),  # Resize to 256x256 (you can change the size)
    #transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
    #transforms.RandomGrayscale(p=0.2),
    #transforms.GaussianBlur(kernel_size=(5, 9), sigma=(0.1, 5)),
    transforms.ToTensor(),          # Convert image to tensor
])

# Create the dataset and dataloader
dataset = CustomImageDataset(root_dir='data/scene_images', transform=transform)

# Random seed for reproducibility 
torch.manual_seed(42)

# Define the size of train and test sets
train_size = int(0.8 * len(dataset))  # 80% for training
test_size = len(dataset) - train_size  # 20% for testing

# Split the dataset into train and test sets
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

# Create DataLoader for both train and test datasets
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True , num_workers=2)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False, num_workers=2)

# Model, optimizer
model = ConvVAE(latent_dim=latent_dim, n_layers = n_layers).to(device)
print(f"Model: {model}")
optimizer = optim.Adam(model.parameters(), lr=lr)

scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer,
        max_lr=lr,
        epochs=epochs,
        steps_per_epoch=
            len(train_loader),
        anneal_strategy="cos",
    )


# Training loop
model.train()
for epoch in range(epochs):
    train_loss, train_recons_loss, train_kdl_loss, train_ssim_loss = 0, 0, 0, 0
    for data in train_loader:  # Removed label handling as there is none
        data = data.to(device)
        optimizer.zero_grad()
        reconstructed_x, mu, log_var = model(data)
        loss, recons_loss, kdl_loss = vae_loss(reconstructed_x, data, mu, log_var, kdl_weight = 1.0)
        #loss, recons_loss, kdl_loss, ssim_loss = vae_loss_ssim(reconstructed_x, data, mu, log_var, kdl_weight = 0.00025, ssim_weight = 0.1)
        loss.backward()
        optimizer.step()
        scheduler.step()
        train_loss += loss.item()
        train_recons_loss += recons_loss.item()
        train_kdl_loss += kdl_loss.item()
        #train_ssim_loss += ssim_loss.item()
    print(f'Epoch {epoch + 1}, Loss: {train_loss:.4f}, Recons Loss: {train_recons_loss:.4f}, KDL Loss: {train_kdl_loss:.4f}, SSIM Loss: {train_ssim_loss:.4f}')
print(f'Finished Variation AutoEncoder.')

# Define parameters to store in checkpoints cpts
checkpoint = {
    'model_state_dict': model.state_dict(),    # Model parameters
    'optimizer_state_dict': optimizer.state_dict(),  # Optimizer parameters
    'epoch': epoch,  # You can also store other info like epoch, loss, etc.
}

# Save the checkpoint as 'vae.cpt'
torch.save(checkpoint, 'cpts/vae_2dot5K_kdl1.cpt')
print(f'Model checkpoint stored.')

