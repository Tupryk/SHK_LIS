import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader, random_split
import matplotlib.pyplot as plt
from utils.dataloader import CustomImageDataset
from model.vae import ConvVAE
from loss.vae_loss import vae_loss
import numpy as np 

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Device Name: {torch.cuda.get_device_name(device)}" if device.type == "cuda" else "cpu")

# Hyperparameters
latent_dim = 128
lr = 1e-4
batch_size = 64
epochs = 10

# Define the transformations
transform = transforms.Compose([
    transforms.Resize((256, 256)),  # Resize to 256x256 (you can change the size)
    transforms.ToTensor(),          # Convert image to tensor
])

# Create the dataset and dataloader
dataset = CustomImageDataset(root_dir='data/scene_images', transform=transform)
data_loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

# Define the size of train and test sets
train_size = int(0.8 * len(dataset))  # 80% for training
test_size = len(dataset) - train_size  # 20% for testing

# Split the dataset into train and test sets
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

# Create DataLoader for both train and test datasets
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# Model, optimizer
vae = ConvVAE(latent_dim=latent_dim).to(device)
optimizer = optim.Adam(vae.parameters(), lr=lr)

scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer,
        max_lr=lr,
        epochs=epochs,
        steps_per_epoch=
            len(train_loader),
        anneal_strategy="cos",
    )


# Training loop
vae.train()
for epoch in range(epochs):
    train_loss = 0
    for data in train_loader:  # Removed label handling as there is none
        data = data.to(device)
        optimizer.zero_grad()
        reconstructed_x, mu, log_var = vae(data)
        loss = vae_loss(reconstructed_x, data, mu, log_var)
        loss.backward()
        optimizer.step()
        scheduler.step()
        train_loss += loss.item()
    print(f'Epoch {epoch + 1}, Loss: {train_loss / len(train_loader.dataset):.4f}')

# Evaluate on test data
vae.eval()
test_loss = 0
with torch.no_grad():
    for data in test_loader:
        data = data.to(device)
        reconstructed_x, mu, log_var = vae(data)
        loss = vae_loss(reconstructed_x, data, mu, log_var)
        test_loss += loss.item()
print(f'Test Loss: {test_loss / len(test_loader.dataset):.4f}')

# Generate new images
with torch.no_grad():
    z = torch.randn(16, latent_dim).to(device)
    samples = vae.decode(z).view(-1, 3, 256, 256)  # Adjusted to correct image size

# Plot the generated samples
plt.figure(figsize=(4, 4))
for i in range(16):
    plt.subplot(4, 4, i + 1)
    #print("samples[i][0] shape:", samples[i].shape)
    image = samples[i].cpu().numpy().transpose(1, 2, 0)  # Convert to (H, W, C) format
    print(image)
    image = (image * 255).astype(np.uint8)
    plt.imshow(image)
    plt.axis('off')
plt.show()
