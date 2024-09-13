import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader, random_split
from utils.dataloader import CustomImageDataset
from model.vae import ConvVAE
from loss.vae_loss import vae_loss, vae_loss_ssim
import numpy as np 
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity as ssim  # for SSIM
from utils.utils import calculate_psnr
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# Hyperparameters
latent_dim = 32
lr = 1e-3
batch_size = 64
epochs = 2500
n_layers = 3

# Define the transformations
transform = transforms.Compose([
    transforms.Resize((256, 256)),  # Resize to 256x256 (you can change the size)
    transforms.ToTensor(),          # Convert image to tensor
])

# Create the dataset and dataloader
dataset = CustomImageDataset(root_dir='data/scene_images', transform=transform)

# Random seed for reproducibility 
#torch.manual_seed(42)

# Define the size of train and test sets
train_size = int(0.8 * len(dataset))  # 80% for training
test_size = len(dataset) - train_size  # 20% for testing

# Split the dataset into train and test sets
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

# Create DataLoader for both train and test datasets
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# Model, optimizer
model = ConvVAE(latent_dim=latent_dim, n_layers = n_layers).to(device)
optimizer = optim.Adam(model.parameters(), lr=lr)


# Load the checkpoint
checkpoint = torch.load('cpts/vae_2dot5K_kdl1.cpt')

print(f"Pretrained vae model loaded.")

# Load the model state dictionary
model.load_state_dict(checkpoint['model_state_dict'])

# If you're resuming training, load the optimizer state as well
optimizer.load_state_dict(checkpoint['optimizer_state_dict'])

# Evaluate on test data
model.eval()
test_loss, test_recons_loss, test_kdl_loss, test_ssim_loss = 0, 0, 0, 0
mse_recon, ssim_recon, psnr_recon = 0, 0, 0  # Initialize metrics accumulators
with torch.no_grad():
    for data in test_loader:
        data = data.to(device)
        reconstructed_x, mu, log_var = model(data)
        loss, recons_loss, kdl_loss = vae_loss(reconstructed_x, data, mu, log_var, kdl_weight=1.0)
        #loss, recons_loss, kdl_loss, ssim_loss = vae_loss_ssim(reconstructed_x, data, mu, log_var)
        test_loss += loss.item()
        test_recons_loss += recons_loss.item()
        test_kdl_loss += kdl_loss.item()
        #test_ssim_loss += ssim_loss.item()

        # Convert tensors to numpy arrays for metric calculations
        data_np = data.cpu().numpy()
        reconstructed_np = reconstructed_x.cpu().numpy()

        # Calculate MSE for this batch
        mse_recon = nn.MSELoss()(reconstructed_x, data).item()

        # Calculate SSIM for this batch (requires HWC format)
        data_np = data_np.transpose(0, 2, 3, 1)  # Convert to (N, H, W, C)
        reconstructed_np = reconstructed_np.transpose(0, 2, 3, 1)  # Convert to (N, H, W, C)
        # Calculate SSIM for this batch (requires HWC format, setting data_range)
        data_range = 1.0  # Assuming the images are normalized between 0 and 1
        ssim_recon = np.mean([ssim(data_np[i], reconstructed_np[i], win_size=5, data_range=data_range, channel_axis=-1) for i in range(data_np.shape[0])])

        # Calculate PSNR for this batch
        psnr_recon = np.mean([calculate_psnr(data_np[i], reconstructed_np[i]) for i in range(data_np.shape[0])])

    print(f'Test Loss: {test_loss:.4f}, Recons Loss: {test_recons_loss:.4f}, KDL Loss: {test_kdl_loss:.4f}, SSIM Loss: {test_ssim_loss:.4f}')
    print(f'Reconstruction Metrics: MSE: {mse_recon:.4f}, SSIM: {ssim_recon:.4f}, PSNR: {psnr_recon:.4f}')

# Generate new images
with torch.no_grad():
    n_samples = 4
    z = torch.randn(n_samples, latent_dim).to(device)
    samples = model.decode(z).view(-1, 3, 256, 256)  # Adjusted to correct image size

# Plot the generated samples
plt.figure(figsize=(n_samples*2, n_samples*2))
for i in range(n_samples):
    plt.subplot(n_samples//2, n_samples//2, i + 1)
    image = samples[i].cpu().numpy().transpose(1, 2, 0)  # Convert to (H, W, C) format
    plt.imshow(image)
    plt.axis('off')
plt.show()