import torch
import torch.nn as nn 
from torchmetrics.functional import structural_similarity_index_measure as ssim

# Loss function
def vae_loss(reconstructed_x, x, mu, log_var, kdl_weight = 0.00025):
    # Reconstruction loss (binary cross entropy)
    recons_loss = nn.functional.mse_loss(reconstructed_x, x)

    # KL divergence loss
    kld_loss = torch.mean(-0.5 * torch.sum(1 + log_var - mu ** 2 - log_var.exp(), dim = 1), dim = 0)
    
    loss = recons_loss + kdl_weight * kld_loss

    return loss, recons_loss, kld_loss


# Initialize SSIM (data_range is the difference between max and min pixel values)
#ssim_module = SSIM(data_range=1.0)

# Loss function with SSIM (using torchmetrics for RGB images)
def vae_loss_ssim(reconstructed_x, x, mu, log_var, kdl_weight=0.00025, ssim_weight=0.1):
    # Reconstruction loss (MSE)
    recons_loss = nn.functional.mse_loss(reconstructed_x, x)

    # KL divergence loss
    kld_loss = torch.mean(-0.5 * torch.sum(1 + log_var - mu ** 2 - log_var.exp(), dim=1), dim=0)

    # SSIM loss (1 - SSIM since SSIM is a similarity measure)
    ssim_loss = ssim_loss = 1 - ssim(reconstructed_x, x)

    # Total loss: weighted sum of reconstruction loss, SSIM loss, and KL divergence
    loss = recons_loss + kdl_weight * kld_loss + ssim_weight * ssim_loss

    return loss, recons_loss, kld_loss, ssim_loss