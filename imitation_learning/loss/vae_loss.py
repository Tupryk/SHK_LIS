import torch
import torch.nn as nn 

# Loss function
def vae_loss(reconstructed_x, x, mu, log_var):
    # Reconstruction loss (binary cross entropy)
    recon_loss = nn.functional.binary_cross_entropy(reconstructed_x, x, reduction='sum')

    # KL divergence loss
    kl_loss = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp())

    return recon_loss + kl_loss
