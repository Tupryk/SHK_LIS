import torch
import torch.nn as nn 

# Loss function
def vae_loss(reconstructed_x, x, mu, log_var, kdl_weight = 0.00025):
    # Reconstruction loss (binary cross entropy)
    recons_loss = nn.functional.mse_loss(reconstructed_x, x)

    # KL divergence loss
    kld_loss = torch.mean(-0.5 * torch.sum(1 + log_var - mu ** 2 - log_var.exp(), dim = 1), dim = 0)
    
    loss = recons_loss + kdl_weight * kld_loss

    return loss, recons_loss, kld_loss
