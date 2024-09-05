import torch 
import torch.nn as nn

# Define the Convolutional VAE model
class ConvVAE(nn.Module):
    def __init__(self, latent_dim):
        super(ConvVAE, self).__init__()

        # Encoder: Convolutional layers
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=4, stride=2, padding=1),  # (batch_size, 32, 14, 14)
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1),  # (batch_size, 64, 7, 7)
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1),  # (batch_size, 128, 3, 3)
            nn.ReLU(),
            nn.Conv2d(128, 256, kernel_size=4, stride=2, padding=1),  # (batch_size, 128, 3, 3)
            nn.ReLU(),
            nn.Flatten(),  # Flatten to (batch_size, 128*3*3)
        )

        # Fully connected layers for mean and log-variance
        self.fc_mu = nn.Linear(256 * 16 * 16, latent_dim)
        self.fc_logvar = nn.Linear(256 * 16 * 16, latent_dim)

        # Decoder: Fully connected layer to reshape into image format
        self.fc_decode = nn.Linear(latent_dim, 256 * 16 * 16)

        # Decoder: Transposed convolutional layers
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(256, 128, kernel_size=4, stride=2, padding=1),  # (batch_size, 64, 7, 7)
            nn.ReLU(),
            nn.ConvTranspose2d(128, 64, kernel_size=4, stride=2, padding=1),  # (batch_size, 64, 7, 7)
            nn.ReLU(),
            nn.ConvTranspose2d(64, 32, kernel_size=4, stride=2, padding=1),   # (batch_size, 32, 14, 14)
            nn.ReLU(),
            nn.ConvTranspose2d(32, 3, kernel_size=4, stride=2, padding=1),    # (batch_size, 1, 28, 28)
            nn.Sigmoid()  # Output should be between 0 and 1
        )

    def encode(self, x):
        # Forward pass through encoder
        #print("Encoder x:", x.shape)
        encoded = self.encoder(x)
        #print("Encoder encoded:", encoded.shape)
        mu = self.fc_mu(encoded)
        #print("Encoder mu:", mu.shape)
        log_var = self.fc_logvar(encoded)
        #print("Encoder logvar:", log_var.shape)

        return mu, log_var

    def reparameterize(self, mu, log_var):
        # Reparameterization trick
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, z):
        # Forward pass through decoder
        x = self.fc_decode(z).view(-1, 256, 16, 16)  # Reshape to the size for convolutional decoder
        return self.decoder(x)

    def forward(self, x):
        # Forward pass through VAE
        mu, log_var = self.encode(x)
        z = self.reparameterize(mu, log_var)
        return self.decode(z), mu, log_var
