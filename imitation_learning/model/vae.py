import torch 
import torch.nn as nn

# Define the Convolutional VAE model
class ConvVAE(nn.Module):
    def __init__(self, latent_dim):
        super(ConvVAE, self).__init__()

        # Encoder: Convolutional layers
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.LeakyReLU(),

            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.LeakyReLU(),

            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.LeakyReLU(),

            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.LeakyReLU(),

            nn.Conv2d(256, 512, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(512),
            nn.LeakyReLU()
        )

        # Fully connected layers for mean and log-variance
        self.fc_mu = nn.Linear(512 * 8 * 8, latent_dim)
        self.fc_logvar = nn.Linear(512 * 8 * 8, latent_dim)

        # Decoder: Fully connected layer to reshape into image format
        self.fc_decode = nn.Linear(latent_dim, 512 * 8 * 8)

        # Decoder: Transposed convolutional layers
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(512, 256, kernel_size=3, stride=2, padding=1, output_padding=1),  # (batch_size, 64, 7, 7)
            nn.BatchNorm2d(256),
            nn.LeakyReLU(),
            nn.ConvTranspose2d(256, 128, kernel_size=3, stride=2, padding=1, output_padding=1),  # (batch_size, 64, 7, 7)
            nn.BatchNorm2d(128),
            nn.LeakyReLU(),
            nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1 , output_padding=1),  # (batch_size, 64, 7, 7)
            nn.BatchNorm2d(64),
            nn.LeakyReLU(),
            nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, output_padding=1),   # (batch_size, 32, 14, 14)
            nn.BatchNorm2d(32),
            nn.LeakyReLU(),
        )

        self.final_layer = nn.Sequential(
            nn.ConvTranspose2d(32, 32, kernel_size=3, stride=2, padding=1, output_padding=1),    # (batch_size, 1, 28, 28)
            nn.BatchNorm2d(32),
            nn.LeakyReLU(),
            nn.Conv2d(32, 3, kernel_size=3, padding=1),  # (batch_size, 128, 3, 3)
            nn.Sigmoid()  # Output should be between 0 and 1
        )

    def encode(self, x):
        # Forward pass through encoder
        #print("Encoder x:", x.shape)
        encoded = self.encoder(x)
        encoded = torch.flatten(encoded, start_dim=1)
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
        x = self.fc_decode(z).view(-1, 512, 8, 8)  # Reshape to the size for convolutional decoder
        x = self.decoder(x)
        x = self.final_layer(x)
        return x

    def forward(self, x):
        # Forward pass through VAE
        mu, log_var = self.encode(x)
        z = self.reparameterize(mu, log_var)
        return self.decode(z), mu, log_var
