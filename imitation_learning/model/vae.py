import torch 
import torch.nn as nn

# Define the Convolutional VAE model
class ConvVAE(nn.Module):
    def __init__(self, latent_dim, n_layers):
        super(ConvVAE, self).__init__()
        self.n_layers = n_layers

        self.layers = [16, 32, 64, 128, 256, 512]
        self.lin_mult = [128, 64, 32, 16, 8]
        self.pred_head = [32]
        self.n_layers = n_layers
        self.index = 16

        # Encoder: Convolutional layers
        self.encoder = nn.Sequential(
            nn.Conv2d(3, self.layers[0], kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(self.layers[0]),
            nn.ReLU(),
            nn.Dropout(0.5),

            nn.Conv2d(self.layers[0], self.layers[1], kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(self.layers[1]),
            nn.ReLU(),
            nn.Dropout(0.5),

            nn.Conv2d(self.layers[1], self.layers[2], kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(self.layers[2]),
            nn.ReLU(),

            nn.Conv2d(self.layers[2], self.layers[3], kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(self.layers[3]),
            nn.ReLU(),

            #nn.Conv2d(layers[3], layers[4], kernel_size=3, stride=2, padding=1),
            #nn.BatchNorm2d(layers[4]),
            #nn.ReLU()
        )

        # Fully connected layers for mean and log-variance
        self.fc_mu = nn.Linear(self.layers[3] * self.index * self.index, latent_dim)
        self.fc_logvar = nn.Linear(self.layers[3] * self.index * self.index, latent_dim)

        # Decoder: Fully connected layer to reshape into image format
        self.fc_decode = nn.Linear(latent_dim, self.layers[3] * self.index * self.index)

        # Decoder: Transposed convolutional layers
        self.decoder = nn.Sequential(
            #nn.ConvTranspose2d(layers[4], layers[3], kernel_size=3, stride=2, padding=1, output_padding=1),  # (batch_size, layers[1], 7, 7)
            #nn.BatchNorm2d(layers[3]),
            #nn.ReLU(),
            nn.ConvTranspose2d(self.layers[3], self.layers[2], kernel_size=3, stride=2, padding=1, output_padding=1),  # (batch_size, layers[1], 7, 7)
            nn.BatchNorm2d(self.layers[2]),
            nn.ReLU(),
            nn.ConvTranspose2d(self.layers[2], self.layers[1], kernel_size=3, stride=2, padding=1 , output_padding=1),  # (batch_size, layers[1], 7, 7)
            nn.BatchNorm2d(self.layers[1]),
            nn.ReLU(),
            nn.ConvTranspose2d(self.layers[1], self.layers[0], kernel_size=3, stride=2, padding=1, output_padding=1),   # (batch_size, layers[0], 14, 14)
            nn.BatchNorm2d(self.layers[0]),
            nn.ReLU(),
        )

        self.final_layer = nn.Sequential(
            nn.ConvTranspose2d(self.layers[0], self.layers[0], kernel_size=3, stride=2, padding=1, output_padding=1),    # (batch_size, 1, 28, 28)
            nn.BatchNorm2d(self.layers[0]),
            nn.ReLU(),
            nn.Conv2d(self.layers[0], 3, kernel_size=3, padding=1),  # (batch_size, layers[2], 3, 3)
            nn.Sigmoid()  # Output should be between 0 and 1
        )

    def encode(self, x):
        # Forward pass through encoder
        encoded = self.encoder(x)
        encoded = torch.flatten(encoded, start_dim=1)
        #print(f"encoded flat shape: {encoded.shape}")
        mu = self.fc_mu(encoded)
        #print(f"mu shape: {mu.shape}")
        log_var = self.fc_logvar(encoded)
        #print(f"log var shape: {log_var.shape}")
        return mu, log_var

    def reparameterize(self, mu, log_var):
        # Reparameterization trick
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        return eps * std + mu

    def decode(self, z):
        # Forward pass through decoder
        #print("z shape", z.shape)
        x = self.fc_decode(z).view(-1, self.layers[3], self.index, self.index) 
        #print("x shape", x.shape)
        x = self.decoder(x)
        #print("x shape", x.shape)
        x = self.final_layer(x)
        #print("x shape", x.shape)
        return x

    def forward(self, x):
        # Forward pass through VAE
        #print(x.shape)
        mu, log_var = self.encode(x)
        #print("mu shape:", mu.shape)
        #print("log_var shape:", log_var.shape)
        z = self.reparameterize(mu, log_var)
        #print("z shape:", z.shape)
        #print("self.decode(z) shape:", self.decode(z).shape)
        return self.decode(z), mu, log_var
