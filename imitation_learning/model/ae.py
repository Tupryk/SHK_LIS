import torch
import torch.nn as nn


class Autoencoder(nn.Module):
    def __init__(self, input_dim=256):
        super(Autoencoder, self).__init__()
        
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=2, padding=1),  # [batch, 16, input_dim/2, input_dim/2]
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),  # [batch, 32, input_dim/4, input_dim/4]
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),  # [batch, 64, input_dim/8, input_dim/8]
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),  # [batch, 128, input_dim/16, input_dim/16]
            nn.ReLU()
        )

        # Calculate the flattened size
        flattened_size = 128 * (input_dim // 16) * (input_dim // 16)
        print(flattened_size)
        
        # Flatten and reduce to 6D
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(flattened_size, 6)

        # Decoder
        self.fc2 = nn.Linear(6, flattened_size)
        self.unflatten = nn.Unflatten(1, (128, input_dim // 16, input_dim // 16))
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1, output_padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1, output_padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(32, 16, kernel_size=3, stride=2, padding=1, output_padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(16, 3, kernel_size=3, stride=2, padding=1, output_padding=1),
            nn.Sigmoid()
        )
    
    def forward(self, x):
        # Encode to 6-dimensional vector
        encoded = self.encoder(x)
        encoded = self.flatten(encoded)
        encoded = self.fc1(encoded)
        
        # Decode back to the original image size
        decoded = self.fc2(encoded)
        decoded = self.unflatten(decoded)
        decoded = self.decoder(decoded)
        return decoded
    

class Diffuser(nn.Module):
    def __init__(self, hidden_count: int = 3, hidden_dim: int = 256):
        super().__init__()
        input_dim = 7 * 96 # joint_dim * example_timestamps
        layers = [nn.Linear(input_dim + 1, hidden_dim)] # input_dim + denoising_step
        for _ in range(hidden_count):
            layers.append(nn.Linear(hidden_dim, hidden_dim))
        layers.append(nn.Linear(hidden_dim, input_dim))
        self.linears = nn.ModuleList(layers)

        # init using kaiming
        for layer in self.linears:
            nn.init.kaiming_uniform_(layer.weight)

    def forward(self, x, t):
        x = torch.concat([x, t], axis=-1)
        for l in self.linears[:-1]:
            x = nn.ReLU()(l(x))
        return self.linears[-1](x)
