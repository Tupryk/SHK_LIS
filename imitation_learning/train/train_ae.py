import torch
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import TensorDataset, DataLoader, random_split
from model.ae import Autoencoder

IMAGE_DIMS = 128
np_data = np.load("./data/arrays.npz")["arr_0"]
# device = "cuda"
print(torch.cuda.is_available())
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Device Name: {torch.cuda.get_device_name(device)}" if device.type == "cuda" else "CPU")
dataset = torch.utils.data.TensorDataset(torch.Tensor(np_data))
loader = torch.utils.data.DataLoader(dataset, batch_size=50, shuffle=True)

# Create the dataset
dataset = TensorDataset(torch.Tensor(np_data))

# Split the dataset into training and validation sets
train_size = int(0.9 * len(dataset))
val_size = len(dataset) - train_size
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

# Create data loaders
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)


def train(nepochs: int = 10, save_as: str = "autoencoder"):
    model = Autoencoder()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    model.to(device)
    train_losses = []
    val_losses = []

    for epoch in range(nepochs):
        # Validation step
        model.eval()  # Set model to evaluation mode
        val_loss = 0
        with torch.no_grad():
            for [data] in val_loader:
                data = data.to(device)
                out = model(data)
                val_loss += torch.mean((data - out) ** 2).item()

        val_loss /= len(val_loader)
        val_losses.append(val_loss)

        model.train()  # Set model to training mode
        for [data] in train_loader:
            data = data.to(device)
            optimizer.zero_grad()

            out = model(data)
            loss = torch.mean((data - out) ** 2)
            train_losses.append(loss.item())

            # Backward pass and optimization
            loss.backward()
            optimizer.step()


        # Print losses for this epoch
        print(f"Epoch {epoch + 1},\t Train Loss: {np.mean(train_losses):.6f},\t Val Loss: {val_loss:.6f}")
        train_losses = []  # Reset training losses after each epoch

        if (epoch+1)%100 == 0:
            torch.save(model.state_dict(), f"./models/{save_as}{epoch+1}.pth")
    # Save the trained model
    torch.save(model.state_dict(), f"./models/{save_as}.pth")
    return model


if __name__ == "__main__":
    save_as = "low_dim_au"
    model = train(2000, save_as)
    # model = Autoencoder().to(device)
    # model.load_state_dict(torch.load('./models/autoencoder.pth'))

    # See results
    image = np.array(val_dataset[np.random.randint(0, len(val_dataset))][0])
    print(image.shape)
    y = model.forward(torch.Tensor([image]).to(device))

    image = np.transpose(image, (1, 2, 0))
    y = y.detach().cpu().numpy()
    y = y[0]
    y = np.maximum(y, 0)
    y = np.transpose(y, (1, 2, 0))

    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    ax[0].imshow(image)
    ax[0].axis('off')
    ax[1].imshow(y)
    ax[1].axis('off')

    plt.tight_layout()
    plt.show()