import torch
import torch.nn as nn
import torch.nn.functional as F

class ConvNet(nn.Module):
    def __init__(self):
        super(ConvNet, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=16, kernel_size=3, stride=1, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.conv2 = nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, stride=1, padding=1)
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        # Fully connected layer
        self.fc = nn.Linear(32 * 7 * 7, 10)  # Assuming input image size is 28x28 and 10 output classes

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool2(x)

        # Flatten the output from the convolutional layers
        x = x.view(-1, 32 * 7 * 7)

        x = self.fc(x)
        x = F.softmax(x, dim=1)
        return x

model = ConvNet()
print(model)
