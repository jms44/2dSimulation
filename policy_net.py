import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
import torch
import numpy as np
from data_loader import DataLoader
import math

class Net(nn.Module):
    def __init__(self, saved_path=None):
        super(Net, self).__init__()
        self.fc1 =  nn.Linear(19, 50)
        self.fc2 =  nn.Linear(50, 50)
        self.fc3 = nn.Linear(50, 5)
        if saved_path is not None:
            self.load_weights(saved_path)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    def load_weights(self, saved_path):
        model_dict = torch.load(saved_path)
        self.load_state_dict(model_dict["model_state_dict"])


if __name__ == "__main__":
    net = Net()
    print(net)
    opt = optim.SGD(net.parameters(), lr=1e-4)

    criterion = nn.MSELoss()

    data_loader = DataLoader(batchSize=12)
    epochs = 10000
    for epoch in np.arange(1, epochs+1):
        lossSum = 0
        batchNum = 0
        opt.zero_grad()
        for x, y in iter(data_loader):
            x = Variable(torch.from_numpy(x).float())
            y = Variable(torch.from_numpy(y).float())
            pred = net(x)
            loss = criterion(pred, y)
            loss.backward()
            lossSum += loss
            batchNum += 1
        opt.step()
        avgLoss = lossSum / batchNum
        print("Epoch {} Loss: {}".format(epoch, avgLoss))
    torch.save({
    'epoch':epochs,
    'model_state_dict':net.state_dict(),
    'optimizer_state_dict': opt.state_dict(),
    'criterion': criterion
    }, "./saved/trained_model_dict.pt")
    print("Finished Saving Model")
