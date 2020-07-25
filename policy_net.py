import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
import torch
import numpy as np
from data_loader import DataLoader
import math
import time

class Net(nn.Module):
    def __init__(self, saved_path=None):
        super(Net, self).__init__()
        self.fc1 =  nn.Linear(23, 200)
        self.fc2 =  nn.Linear(200, 200)
        self.fc4 =  nn.Linear(200, 200)
        self.fc5 =  nn.Linear(200, 200)

        self.fc3 = nn.Linear(200, 5)
        if saved_path is not None:
            self.load_weights(saved_path)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))

        x = self.fc3(x)
        return x

    def load_weights(self, saved_path):
        model_dict = torch.load(saved_path)
        self.load_state_dict(model_dict["model_state_dict"])


if __name__ == "__main__":
    net = Net()
    print(net)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    net.to(device)

    opt = optim.SGD(net.parameters(), lr=1e-1)

    criterion = nn.MSELoss()
    data_loader = DataLoader(batchSize=32, labelFormat=1, data_dir="./data/overfit/")
    epochs = 10000
    for epoch in np.arange(1, epochs+1):
        lossSum = 0
        batchNum = 0
        opt.zero_grad()
        for x, y in iter(data_loader):
            t = time.time()
            x = Variable(torch.from_numpy(x).float()).to(device)
            y = Variable(torch.from_numpy(y).float()).to(device)
            t_1 = time.time()
            pred = net(x)
            t_f = time.time()
            loss = criterion(pred, y)
            loss.backward()
            lossSum += loss
            batchNum += 1
            t_2 = time.time()

        opt.step()
        t_3 = time.time()
        avgLoss = lossSum / batchNum
        #print("Data Load Time {}, Forward Time {}, Backward Time {}, Step Time {}".format(t_1 - t, t_f - t_1, t_2 - t_f, t_3 - t_2))
        if avgLoss < .3:
            opt = optim.SGD(net.parameters(), lr=4e-5)

        print("Epoch {} Loss: {}".format(epoch, avgLoss))
    torch.save({
    'epoch':epochs,
    'model_state_dict':net.state_dict(),
    'optimizer_state_dict': opt.state_dict(),
    'criterion': criterion
    }, "./saved/trained_model_dict.pt")
    print("Finished Saving Model")
