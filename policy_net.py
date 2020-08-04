import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
import torch
import numpy as np
from data_loader import DataLoader
import math
import time
from torch.utils.tensorboard import SummaryWriter

class Net(nn.Module):
    def __init__(self, saved_path=None):
        super(Net, self).__init__()
        self.fc1 =  nn.Linear(24, 200)
        self.fc2 =  nn.Linear(200, 200)
        self.fc4 =  nn.Linear(200, 200)
        self.fc3 = nn.Linear(200, 4)
        if saved_path is not None:
            self.load_weights(saved_path)

    def forward(self, x):
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x = F.tanh(self.fc4(x))
        x = F.tanh(self.fc3(x))
        x = x * 3
        return x

    def load_weights(self, saved_path):
        model_dict = torch.load(saved_path)
        self.load_state_dict(model_dict["model_state_dict"])


def my_loss(input, target):
    loss = ((input-target)**2).mean()
    scale = .1
    min_speed_no_loss = .4
    dif  = min_speed_no_loss - (input**2).sum()
    speed_penalty = scale * F.relu(dif)
    loss = loss - speed_penalty
    return loss

if __name__ == "__main__":
    net = Net()
    print(net)
    writer = SummaryWriter()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    net.to(device)

    opt = optim.Adam(net.parameters(), lr=4e-3)

    criterion = nn.MSELoss()#my_loss
    data_loader = DataLoader(batchSize=320, labelFormat=2, data_dir="./data/runsIK/")
    val_loader = DataLoader(batchSize=320, labelFormat=2, data_dir="./data/val_runsIK/")
    epochs = 4000
    for epoch in np.arange(1, epochs+1):
        lossSum = 0
        points = 0
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
            lossSum += loss * len(x)
            points += len(x)
            t_2 = time.time()
        lossValSum = 0
        pointsVal = 0
        for x, y in iter(val_loader):
            x = Variable(torch.from_numpy(x).float()).to(device)
            y = Variable(torch.from_numpy(y).float()).to(device)
            pred = net(x)
            lossVal = criterion(pred, y)
            lossValSum += lossVal * len(x)
            pointsVal += len(x)

        opt.step()
        t_3 = time.time()
        avgLoss = lossSum / points
        avgLossVal = lossValSum / pointsVal
        #print("Data Load Time {}, Forward Time {}, Backward Time {}, Step Time {}".format(t_1 - t, t_f - t_1, t_2 - t_f, t_3 - t_2))
        if avgLoss < .33:
            opt = optim.Adam(net.parameters(), lr=1e-5)

        print("Epoch {} Loss: {} Val Loss: {}".format(epoch, avgLoss, avgLossVal))
        writer.add_scalar("Train Loss", avgLoss, epoch)
        writer.add_scalar("Val Loss", avgLossVal, epoch)

    torch.save({
    'epoch':epochs,
    'model_state_dict':net.state_dict(),
    'optimizer_state_dict': opt.state_dict(),
    'criterion': criterion
    }, "./saved/trained_model_dict.pt")
    print("Finished Saving Model")
