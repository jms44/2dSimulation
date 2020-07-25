import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
from arm import Arm
from door import Door
from policy_net import Net
from data_loader import DataLoader
from torch.autograd import Variable
import torch
import json
import glob
import random
class ArmSim (Framework):
    name = "Arm Simulation"

    def __reset(self):
        Framework.__init__(self)
        Framework.setZoom(self, 115)
        Framework.setCenter(self, [0, 1.74])
        self.wristAngle = math.pi/2
        self.doorOpened = False
        self.recording = False
        self.reset = False
        self.arm_angles = [random.uniform(math.pi/3, 2*math.pi/3), random.uniform(math.pi/3, 2*math.pi/3), math.pi/2, math.pi/2, math.pi/2]
        lengths = [1, 1, .4, .27]
        self.arm = Arm(self.world, self.arm_angles, lengths, basePos = (-.1, 0.08), motorLimit=4*math.pi/2)
        ground = self.world.CreateBody(position=(0, 0))
        ground.CreateEdgeChain([(-4, -2), (-4, .6), (-2, .6),(-2, 0),(2, 0),(2, 6),(4, 6),(4, -2), (-2, -2)])
        self.model = Net(saved_path = "./saved/trained_model_dict.pt")
        self.door = Door(self.world, (-2, .62), 1.2)


    def __init__(self):
        self.__reset()

    def Step(self, settings):
        if not self.doorOpened:
            settings.positionIterations = 50
            settings.velocityIterations = 250
            Framework.Step(self, settings)

            if self.door.joint.angle < -math.pi/3:
                self.doorOpened = True
                print("Door Successfully Opened")

            x = DataLoader.get_observation(self.arm, self.door, labelFormat = 0)[0:23]

            with torch.no_grad():
                inp = torch.from_numpy(x)
                print(len(inp))
                pred = self.model(Variable(inp).float())
                pred = DataLoader.output_to_dict(pred)

            targetAngles = pred['targetSpeeds'].data.numpy().astype('float')
            print(targetAngles)
            self.arm.set_target_angles(targetAngles)
            self.arm.update_arm()
            #self.arm.set_motor_speeds(targetSpeeds)

        if self.reset:
            self.__reset()

    def Keyboard(self, key):
        if key == Keys.K_d:
            self.reset = True


    def MouseDown(self, p):
        pass

    def KeyboardUp(self, key):
        pass
if __name__ == "__main__":
    main(ArmSim)
