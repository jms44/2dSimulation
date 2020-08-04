import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
from arm import Arm
from string import ascii_lowercase
from door import Door

class ArmSim (Framework):
    name = "Arm Simulation"
    alphabet = [False] * 26


    def __init__(self):
        Framework.__init__(self)
        self.wristAngle = math.pi/2
        self.doorOpened = False

        self.arm_angles = [math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2]
        lengths = [1, 1, .4, .27]
        self.arm = Arm(self.world, self.arm_angles, lengths, basePos = (-.1, 0.08), motorRange=2*math.pi)
        ground = self.world.CreateBody(position=(0, 0))
        ground.CreateEdgeChain([(-4, -2), (-4, .6), (-2, .6),(-2, 0),(2, 0),(2, 6),(4, 6),(4, -2), (-2, -2)])

        self.door = Door(self.world, (-2, .62), 1.2)

        box = self.world.CreateDynamicBody(position = (1, 2))
        box.CreatePolygonFixture(box=(.1, .1), density=0.3, friction=0.5)

    def Step(self, settings):

        settings.positionIterations = 50
        settings.velocityIterations = 250
        if self.door.joint.angle < -math.pi/3:
            self.doorOpened = True
            print("Door Successfully Opened")
        Framework.Step(self, settings)
        self.update_keyboard_angles()
        self.arm.update_arm(target_pose_mode = True)
        renderer = self.renderer

    def update_keyboard_angles(self):
        angleDif = math.pi/120
        mid = (math.pi/2)
        max = mid+self.arm.motorRange/2
        min = mid-self.arm.motorRange/2
        clawMin = mid-self.arm.motorRange/8
        clawMax = mid+self.arm.motorRange/8

        if self.alphabet[ascii_lowercase.index('q')] and self.arm_angles[0] <= max:
            self.arm_angles[0] += angleDif
        if self.alphabet[ascii_lowercase.index('w')] and self.arm_angles[0] >= min:
            self.arm_angles[0] -= angleDif
        if self.alphabet[ascii_lowercase.index('e')] and self.arm_angles[1] <= max:
            self.arm_angles[1] += angleDif
        if self.alphabet[ascii_lowercase.index('r')] and self.arm_angles[1] >= min:
            self.arm_angles[1] -= angleDif
        if self.alphabet[ascii_lowercase.index('u')] and self.arm_angles[2] <= max:
            self.arm_angles[2] += angleDif
        if self.alphabet[ascii_lowercase.index('i')] and self.arm_angles[2] >= min:
            self.arm_angles[2] -= angleDif
        if self.alphabet[ascii_lowercase.index('o')] and self.arm_angles[3] <= clawMax:
            self.arm_angles[3] += angleDif
        if self.alphabet[ascii_lowercase.index('p')]  and self.arm_angles[3] >= clawMin:
            self.arm_angles[3] -= angleDif

        if self.alphabet[ascii_lowercase.index('k')]:
            self.wristAngle += angleDif
        if self.alphabet[ascii_lowercase.index('l')]:
            self.wristAngle -= angleDif
        p = self.arm.get_target_pose()
        self.arm_angles[-1] = math.pi - self.arm_angles[-2]
        self.arm.target_pose = [p[0], p[1], self.wristAngle, self.arm_angles[-2], self.arm_angles[-1]]


    def Keyboard(self, key):
        for letter in ascii_lowercase:
            if key == eval("Keys.K_" + letter):
                self.alphabet[ascii_lowercase.index(letter)] = True

    def MouseDown(self, p):
        self.arm.target_pose[0] = p[0]
        self.arm.target_pose[1] = p[1]

    def KeyboardUp(self, key):
        for letter in ascii_lowercase:
            if key == eval("Keys.K_" + letter):
                self.alphabet[ascii_lowercase.index(letter)] = False
if __name__ == "__main__":
    main(ArmSim)
