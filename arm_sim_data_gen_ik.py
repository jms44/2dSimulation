import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
from arm import Arm
from door import Door
import numpy as np
from string import ascii_lowercase
import random
import json
import glob



class SimDataGen:
    def __init__(self, dataDir):
        self.dataDir = dataDir
        names = glob.glob(dataDir + "*")
        nums = [int(name[name.rfind('/')+1:-5]) for name in names]
        self.runNum = 0
        if len(nums) > 0:
            self.runNum = max(nums) + 1
        self.newFile()

    def newFile(self):
        self.jsonData = []

    def closeFile(self):
        fileName = self.dataDir + str(self.runNum).zfill(9) + '.json'
        jsonDict = {'runs': self.jsonData}
        self.runNum += 1
        with open(fileName, 'w') as outfile:
            json.dump(jsonDict, outfile)

    def produceAnnotation(self, arm, door, arm_speeds):
        self.arm = arm
        self.door = door
        #info for inputmotorLimit
        currentAngles = self.arm.get_angles()
        currentPose = self.arm.get_points(currentAngles).tolist()
        doorPose = self.door.getPose().tolist()
        knobPose = self.door.getKnobPose().tolist()
        knobClawDif = [knobPose[0] - currentPose[-1][0], knobPose[1] - currentPose[-1][1]]
        motorSpeeds = self.arm.get_motor_speeds()

        #info for output
        targetPose = self.arm.get_target_pose()
        targetAngles = self.arm.get_target_angles().tolist()
        targetSpeeds = self.arm.get_target_speeds()

        annot = {}
        annot['currentPose'] = currentPose
        annot['currentAngles'] = currentAngles
        annot['doorPose'] = doorPose
        annot['knobPose'] = knobPose
        annot['knobClawDif'] = knobClawDif
        annot['motorSpeeds'] = motorSpeeds
        annot['targetPose'] = targetPose
        annot['targetAngles'] = targetAngles
        annot['targetSpeeds'] = targetSpeeds
        annot['targetIKSpeeds'] = arm_speeds[0:4]
        self.jsonData.append(annot)

class ArmSim (Framework):
    name = "Arm Simulation Data Gen"
    alphabet = [False] * 26


    def __reset(self):
        self.alphabet = [False] * 26
        Framework.__init__(self)
        Framework.setZoom(self, 115)
        Framework.setCenter(self, [0, 1.74])
        self.arm_speeds = [0,0,0,3,-3]
        self.wristAngle = math.pi/2
        self.doorOpened = False
        self.recording = False
        self.reset = False
        self.arm_angles = [math.pi/4, 3*math.pi/4, 3*math.pi/4, math.pi/2, math.pi/2]
        lengths = [1, 1, .4, .27]
        self.arm = Arm(self.world, self.arm_angles, lengths, basePos = (-.1, 0.08), motorRange=2*math.pi)

        #ground = self.world.CreateBody(position=(0, 0))
        #ground.CreateEdgeChain([(-4, -2), (-4, .6), (-2, .6),(-2, 0),(2, 0),(2, 6),(4, 6),(4, -2), (-2, -2)])

        self.door = Door(self.world, (-2, .62), 1.2)

    def __init__(self):
        self.__reset()
        self.dataGen = SimDataGen("./data/runsIK/")

    def Step(self, settings):
        if not self.doorOpened:
            #print(self.arm.get_points(self.arm.get_angles()))
            settings.positionIterations = 50
            settings.velocityIterations = 250
            if self.door.joint.angle < -math.pi/3:
                self.doorOpened = True
                print("Door Successfully Opened")
            Framework.Step(self, settings)
            self.update_keyboard_angles()
            self.arm.update_arm_IK(self.arm_speeds[0], self.arm_speeds[1], self.arm_speeds[2], self.arm_speeds[3])
            if self.recording:
                print("recording")
                self.dataGen.produceAnnotation(self.arm, self.door, self.arm_speeds)

        if self.reset:
            print("reset")
            if self.recording:
                self.dataGen.closeFile()
                print("closed file")
            self.dataGen.newFile()
            self.__reset()
            self.reset = False

    def update_keyboard_angles(self):

        if self.alphabet[ascii_lowercase.index('a')]:
            self.arm_speeds[0] = -1
        elif self.alphabet[ascii_lowercase.index('d')]:
            self.arm_speeds[0] = 1
        else:
            self.arm_speeds[0] = 0

        if self.alphabet[ascii_lowercase.index('w')]:
            self.arm_speeds[1] = 1
        elif self.alphabet[ascii_lowercase.index('s')]:
            self.arm_speeds[1] = -1
        else:
            self.arm_speeds[1] = 0

        if self.alphabet[ascii_lowercase.index('j')]:
            self.arm_speeds[2] = 1
        elif self.alphabet[ascii_lowercase.index('l')]:
            self.arm_speeds[2] = -1
        else:
            self.arm_speeds[2] = 0

        if self.alphabet[ascii_lowercase.index('i')]:
            self.arm_speeds[3] = -3
            self.arm_speeds[4] = 3
        elif self.alphabet[ascii_lowercase.index('k')]:
            self.arm_speeds[3] = 3
            self.arm_speeds[4] = -3




        angleDif = math.pi/120
        mid = (math.pi/2)
        max = mid+self.arm.motorRange/2
        min = mid-self.arm.motorRange/2
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
        if self.alphabet[ascii_lowercase.index('o')] and self.arm_angles[3] <= max:
            self.arm_angles[3] += angleDif
        if self.alphabet[ascii_lowercase.index('p')]  and self.arm_angles[3] >= min:
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

        if self.alphabet[ascii_lowercase.index('b')]:
            self.recording = True
        if self.alphabet[ascii_lowercase.index('n')]:
            self.recording = False
        if self.alphabet[ascii_lowercase.index('m')]:
            print("reset")
            self.reset = True

    def MouseDown(self, p):
        self.arm.target_pose[0] = p[0]
        self.arm.target_pose[1] = p[1]

    def KeyboardUp(self, key):
        for letter in ascii_lowercase:
            if key == eval("Keys.K_" + letter):
                self.alphabet[ascii_lowercase.index(letter)] = False
if __name__ == "__main__":
    main(ArmSim)
