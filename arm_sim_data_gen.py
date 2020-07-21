import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
from arm import Arm
from door import Door
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

    def produceAnnotation(self, arm, door):
        self.arm = arm
        self.door = door
        #info for input
        currentPose = self.arm.get_points()
        currentAngles = self.arm.get_angles()
        doorPose = self.door.getPose()
        knobPose = self.door.getKnobPose()
        knobClawDif = [knobPose[0] - currentPose[-1][0], knobPose[1] - currentPose[-1][1]]
        motorSpeeds = self.arm.get_motor_speeds()

        #info for output
        targetPose = self.arm.get_target_pose()
        targetAngles = self.arm.get_target_angles()
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

        self.jsonData.append(annot)

class ArmSim (Framework):
    name = "Arm Simulation"


    def __reset(self):
        self.qweruiop = [False] * 10
        Framework.__init__(self)
        Framework.setZoom(self, 115)
        Framework.setCenter(self, [0, 1.74])
        self.wristAngle = math.pi/2
        self.doorOpened = False
        self.recording = False
        self.reset = False
        self.arm_angles = [math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2]
        lengths = [1, 1, .4, .27]
        self.arm = Arm(self.world, self.arm_angles, lengths, basePos = (-.1, 0.08), motorLimit=4*math.pi/2)
        ground = self.world.CreateBody(position=(0, 0))
        ground.CreateEdgeChain([(-4, -2), (-4, .6), (-2, .6),(-2, 0),(2, 0),(2, 6),(4, 6),(4, -2), (-2, -2)])

        self.door = Door(self.world, (-2, .62), 1.2)


    def __init__(self):
        self.__reset()
        self.dataGen = SimDataGen("/Users/joshuasmith/Desktop/NistInternship/Door2D/data/runs/")


        #box = self.world.CreateDynamicBody(position = (1, 2))
        #box.CreatePolygonFixture(box=(.1, .1), density=0.3, friction=0.5)

    def Step(self, settings):
        if not self.doorOpened:
            settings.positionIterations = 50
            settings.velocityIterations = 250
            if self.door.joint.angle < -math.pi/3:
                self.doorOpened = True
                print("Door Successfully Opened")
            Framework.Step(self, settings)
            self.update_keyboard_angles()
            self.arm.update_arm()
            if self.recording:
                print("recording")
                self.dataGen.produceAnnotation(self.arm, self.door)

        if self.reset:
            if self.recording:
                self.dataGen.closeFile()
            self.__reset()
            self.dataGen.newFile()

    def update_keyboard_angles(self):
        angleDif = math.pi/120
        mid = (math.pi/2)
        for i in range(len(self.arm_angles) - 1):
            if self.qweruiop[2*i] and self.arm_angles[i] <= (mid+self.arm.motorLimit/2) - angleDif:
                self.arm_angles[i] += angleDif
            if self.qweruiop[(2*i) + 1] and self.arm_angles[i] >= (mid-self.arm.motorLimit/2) + angleDif:
                self.arm_angles[i] -= angleDif
        if self.qweruiop[8]:
            self.wristAngle += angleDif
        if self.qweruiop[9]:
            self.wristAngle -= angleDif
        p = self.arm.get_target_pose()
        self.arm.target_pose = [p[0], p[1], self.wristAngle]
        self.arm_angles[-1] = math.pi - self.arm_angles[-2]


    def Keyboard(self, key):
        if key == Keys.K_q:
            self.qweruiop[0] = True
        if key == Keys.K_w:
            self.qweruiop[1] = True
        if key == Keys.K_e:
            self.qweruiop[2] = True
        if key == Keys.K_r:
            self.qweruiop[3] = True
        if key == Keys.K_u:
            self.qweruiop[4] = True
        if key == Keys.K_i:
            self.qweruiop[5] = True
        if key == Keys.K_o:
            self.qweruiop[6] = True
        if key == Keys.K_p:
            self.qweruiop[7] = True
        if key == Keys.K_k:
            self.qweruiop[8] = True
        if key == Keys.K_l:
            self.qweruiop[9] = True
        if key == Keys.K_a:
            self.recording = True
        if key == Keys.K_s:
            self.recording = False
        if key == Keys.K_d:
            self.reset = True


    def MouseDown(self, p):
        self.arm.target_pose = [p[0], p[1], self.wristAngle]

    def KeyboardUp(self, key):
        if key == Keys.K_q:
            self.qweruiop[0] = False
        if key == Keys.K_w:
            self.qweruiop[1] = False
        if key == Keys.K_e:
            self.qweruiop[2] = False
        if key == Keys.K_r:
            self.qweruiop[3] = False
        if key == Keys.K_u:
            self.qweruiop[4] = False
        if key == Keys.K_i:
            self.qweruiop[5] = False
        if key == Keys.K_o:
            self.qweruiop[6] = False
        if key == Keys.K_p:
            self.qweruiop[7] = False
        if key == Keys.K_k:
            self.qweruiop[8] = False
        if key == Keys.K_l:
            self.qweruiop[9] = False
if __name__ == "__main__":
    main(ArmSim)
