import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
from arm import Arm

class Door:
    def __init__(self, world, pos, length, thickness = 0.05):
        self.world = world
        ground = self.world.CreateStaticBody(position=pos)
        p1 = [pos[0], pos[1] + length]
        doorBody = self.world.CreateDynamicBody(position=p1)
        doorBody.CreatePolygonFixture(box=(thickness/2, length/2, (0, -length/2), 0), density=0.1, friction=0.3)
        angle = 2 * math.pi / 8
        radius = 0.11;
        verts = []
        xOffset = 0.15
        yOffset = -0.15
        for i in range(8):
            x = xOffset + radius * math.cos(angle * i);
            y = yOffset + radius * math.sin(angle * i);
            verts.append((x, y))
        doorBody.CreatePolygonFixture(vertices=verts, density=0.1, friction=0.3)

        wjd = Box2D.b2RevoluteJointDef(
            bodyA=ground,
            bodyB=doorBody,
            localAnchorA=(0, 0),
            localAnchorB=(0, -length),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=.1,
            motorSpeed=.5,
            referenceAngle = 0,
            lowerAngle=-math.pi/2,
            upperAngle=0.0001,
        )

        self.joint = self.world.CreateJoint(wjd)
        self.body = doorBody



class ArmSim (Framework):
    name = "Arm Simulation"
    qweruiop = [False] * 10


    def __init__(self):
        Framework.__init__(self)
        self.wristAngle = math.pi/2
        self.doorOpened = False

        self.arm_angles = [math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2]
        lengths = [1, 1, .4, .27]
        self.arm = Arm(self.world, self.arm_angles, lengths, basePos = (-.1, 0.08), motorLimit=4*math.pi/2)
        ground = self.world.CreateBody(position=(0, 0))
        ground.CreateEdgeChain([(-4, -2), (-4, .6), (-2, .6),(-2, 0),(2, 0),(2, 6),(4, 6),(4, -2), (-2, -2)])

        self.door = Door(self.world, (-2, .62), 1.2)

        box = self.world.CreateDynamicBody(position = (1, 2))
        box.CreatePolygonFixture(box=(.1, .1), density=0.3, friction=0.5)

    def Step(self, settings):

        print(settings)
        settings.positionIterations = 50
        settings.velocityIterations = 250
        if self.door.joint.angle < -math.pi/3:
            self.doorOpened = True
            print("Door Successfully Opened")
        Framework.Step(self, settings)
        self.update_keyboard_angles()
        self.arm.update_arm()
        renderer = self.renderer


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
