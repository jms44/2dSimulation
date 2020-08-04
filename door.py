import math
import Box2D
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2Color, b2EdgeShape, b2FixtureDef, b2PolygonShape)
import numpy as np
class Door:
    def getPose(self):
        return self.pos

    def getKnobPose(self):
        x = self.pos[0] - math.sin(self.body.angle - self.knobAngle)
        y = self.pos[1] + math.cos(self.body.angle - self.knobAngle)
        return np.array([x, y])

    def __init__(self, world, pos, length, thickness = 0.05):
        self.world = world
        self.pos = np.array(pos)
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
        self.knob = doorBody.CreatePolygonFixture(vertices=verts, density=0.1, friction=0.3)

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

        knobX = xOffset
        knobY = length - yOffset
        self.knobLength = math.sqrt((knobX**2) + (knobY**2))
        self.knobAngle = math.atan2(knobX, knobY)
        self.joint = self.world.CreateJoint(wjd)
        self.body = doorBody
