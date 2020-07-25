import math
import Box2D
import copy
import numpy as np
class Arm:
    leftBend = True

    def set_target_angles(self, angles, pos_mode = False):
        self.pos_mode = pos_mode
        self.target_angles = angles

    def get_target_angles(self):
        return self.target_angles

    def get_target_pose(self):
        return self.target_pose

    def get_target_speeds(self):
        return self.target_speeds

    def get_angles(self):
        return [self.shoulderMotor.angle,  self.elbowMotor.angle, \
            self.wristMotor.angle, self.clawMotor.angle, self.clawMotor2.angle]

    def set_motor_speeds(self, speeds):
        self.target_speeds = speeds
        self.shoulderMotor.motorSpeed = speeds[0]
        self.elbowMotor.motorSpeed = speeds[1]
        self.wristMotor.motorSpeed = speeds[2]
        self.clawMotor.motorSpeed = speeds[3]
        self.clawMotor2.motorSpeed = speeds[4]


    def get_motor_speeds(self):
        return [self.shoulderMotor.speed, self.elbowMotor.speed, self.wristMotor.speed, self.clawMotor.speed, self.clawMotor2.speed]

    def PD(self, target, d_gain = 0, p_gain = 5):
        current = self.get_angles()
        d = []
        difs = []
        for i in range(len(current)):
            dif = target[i] - current[i]
            while dif > math.pi or dif < -math.pi:
                if dif > math.pi:
                    dif -= 2*math.pi
                else:
                    dif += 2*math.pi
            difs.append(dif)
            d.append(dif)
        self.prev_error = dif
        return[(d[i]*d_gain) + (difs[i]*p_gain) for i in range(len(current))]

    def update_arm(self):
        target = self.get_target_angles()
        #pos = self.get_target_pose()
        #ikAngles = self.det_inverse_kinematics(pos[0], pos[1], pos[2])
        #for i in range(3):
        #    target[i] = ikAngles[i]
        speeds = self.PD(target)
        self.set_target_angles(target)
        self.set_motor_speeds(speeds)

    def get_abs_angles(self, angles):
        return [angles[0], angles[0] + angles[1] - math.pi/2,\
            angles[0] + angles[1] + angles[2] - math.pi, \
            angles[0] + angles[1] + angles[2] + angles[3] - (3*math.pi/2), \
            angles[0] + angles[1] + angles[2] + angles[4] - (3*math.pi/2)]

    def set_target_position(self, x, y, wrist, claw):
        self.pos_mode = True
        x = x - self.basePos[0]
        y = y - self.basePos[1]
        target_position = np.array([x, y, angle])

    def det_inverse_kinematics(self, x, y, wristAngle):
        currAngles = self.get_target_angles()
        leftBend = x > 0
        x = x - self.basePos[0]
        y = y - self.basePos[1]
        x = x - ((self.lengths[2] + self.lengths[3]/2) * math.cos(wristAngle))
        y = y - ((self.lengths[2] + self.lengths[3]/2) * math.sin(wristAngle))
        q1 = math.atan2(y, x)
        l = math.sqrt((x*x)+(y*y))
        try:
            q2 = math.acos(((l ** 2) + (self.lengths[0] ** 2) - (self.lengths[1] ** 2))/(2*self.lengths[0]*l))
        except ValueError:
            return currAngles

        if False:
            shoulder = q1 + q2
        else:
            shoulder = q1 - q2

        x = x - (self.lengths[0] * math.cos(shoulder))
        y = y - (self.lengths[0] * math.sin(shoulder))
        elbow = math.atan2(y, x)
        elbow = elbow - shoulder + (math.pi/2)
        wrist = wristAngle - elbow - shoulder + math.pi
        angles = [shoulder, elbow, wrist]
        return angles

    def get_points(self):
        abs_angles  = self.get_abs_angles(self.get_angles())[:3]
        points = [[0,0],]
        for i in range(len(abs_angles)):
            length = self.lengths[i]
            if i == 2:
                length += self.lengths[3]/2
            xDif = length * math.cos(abs_angles[i])
            yDif = length * math.sin(abs_angles[i])
            points.append([xDif + points[i][0], yDif+ points[i][1]])
        return points


    def __init__(self, world, angles, lengths, basePos = (0,0), thickness=0.05, dense=0.1, motorLimit = math.pi, maxTorques=[2, 1.6, .8, .4],  rot_damping = 10, lin_damping = 10):
        rotMin = (math.pi/2) - (motorLimit/2)
        rotMax = (math.pi/2) + (motorLimit/2)
        self.motorLimit = motorLimit
        self.prev_error = [0,0,0,0,0]
        self.target_pose = [0, 2, math.pi/2]
        self.world = world
        self.angles = angles
        self.basePos = basePos
        self.pos_mode = False
        self.set_target_angles(angles)
        self.lengths = lengths
        self.thickness = thickness
        self.addAngles = self.get_abs_angles(angles)
        self.clawAngle = math.pi
        ground = self.world.CreateStaticBody(position=basePos)
        dif0 = [(self.lengths[0] * math.cos(self.angles[0])), (self.lengths[0] * math.sin(self.angles[0]))]
        p1 = [dif0[0] + basePos[0], dif0[1] + basePos[1]]
        shoulderBody = self.world.CreateDynamicBody(position=p1, angle = 0,  angularDamping = rot_damping, linearDamping=lin_damping)
        shoulderBody.CreatePolygonFixture(box=(self.lengths[0]/2, self.thickness, (-dif0[0]/2,-dif0[1]/2), self.angles[0]), density=dense, friction=0.3)

        sjd = Box2D.b2RevoluteJointDef(
            bodyA=ground,
            bodyB=shoulderBody,
            localAnchorA=(0, 0),
            localAnchorB=(-dif0[0], -dif0[1]),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=maxTorques[0],
            motorSpeed=0,
            referenceAngle = -self.angles[0],
            lowerAngle=rotMin,
            upperAngle=rotMax,
        )

        self.shoulderMotor = self.world.CreateJoint(sjd)

        dif = [self.lengths[1] * math.cos(self.addAngles[1]), self.lengths[1] * math.sin(self.addAngles[1])]
        p2 = [p1[0] + dif[0], p1[1] + dif[1]]
        elbowBody = self.world.CreateDynamicBody(position=p2, angle = 0,  angularDamping = rot_damping, linearDamping=lin_damping)
        elbowBody.CreatePolygonFixture(box=(self.lengths[1]/2, self.thickness, (-dif[0]/2,-dif[1]/2), self.addAngles[1]), density=dense, friction=0.3)


        ejd = Box2D.b2RevoluteJointDef(
            bodyA=shoulderBody,
            bodyB=elbowBody,
            localAnchorA=(0, 0),
            localAnchorB=(-dif[0], -dif[1]),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=maxTorques[1],
            motorSpeed=0,
            referenceAngle = -self.angles[1],
            lowerAngle=rotMin,
            upperAngle=rotMax,
        )

        self.elbowMotor = self.world.CreateJoint(ejd)

        dif2 = [self.lengths[2] * math.cos(self.addAngles[2]), self.lengths[2] * math.sin(self.addAngles[2])]
        p3 = [p2[0] + dif2[0], p2[1] + dif2[1]]
        wristBody = self.world.CreateDynamicBody(position=p3, angle = 0,  angularDamping = rot_damping, linearDamping=lin_damping)
        wristBody.CreatePolygonFixture(box=(self.lengths[2]/2, self.thickness, (-dif2[0]/2,-dif2[1]/2), self.addAngles[2]), density=dense, friction=0.3)
        wristBody.CreatePolygonFixture(box=(self.lengths[2]/2, self.thickness, (0, 0), self.addAngles[2] + math.pi/2), density=dense, friction=0.3)

        wjd = Box2D.b2RevoluteJointDef(
            bodyA=elbowBody,
            bodyB=wristBody,
            localAnchorA=(0, 0),
            localAnchorB=(-dif2[0], -dif2[1]),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=maxTorques[2],
            motorSpeed=0,
            referenceAngle = -self.angles[2],
            lowerAngle=rotMin,
            upperAngle=rotMax,
        )

        self.wristMotor = self.world.CreateJoint(wjd)

        dif3 = [self.lengths[3] * math.cos(self.addAngles[3]), self.lengths[3] * math.sin(self.addAngles[3])]
        dif4 = [self.lengths[2]/2 * math.cos(self.addAngles[3] + math.pi/2), self.lengths[2]/2 * math.sin(self.addAngles[3] + math.pi/2)]
        p4 = [p3[0] + dif3[0] + dif4[0], p3[1] + dif3[1] + dif4[1]]
        clawBody = self.world.CreateDynamicBody(position=p4, angle = 0,  angularDamping = rot_damping, linearDamping=lin_damping)
        clawBody.CreatePolygonFixture(box=(self.lengths[3]/2, self.thickness, (-dif3[0]/2,-dif3[1]/2), self.addAngles[3]), density=dense, friction=1.3)

        cjd = Box2D.b2RevoluteJointDef(
            bodyA=wristBody,
            bodyB=clawBody,
            localAnchorA=(dif4[0], dif4[1]),
            localAnchorB=(-dif3[0], -dif3[1]),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=maxTorques[3],
            motorSpeed=0,
            referenceAngle = -self.angles[3],
            lowerAngle=math.pi/4,
            upperAngle=3*math.pi/4,
        )

        self.clawMotor = self.world.CreateJoint(cjd)

        p5 = [p3[0] + dif3[0] - dif4[0], p3[1] + dif3[1] - dif4[1]]
        clawBody2 = self.world.CreateDynamicBody(position=p5, angle = 0,  angularDamping = rot_damping, linearDamping=lin_damping)
        clawBody2.CreatePolygonFixture(box=(self.lengths[3]/2, self.thickness, (-dif3[0]/2,-dif3[1]/2), self.addAngles[4]), density=dense, friction=1.3)

        cjd2 = Box2D.b2RevoluteJointDef(
            bodyA=wristBody,
            bodyB=clawBody2,
            localAnchorA=(-dif4[0], -dif4[1]),
            localAnchorB=(-dif3[0], -dif3[1]),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=maxTorques[3],
            motorSpeed=0,
            referenceAngle = -self.angles[4],
            lowerAngle=math.pi/4,
            upperAngle=3*math.pi/4,
        )

        self.clawMotor2 = self.world.CreateJoint(cjd2)

        self.update_arm()
