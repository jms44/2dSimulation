import sys, random
import pygame
from pygame.locals import *
import pymunk
import pymunk.pygame_util
import math

class ArmLink:
    def __init__(self, space, base_body, body_pos, length, angle, mass=1, thickness=5):
        b = body_pos
        dif = (math.sin(angle) * length, math.cos(angle) * length)
        v = b + dif

        base_dif = body_pos - base_body.position
        link_body = pymunk.Body(mass, mass * length * length  / 3)
        link_body.position = body_pos
        self.link = pymunk.Segment(link_body, (0,0), dif, thickness)
        self.link.filter = pymunk.ShapeFilter(categories=0b1, mask=0b0)

        link_body.moment = pymunk.moment_for_segment(mass, self.link.a, self.link.b, thickness)

        link_body.center_of_gravity = (self.link.a + self.link.b)/2.0

        base_motor = pymunk.SimpleMotor(base_body, link_body, 0)
        base_joint = pymunk.PinJoint(base_body, link_body, base_dif, (0,0))
        base_joint.error_bias = 0.0001

        base_clamp = pymunk.RotaryLimitJoint(base_body, link_body, (-3*math.pi/4), (3*math.pi/4))

        space.add(self.link, link_body, base_motor, base_joint, base_clamp)

        print("Body pos: " + str(link_body.position))
        print("COG pos: " + str(link_body.center_of_gravity))
        print("a" + str(self.link.a))
        print("b" + str(self.link.b))


        self.motor = base_motor
        self.body = link_body
        self.start_tip = v

class Arm:
    def __init__(self, space, lengths, angles):
        arm_base_body = pymunk.Body(body_type = pymunk.Body.STATIC)
        arm_base_body.position = (500,100)

        start_body = arm_base_body
        start_pos = arm_base_body.position
        self.motors = []
        self.segments = []
        self.bodies = []

        for i in range(len(lengths)):
            armLink = ArmLink(space, start_body, start_pos, lengths[i], angles[i])
            start_body = armLink.body
            start_pos = armLink.start_tip
            self.motors.append(armLink.motor)
            self.segments.append(armLink.link)
            self.bodies.append(armLink.body)

    def getAbsAngles(self):
        angles = []
        for body in self.bodies:
            angles.append(body.angle)
        return angles


def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 1000))
    pygame.display.set_caption("Arm Simulation")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0.0, -900.0)

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    arm = Arm(space, [100, 100, 35], [0, 1, 0])

    ticks_to_next_ball = 10
    while True:
        print(arm.getAbsAngles())
        screen.fill((255,255,255))
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)


        space.debug_draw(draw_options)
        space.step(1/50.0)
        pygame.display.flip()
        clock.tick(50)

if __name__ == '__main__':
    main()
