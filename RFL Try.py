# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 17:34:04 2022

@author: user
"""

import gym
from gym import Env
from gym.spaces import Discrete, Box, Dict

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
import os

import random as rd

import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d

import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions

from pymunk._chipmunk_cffi import ffi, lib
pygame.init()
gameDisplay = pygame.display.set_mode((50,50))
size = 800, 800
display = pygame.display.set_mode((size))
options = DrawOptions(display)
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body
b1 = space.static_body
FPS = 120

def convert_coordinates(point):
    return int(point[0]), int(800-point[1])

def get_theta(x_h, x_1, y_h, y_1):
    return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1, x2, y1, y2, theta):
    return np.arctan2(x2 - x1, y2- y1) - theta

def get_iota(x1, x2, y1, y2, theta, phi):
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi

class measurement_body:
    def __init__(self):
        self.body = pymunk.Body()
        self.body.position = (400,40)
        self.shape = pymunk.Circle(self.body, 1)
        self.shape.color = (255,0,0)
        space.add(self.body, self.shape)
    
class Segment:
    def __init__(self, p0, a, b, radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.body.center_of_gravity = center_of_gravity
        self.shape = pymunk.Segment(self.body, self.a, self.b, radius)
        self.shape.density = density
        self.shape.elasticity = 0
        self.shape.filter = pymunk.ShapeFilter(group=1)
        self.shape.color = (0, 255, 0, 0)
        space.add(self.body, self.shape)

class Leg:
    def __init__(self, p0, a, b, c, d, radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.body.center_of_gravity = center_of_gravity
        self.leg= pymunk.Segment(self.body, self.a, self.b , radius=radius)
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = density
        self.leg.color = (0, 255, 0, 0)
        self.foot= pymunk.Segment(self.body, self.c, self.d, radius=radius)
        self.foot.filter = pymunk.ShapeFilter(group = 1)
        self.foot.density = density
        self.foot.filter = pymunk.ShapeFilter(group=1)
        self.foot.color = (0, 255, 0, 0)
        space.add(self.body, self.leg, self.foot)
        
        
        
class Simplemotor:
    def __init__(self, b, b2, rate=5, switch="off"):
        self.rate = rate
        self.b = b
        self.b2 = b2
        self.simplemotor = pymunk.SimpleMotor(self.b, self.b2, self.rate)
        space.add(self.simplemotor)
    def remove(self):
        space.remove(self.simplemotor)

class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)
    
        

                  
# class dead_hang_joint:
#     def __init__(self, b, b2, min, max, collide=True):
#         joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, angvel1}\nseg2:{angvel2}")
        # print(leg.bomax)
#         joint.collide_bodies = collide
#     def dead_position(self, constraints, phi):
#         if phi == 0 and len(constraints) < 6:  
            
        

class PivotJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)

class Swing_body:
    def __init__(self,p0, vx1,vy1,vx2,vy2,vx3,vy3, radius=10, center_of_gravity = (0,0), density=0.05):
        self.body = pymunk.Body()
        self.body.position = p0
        s1 = pymunk.Segment(self.body, vx1, vy1 , radius=radius)
        s1.filter = pymunk.ShapeFilter(group = 1)
        s1.density = density
        s2 = pymunk.Segment(self.body, vx2, vy2, radius=radius)
        s2.filter = pymunk.ShapeFilter(group = 1)
        s2.density = density
        s3 = pymunk.Segment(self.body, vx3,vy3, radius=radius)
        s3.filter = pymunk.ShapeFilter(group = 1)
        s3.density = density
        space.add(self.body, s1,s2,s3)
        
def angle_reached(theta, high_score):
    if len(high_score) == 0:
        high_score.append(theta)
    elif high_score[0] < abs(theta):
        high_score[0] = abs(theta)
        
    highest_score = high_score[0]
    return highest_score


# b1 = measurement_body()

# hinge_point1 = (0, -100) # seg 1
# hinge_point2 = (0, 100)
# swing_body = (400, 625)
# swing_top1 = (30, -25)
# swing_top2 = (-30, -25)
# swing_mid1 = (0, -25)
# swing_mid2 = (0, 25)
# swing_bottom1 = (-20, 25)
# swing_bottom2 = (20, 25)
# hinge_point3 = (0, -30) # seg 2
# hinge_point4 = (0, 30)

# "Pymunk Bodies"

# segment = Segment((400 , 500), hinge_point1 , hinge_point2)
# leg = Leg((420,680), hinge_point3, hinge_point4, (0,30), (15,30), density= 0.05)
# swing = Swing_body(swing_body, swing_top1,swing_top2, swing_mid1, swing_mid2, swing_bottom1, swing_bottom2)
# PinJoint(swing.body, leg.body, swing_bottom2, hinge_point3)
# PinJoint(segment.body, swing.body, hinge_point2, swing_mid1)
# PinJoint(b0, segment.body, (400,400), hinge_point1)

class ModelEnv(Env):
    def __init__(self):
        self.state = None
        self.action_space = Discrete(2)
        self.model_length = 60000
        self.observation_space = Box(np.array([0,0,0]), np.array([np.pi/2,np.pi/2,np.pi/2]))
        
    def reset(self):
        self.hinge_point1 = (0, -100) # seg 1
        self.hinge_point2 = (0, 100)
        self.swing_body = (400, 625)
        self.swing_top1 = (30, -25)
        self.swing_top2 = (-30, -25)
        self.swing_mid1 = (0, -25)
        self.swing_mid2 = (0, 25)
        self.swing_bottom1 = (-20, 25)
        self.swing_bottom2 = (20, 25)
        self.hinge_point3 = (0, -30) # seg 2
        self.hinge_point4 = (0, 30)
        
        "Pymunk Bodies"
        
        self.segment = Segment((400 , 500), self.hinge_point1 , self.hinge_point2)
        self.leg = Leg((420,680), self.hinge_point3, self.hinge_point4, (0,30), (15,30), density= 0.05)
        self.swing = Swing_body(self.swing_body, self.swing_top1,self.swing_top2, self.swing_mid1, self.swing_mid2, self.swing_bottom1, self.swing_bottom2)
        self.pinjoint1 = PinJoint(self.swing.body, self.leg.body, self.swing_bottom2, self.hinge_point3)
        self.pinjoint2 = PinJoint(self.segment.body, self.swing.body, self.hinge_point2, self.swing_mid1)
        self.pinjoint3 = PinJoint(b0, self.segment.body, (400,400), self.hinge_point1)

        
        self.model_length = 60000
        xh, yh = (400,400)
        x1, y1 = self.segment.body.position[0], self.segment.body.position[1]
        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = self.segment.body.position[0] + 100*np.sin(theta) , self.segment.body.position[1] + 100*np.cos(theta) 
        x3, y3 = self.swing.body.position[0], self.swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = self.swing.body.position[0] + 25*np.sin(theta+phi) + 20*np.cos(theta+phi), self.swing.body.position[1] + 25*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = self.leg.body.position[0], self.leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        self.high_score = []
        
        self.state = np.array([theta, phi, iota])
        return self.state
            
    def clean(self):
        space.remove(self.pinjoint1)
        space.remove(self.pinjoint2)
        space.remove(self.pinjoint3)
        space.remove(self.segment.body)
        space.remove(self.leg.body)
        space.remove(self.swing.body)

        
    def step(self, action):
        pygame.display.set_caption("Double pendulum interactive Simulation")
        
        x = "const motor"
        constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
        
        xh, yh = (400,400)
        x1, y1 = self.segment.body.position[0], self.segment.body.position[1]
        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = self.segment.body.position[0] + 100*np.sin(theta) , self.segment.body.position[1] + 100*np.cos(theta) 
        x3, y3 = self.swing.body.position[0], self.swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = self.swing.body.position[0] + 25*np.sin(theta+phi) + 20*np.cos(theta+phi), self.swing.body.position[1] + 25*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = self.leg.body.position[0], self.leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        #print(f"iota={iota}")
        const_angvel = self.swing.body.angular_velocity
        
        self.model_length -= 1

        if  self.model_length <=0:
            done = True
        else:
            done = False
        # print(f"seg1:{angvel1}\nseg2:{angvel2}")
        # print(leg.body.angular_velocity)
        # abs_vel = np.sqrt(segment.body.velocity[0]**2 + segment.body.velocity[1]**2)
        # if segment.body.velocity[0]< 1:
        #     rad_vel = -abs_vel/150
        # else:
        #     rad_vel = abs_vel/150
        # print(rad_vel)
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                #print(f"Highest angle reached was:{np.rad2deg(highest_score)}")
                pygame.quit()
                sys.exit()
            if self.model_length == 0:
                self.clean()
                
        if action == 0:
            if iota < np.pi/2 and x == "const motor":
                constant_motor.remove()
                x = "positive motor"
                pos_motor = Simplemotor(self.swing.body, self.leg.body, 5)
        elif action == 1:
            if iota > 0 and x == "const motor":
                constant_motor.remove()
                x = "negetive motor"
                neg_motor = Simplemotor(self.swing.body, self.leg.body, -5)
                
        if iota >= np.pi/2 and x == "positive motor":
            pos_motor.remove()
            constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
            x = "const motor"
        if iota <= 0 and x == "negetive motor":
            neg_motor.remove()
            constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
            x = "const motor"
                            
                        
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_SPACE]: # kick input
        #     simplemotor.switch = "on"
        #     if iota >= np.pi/2:
        #         if len(space.constraints) == 5:
        #             space.remove(simplemotor.simplemotor)
        #         leg.body.angular_velocity = angvel1
        #     else:
        #         simplemotor.drive(space.constraints, phi)
        # else:
        #     simplemotor.switch = "off"
        #     if iota <= 0:
        #         leg.body.angular_velocity = angvel1
        #     else:
        #         simplemotor.drive(space.constraints, phi)
        font =  pygame.font.SysFont('microsoft Yahei',60)
        Theta = format(np.rad2deg(theta), '.2f')
        surface = font.render('Theta is:{}'.format(Theta), False, (255,200,10))
        reward = angle_reached(theta, self.high_score)
        self.state = np.array([theta,phi,iota])
        display.fill((255, 255, 255)) 
        gameDisplay.blit(surface,(50,50))
        pygame.display.flip()
        space.debug_draw(options)
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)
        #print(len(space.constraints))
        #print(x)
        info = {}
        return self.state, reward, done, info
    def render():
        pass
        

#check_env(env, warn=True)env = ModelEnv()
# a = 0
# action = 0
# episodes = 5
# the = []
env = ModelEnv()
# for episode in range(1, episodes +1):
#     state = env.reset()
#     done = False
#     score = 0
#     action = env.action_space.sample()
#     a = 0
#     pygame.display.set_caption("Double pendulum interactive Simulation")
#     while not done:
#         if a % 9 == 0:
#             action = env.action_space.sample()
#         state, done, highest_score = env.step(action)
#         the.append(np.abs(np.rad2deg(highest_score)))
#         #print(max(the))
#         a += 1
#     print(state)
#     print(np.rad2deg(highest_score))
# log_path = os.path.join('Training','Logs')
# model = PPO("MlpPolicy", env, verbose = 1, tensorboard_log=log_path)
# model.learn(total_timesteps = 10000)
ppo_path = os.path.join('Project','Training','Saved Models','PPO_Segment') # should create the file and The PPO_Segment has uploaded.
model = PPO.load(ppo_path, env)

obs = env.reset()
the = []
a = 0
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    the.append(np.absolute(np.rad2deg(rewards)))
    if max(the) > a:
        print('The max angle is {}'.format(max(the)))
    a = max(the)

env.close()
print(max(the))
