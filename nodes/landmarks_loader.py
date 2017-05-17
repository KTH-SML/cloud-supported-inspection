#! /usr/bin/env python

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import cloud_coverage.srv as ccs
import rospy as rp

import threading as thd
import math as m


"""Cylinder"""
NUM_LANDMARKS = 100
DELTA_ANGLE = 0.3
DELTA_HEIGHT = 0.06
RADIUS = 1.5

angle = 0.2
height = 0.0

poses = list()
targets = list()

for index in range(NUM_LANDMARKS):
    position = gmi.Point(RADIUS*m.cos(angle), RADIUS*m.sin(angle), height)
    orientation = gmi.UnitQuaternion(axis=gmi.Vector(0,0,1), angle=angle+m.pi)
    pose = gmi.Pose(position, orientation)
    poses.append(pose.serialized)
    targets.append(0.99)
    angle += DELTA_ANGLE
    height += DELTA_HEIGHT


"""Wall"""
# poses = list()
# targets = list()
#
# X0 = -2.0
# Z0 = 1.0
# XMAX = 2.0
# ZMAX = 4.0
# DX = 0.5
# DZ = 0.3
#
# z = Z0
# while z < ZMAX:
#     x = X0
#     while x < XMAX:
#         pos = gmi.Point(x,2.0,z)
#         ori = gmi.UnitQuaternion(axis=gmi.E3, angle=m.pi/2)
#         pose = gmi.Pose(pos,ori)
#         poses.append(pose.serialized)
#         targets.append(0.99)
#         x += DX
#     z += DZ

rp.init_node("landmarks_loader")

rp.wait_for_service("add_landmarks")
proxy = rp.ServiceProxy("add_landmarks", ccs.AddLandmarks)
proxy.call(poses, targets)
