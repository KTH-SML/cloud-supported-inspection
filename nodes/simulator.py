#! /usr/bin/env python

import sys
sys.ps1 = 'SOMETHING'

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import cloud_coverage.srv as ccs
import rospy as rp

import threading as thd



rp.init_node("simulator")
init_pos = gmi.Point(*[float(arg) for arg in rp.get_param("initial_position", "0 0 0").split()])
rp.logwarn(init_pos)

pose = gmi.Pose(init_pos, gmi.UnitQuaternion())
cmd_twist = None

LOCK = thd.Lock()
FREQUENCY = 3e1
TIME_STEP = 1/FREQUENCY
RATE = rp.Rate(FREQUENCY)

def twist_callback(msg):
    global cmd_twist
    LOCK.acquire()
    cmd_twist = gmi.Twist(msg)
    LOCK.release()

sub = rp.Subscriber(name="cmd_twist", data_class=gms.Twist, callback=twist_callback)

pub = rp.Publisher(name="pose", data_class=gms.Pose, queue_size=10)

while not rp.is_shutdown():
    LOCK.acquire()
    if not cmd_twist is None:
        pose = cmd_twist.integrate(TIME_STEP).apply_to(pose)
        cmd_twist = None
    LOCK.release()
    pub.publish(pose.position, pose.orientation)
    RATE.sleep()
