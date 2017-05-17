#! /usr/bin/env python
import geometry_msgs.msg as gms
import geomthree.impl as gmi
import rospy as rp
import threading as thd


def saturate(vec, lim):
    if vec.norm > lim:
        return vec/vec.norm*lim
    return vec

rp.init_node("controller_bridge")
pub = rp.Publisher("reference", gms.PoseStamped, queue_size=10)
FREQUENCY = 30.0
PERIOD = 1/FREQUENCY
RATE = rp.Rate(FREQUENCY)
LOCK = thd.Lock()

pose_measurement = None
cmdvel = None

def pose_callback(msg):
    global pose_measurement
    LOCK.acquire()
    pose_measurement = gmi.Pose(msg)
    LOCK.release()

def cmdvel_callback(msg):
    global cmdvel
    LOCK.acquire()
    cmdvel = gmi.Twist(msg)
    LOCK.release()


rp.Subscriber("pose", gms.Pose, callback=pose_callback)
rp.Subscriber("cmd_twist", gms.Twist, callback=cmdvel_callback)

while pose_measurement is None or cmdvel is None:
    RATE.sleep()

while not rp.is_shutdown():
    LOCK.acquire()
    cmdvel.linear = saturate(cmdvel.linear, 0.1)
    reference = cmdvel.integrate(0.1).apply_to(pose_measurement)
    msg = gms.PoseStamped(pose=reference.serialized)
    pub.publish(msg)
    LOCK.release()
    RATE.sleep()
