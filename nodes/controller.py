#! /usr/bin/env python

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import cloud_coverage.srv as ccs
import cloud_coverage.footprints as cfp
import cloud_coverage.feasible_sets as cfs
import rospy as rp

import threading as thd
import math as m
import numpy as np

import recordclass as rcc



Landmark = rcc.recordclass('Landmark', ['POSE', 'TARGET_COVERAGE', 'coverage'])
Landmark.__new__.__defaults__ = 0.0, None
landmarks = dict()

Agent = rcc.recordclass('Agent', ['POSE_SUBSCRIBER', 'pose'])
Agent.__new__.__defaults__ = None,

pose = None
cmd_twist = None
#footprint = cfp.CompositeFootprint(best_distance=1.5)
footprint = cfp.EggFootprint(small_radius=1.0, big_radius=10.0)
footprint = footprint*footprint*cfp.AlignmentFootprint()
OFFSET = [float(elem) for elem in rp.get_param("/offset").split()]
rp.logwarn(OFFSET)
feasible_set = cfs.FeasibleCylinder(radius=5.0, x0=OFFSET[0], y0=OFFSET[1])
collision_set = cfs.FeasibleSphere(radius=2.5)



NAME = rp.get_param("name")

rp.init_node("controller")
INITIAL_TIME = rp.get_time()
ARRIVAL_TIME = rp.get_param("arrival_time", default=0.0)
DEPARTURE_TIME = rp.get_param("departure_time", default=float("inf"))

LOCK = thd.Lock()
FREQUENCY = 3e1
TIME_STEP = 1/FREQUENCY
FAST_RATE = rp.Rate(FREQUENCY)
SLOW_RATE = rp.Rate(FREQUENCY)
rate = FAST_RATE
GAIN = 5.0
SATURATION = 1.0

while rp.get_time() < INITIAL_TIME + ARRIVAL_TIME:
    rate.sleep()

rp.wait_for_service("/draw_agent")
draw_agent_proxy = rp.ServiceProxy(name="/draw_agent", service_class=ccs.DrawAgent)
draw_agent_proxy.call(NAME)


other_agents = dict()
def other_poses_callback(msg, name):
    global other_agents
    LOCK.acquire()
    other_agents[name].pose = gmi.Pose(msg)
    LOCK.release()

def new_agent_callback(msg):
    global other_agents
    LOCK.acquire()
    if not msg.data in other_agents and not msg.data == NAME:
        sub = rp.Subscriber("/"+msg.data+"/pose", gms.Pose, callback=other_poses_callback, callback_args=msg.data)
        other_agents[msg.data] = Agent(sub)
        rp.logwarn("{}: a new agent has come and its name is {}".format(NAME, msg.data))
    LOCK.release()

rp.Subscriber("/new_agent", sms.String, callback=new_agent_callback)

new_agent_pub = rp.Publisher("/new_agent", sms.String, queue_size=10)



def pose_callback(msg):
    global pose
    LOCK.acquire()
    pose = gmi.Pose(msg)
    LOCK.release()

rp.Subscriber(name="pose", data_class=gms.Pose, callback=pose_callback)

rp.wait_for_service(service="/assign_landmarks")
assign_landmarks_proxy = rp.ServiceProxy(name="/assign_landmarks", service_class=ccs.AssignLandmarks)

rp.wait_for_service(service="/return_landmarks")
return_landmarks_proxy = rp.ServiceProxy(name="/return_landmarks", service_class=ccs.ReturnLandmarks)

pub = rp.Publisher(name="cmd_twist", data_class=gms.Twist, queue_size=10)


stop = False
while not rp.is_shutdown() and not stop and rp.get_time() < INITIAL_TIME + DEPARTURE_TIME:
    LOCK.acquire()
    if not pose is None:
        pos_grad = gmi.Vector()
        ori_grad = gmi.Vector()
        if len(landmarks) is 0:
            new_landmarks = assign_landmarks_proxy.call(pose, NAME)
            for lmk_pose, id_, target_coverage, coverage in zip(new_landmarks.poses, new_landmarks.ids, new_landmarks.target_coverages, new_landmarks.coverages):
                landmarks[id_] = Landmark(gmi.Pose(lmk_pose), target_coverage, coverage)
        # if len(landmarks) is 0:
        #     stop = True
        if len(landmarks)>0:
            rate = FAST_RATE
            for landmark in landmarks.values():
                val, pg, og = footprint(pose, landmark.POSE)
                #rp.logwarn(val)
                landmark.coverage = val
                pos_grad += pg
                ori_grad += og
                #rp.logwarn(landmark)
            pos_grad *= GAIN/float(len(landmarks))
            ori_grad *= GAIN/float(len(landmarks))
            ori_grad = ori_grad.project_onto(gmi.E3)
        colliding_nbr = None
        collision_danger = 0.0
        for name, nbr in other_agents.items():
            if not nbr.pose is None:
                violation = collision_set.indicator_function(nbr.pose.position, pose.position)
                if (name < NAME or (len(landmarks) is 0)) and violation < collision_danger:
                    colliding_nbr = name
                    collision_danger = violation
                    rp.logwarn("@{}: Collision detected: {}".format(NAME, (nbr.pose.position-pose.position).norm))
                    #owo = collision_set.outward_orthogonal(nbr.pose.position, pose.position)
                    # if owo*pos_grad < 0:
                    #     owos.append(owo)
                #nbr.pose = None
        # if len(owos) > 2:
        #     pos_grad = gmi.Vector()
        # if len(owos) is 2:
        #     pos_grad = pos_grad.project_onto(owos[0].cross(owos[1]))
        # if len(owos) is 1:
        #     pos_grad -= pos_grad.project_onto(owos[0])
        if not colliding_nbr is None:
            pos_grad = -GAIN*collision_danger*collision_set.outward_orthogonal(other_agents[colliding_nbr].pose.position, pose.position)
            #rp.logwarn(pos_grad.norm)
        if feasible_set.indicator_function(pose.position) < 0.0:
            rp.logwarn("@{}: Out of the feasible cylinder: distance is {}".format(NAME, np.linalg.norm([pose.position.x, pose.position.y])))
            owo = feasible_set.outward_orthogonal(pose.position)
            if owo*pos_grad < 0:
                pos_grad -= pos_grad.project_onto(owo)
        pos_grad = pos_grad.saturate(SATURATION)
        ori_grad = ori_grad.saturate(0.3*SATURATION)
        # if pos_grad.norm < 1e-4:
        #     ids_to_return = list()
        #     coverages_to_return = list()
        #     for key, landmark in landmarks.items():
        #         ids_to_return.append(key)
        #         coverages_to_return.append(landmarks[key].coverage)
        #     if len(ids_to_return) > 0:
        #         return_landmarks_proxy.call(ids_to_return, coverages_to_return)
        #         rp.logwarn("@{} controller: I returned the landmarks {}".format(NAME, ids_to_return))
        #         for id_ in ids_to_return:
        #             landmarks.pop(id_)
        #     rate = SLOW_RATE
        pub.publish(pos_grad, ori_grad)
        #pose = None
        ids_to_return = list()
        coverages_to_return = list()
        for key, landmark in landmarks.items():
            if landmark.coverage > landmark.TARGET_COVERAGE:
                ids_to_return.append(key)
                coverages_to_return.append(landmarks[key].coverage)
        if len(ids_to_return) > 0:
            try:
                return_landmarks_proxy.call(ids_to_return, coverages_to_return)
                rp.logwarn("@{} controller: I returned the landmarks {}".format(NAME, ids_to_return))
                for id_ in ids_to_return:
                    landmarks.pop(id_)
            except:
                pass
    LOCK.release()
    new_agent_pub.publish(NAME)
    rate.sleep()

LOCK.acquire()
ids_to_return = list()
coverages_to_return = list()
for key, landmark in landmarks.items():
    ids_to_return.append(key)
    coverages_to_return.append(landmarks[key].coverage)
if len(ids_to_return) > 0:
    return_landmarks_proxy.call(ids_to_return, coverages_to_return)
    rp.logwarn("@{} controller: I returned the landmarks {}".format(NAME, ids_to_return))
for id_ in ids_to_return:
    landmarks.pop(id_)
LOCK.release()

rp.wait_for_service("/cancel_agent")
draw_agent_proxy = rp.ServiceProxy(name="/cancel_agent", service_class=ccs.CancelAgent)
draw_agent_proxy.call(NAME)
