#! /usr/bin/env python

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import cloud_coverage.srv as ccs
import cloud_coverage.footprints as cfp
import cloud_coverage.feasible_sets as cfs
import rospy as rp
import cloud_coverage.landmarks_loader as cll

import threading as thd
import math as m
import numpy as np

import recordclass as rcc
import enum
import random as rdm






Landmark = rcc.recordclass('Landmark', ['POSE', 'TARGET_COVERAGE', 'coverage', 'contributed_coverage'])
Landmark.__new__.__defaults__ = 0.0, 0.0
landmarks = [Landmark(cll.POSES[i], cll.TARGETS[i]) for i in range(cll.NUM_LANDMARKS)]
reference_landmark = None



Agent = rcc.recordclass('Agent', ['POSE_SUBSCRIBER', 'pose'])
Agent.__new__.__defaults__ = None,

ControllerType = enum.Enum("ControllerType", "POSE COVERAGE")
active_controller = ControllerType.COVERAGE

pose = None
cmd_twist = None
last_cloud_connection_time = None

coverages = [0.0]*cll.NUM_LANDMARKS
contributed_coverages = [0.0]*cll.NUM_LANDMARKS
total_contribution = 0.0


#footprint = cfp.CompositeFootprint(best_distance=1.5)
#OFFSET = [float(elem) for elem in rp.get_param("/#offset").split()]
#rp.logwarn(OFFSET#)
#feasible_set = cfs#.FeasibleCylinder(radius=0.0, x0=OFFSET[0], y0=OFFSET[1])
#collision_set = cfs#.FeasibleSphere(radius=2.5)



"""Constants"""
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
GAIN = 0.5
SATURATION = 0.5
FOOTPRINT = cfp.EggFootprint()
FOOTPRINT = FOOTPRINT*FOOTPRINT*cfp.AlignmentFootprint()
FEASIBLE_SET = None
COLLISION_SET = None
CLOUD_DWELL_TIME = 5.0
CONTRIBUTION_THRESHOLD = 0.5

rp.wait_for_service(service="/cloud_access")
CLOUD_ACCESS_PROXY = rp.ServiceProxy(name="/cloud_access", service_class=ccs.CloudAccess)
cloud_accesses_counter = 0
cloud_accesses_pub = rp.Publisher("cloud_accesses", sms.Int32, queue_size=10)


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
controller_pub = rp.Publisher("controller_type", sms.Bool, queue_size=10)


def pose_callback(msg):
    global pose
    LOCK.acquire()
    pose = gmi.Pose(msg)
    LOCK.release()

rp.Subscriber(name="pose", data_class=gms.Pose, callback=pose_callback)

# rp.wait_for_service(service="/assign_landmarks")
# assign_landmarks_proxy = rp.ServiceProxy(name="/assign_landmarks", service_class=ccs.AssignLandmarks)

# rp.wait_for_service(service="/return_landmarks")
# return_landmarks_proxy = rp.ServiceProxy(name="/return_landmarks", service_class=ccs.ReturnLandmarks)

pub = rp.Publisher(name="cmd_twist", data_class=gms.Twist, queue_size=10)



def pose_controller():
    pos_grad = gmi.Vector()
    ori_grad = gmi.Vector()
    if not reference_landmark is None:
        ref_pose = reference_landmark.POSE
        pos_grad = GAIN*((ref_pose.position-pose.position)-pose.orientation.xh)
        ori_grad = 0.3*GAIN*(pose.orientation.xh.cross(ref_pose.orientation.xh))
    value = 0.0
    for landmark in landmarks:
        if landmark.coverage < landmark.TARGET_COVERAGE:
            val, _, _ = FOOTPRINT(pose, landmark.POSE)
            landmark.coverage += val
            landmark.contributed_coverage += val
            value += val
    return value, pos_grad, ori_grad


def coverage_controller():
    value = 0.0
    pos_grad = gmi.Vector()
    ori_grad = gmi.Vector()
    for landmark in landmarks:
        if landmark.coverage < landmark.TARGET_COVERAGE:
            val, pg, og = FOOTPRINT(pose, landmark.POSE)
            value += val
            landmark.coverage += val
            landmark.contributed_coverage += val
            pos_grad += pg
            ori_grad += og
    return value, pos_grad, ori_grad


def cloud_access():
    rp.logwarn(NAME + ": Accessing the cloud.")
    global last_cloud_connection_time, cloud_accesses_counter
    contributions = [lmk.contributed_coverage for lmk in landmarks]
    request = ccs.CloudAccessRequest(contributions)
    response = CLOUD_ACCESS_PROXY.call(request)
    for index, coverage in enumerate(response.coverages):
        landmarks[index].coverage = coverage
    last_cloud_connection_time = rp.get_time()
    cloud_accesses_counter += 1






inspection_complete = False
while not rp.is_shutdown() and rp.get_time() < INITIAL_TIME + DEPARTURE_TIME:
    LOCK.acquire()
    if not pose is None:
        active_landmarks = filter(lambda lmk: lmk.coverage < lmk.TARGET_COVERAGE, landmarks)
        if len(active_landmarks) is 0 and not inspection_complete:
            inspection_complete = True
            reference_landmark = None
            cloud_access()
        if active_controller is ControllerType.POSE:
            value, pos_grad, ori_grad = pose_controller()
            if value > 0.5:
                active_controller = ControllerType.COVERAGE
                reference_landmark = None
                rp.logwarn(NAME + ": Swithing to the coverage controller")
        if active_controller is ControllerType.COVERAGE:
            value, pos_grad, ori_grad = coverage_controller()
            if value < 0.01 and len(active_landmarks)>0:
                active_controller = ControllerType.POSE
                distances = [(lmk.POSE.position-pose.position).norm + 5*(1-lmk.POSE.orientation.xh*pose.orientation.xh) for lmk in active_landmarks]
                min_idx = np.argmin(distances)
                reference_landmark = active_landmarks[min_idx]
        total_contribution += value
        if (total_contribution > CONTRIBUTION_THRESHOLD*len(active_landmarks)) and (last_cloud_connection_time is None or rp.get_time() > last_cloud_connection_time + CLOUD_DWELL_TIME):
            cloud_access()
        # if len(landmarks) is 0:
        #     new_landmarks = assign_landmarks_proxy.call(pose, NAME, LANDMARK_CAPACITY)
        #     for lmk_pose, id_, target_coverage, coverage in zip(new_landmarks.poses, new_landmarks.ids, new_landmarks.target_coverages, new_landmarks.coverages):
        #         landmarks[id_] = Landmark(gmi.Pose(lmk_pose), target_coverage, coverage)
        # if len(landmarks) is 0:
        #     stop = True
        # if len(landmarks)>0:
        #     rate = FAST_RATE
        #     for landmark in landmarks.values():
        #         val, pg, og = footprint(pose, landmark.POSE)
        #         #rp.logwarn(val)
        #         landmark.coverage += val
        #         pos_grad += pg
        #         ori_grad += og
        #         #rp.logwarn(landmark)
        #num = len(filter(lambda lmk: lmk.coverage < lmk.TARGET_COVERAGE and FOOTPRINT(pose, lmk.POSE)[0]>0, landmarks)) + 1
        pos_grad *= GAIN*(1.0+len(active_landmarks))
        ori_grad *= 0.4*GAIN*(1.0+len(active_landmarks))
        ori_grad = ori_grad.project_onto(gmi.E3)
        # if not COLLISION_SET is None:
        #     colliding_nbr = None
        #     collision_danger = 0.0
        #     for name, nbr in other_agents.items():
        #         if not nbr.pose is None:
        #             violation = collision_set.indicator_function(nbr.pose.position, pose.position)
        #             if (name < NAME or (len(landmarks) is 0)) and violation < collision_danger:
        #                 colliding_nbr = name
        #                 collision_danger = violation
        #                 rp.logwarn("@{}: Collision detected: {}".format(NAME, (nbr.pose.position-pose.position).norm))
        #                 #owo = collision_set.outward_orthogonal(nbr.pose.position, pose.position)
        #                 # if owo*pos_grad < 0:
        #                 #     owos.append(owo)
        #             #nbr.pose = None
        #     # if len(owos) > 2:
        #     #     pos_grad = gmi.Vector()
        #     # if len(owos) is 2:
        #     #     pos_grad = pos_grad.project_onto(owos[0].cross(owos[1]))
        #     # if len(owos) is 1:
        #     #     pos_grad -= pos_grad.project_onto(owos[0])
        #     if not colliding_nbr is None:
        #         pos_grad = -GAIN*collision_danger*collision_set.outward_orthogonal(other_agents[colliding_nbr].pose.position, pose.position)
                #rp.logwarn(pos_grad.norm)
        # if not FEASIBLE_SET is None and FEASIBLE_SET.indicator_function(pose.position) < 0.0:
        #     rp.logwarn("@{}: Out of the feasible cylinder: distance is {}".format(NAME, np.linalg.norm([pose.position.x, pose.position.y])))
        #     owo = FEASIBLE_SET.outward_orthogonal(pose.position)
        #     if owo*pos_grad < 0:
        #         pos_grad -= pos_grad.project_onto(owo)
        pos_grad = pos_grad.saturate(SATURATION)
        ori_grad = ori_grad.saturate(0.4*SATURATION)
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
        controller_pub.publish(active_controller is ControllerType.COVERAGE)
        cloud_accesses_pub.publish(cloud_accesses_counter)
        #pose = None
        # ids_to_return = list()
        # coverages_to_return = list()
        # for index, landmark in enumerate(landmarks):
        #     if landmark.coverage > landmark.TARGET_COVERAGE:
        #         ids_to_return.append(key)
        #         coverages_to_return.append(landmarks[key].coverage)
        # if len(ids_to_return) > 0:
        #     try:
        #         return_landmarks_proxy.call(ids_to_return, coverages_to_return)
        #         rp.logwarn("@{} controller: I returned the landmarks {}".format(NAME, ids_to_return))
        #         for id_ in ids_to_return:
        #             landmarks.pop(id_)
        #     except:
        #         pass
    LOCK.release()
    new_agent_pub.publish(NAME)
    rate.sleep()

# LOCK.acquire()
# ids_to_return = list()
# coverages_to_return = list()
# for key, landmark in landmarks.items():
#     ids_to_return.append(key)
#     coverages_to_return.append(landmarks[key].coverage)
# if len(ids_to_return) > 0:
#     return_landmarks_proxy.call(ids_to_return, coverages_to_return)
#     rp.logwarn("@{} controller: I returned the landmarks {}".format(NAME, ids_to_return))
# for id_ in ids_to_return:
#     landmarks.pop(id_)
# LOCK.release()

# if inspection_complete:
#     rp.logwarn("{}: The inspection is complete!".format(NAME))

cloud_access()
rp.logwarn(NAME + ": Inspection complete!")
# rp.wait_for_service("/cancel_agent")
# draw_agent_proxy = rp.ServiceProxy(name="/cancel_agent", service_class=ccs.CancelAgent)
# draw_agent_proxy.call(NAME)
