#! /usr/bin/env python

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import cloud_coverage.srv as ccs

import rospy as rp
import threading as thd

import recordclass as rcc
import random as rdm

import numpy as np
import cloud_coverage.footprints as ftp



Landmark = rcc.recordclass('Landmark', ['POSE', 'TARGET_COVERAGE', 'coverage', 'assigned_agent'])
Landmark.__new__.__defaults__ = 0.0, None

FOOTPRINT = ftp.CompositeFootprint()
database = dict()
LOCK = thd.Lock()





rp.init_node("cloud")
RATE = rp.Rate(15.0)


















rp.wait_for_service(service="draw_landmarks")
draw_landmarks_proxy = rp.ServiceProxy(name="draw_landmarks", service_class=ccs.DrawLandmarks)

def add_landmarks_handler(request):
    LOCK.acquire()
    start_counter = len(database)
    counter = start_counter
    draw_landmarks_request = ccs.DrawLandmarksRequest()
    for pose, target in zip(request.poses, request.target_coverages):
        database[counter] = Landmark(gmi.Pose(pose), target)
        counter += 1
    ids = list(range(start_counter, counter))
    draw_landmarks_proxy.call(request.poses, ids, ["black"]*len(ids), [0.75]*len(ids))
    LOCK.release()
    return ccs.AddLandmarksResponse(ids)

rp.Service(name="add_landmarks", service_class=ccs.AddLandmarks, handler=add_landmarks_handler)






def forward_database_handler(request):
    LOCK.acquire()
    LOCK.release()
    return css.ForwardDatabaseResponse()








rp.wait_for_service(service="recolor_landmarks")
recolor_landmarks_proxy = rp.ServiceProxy(name="recolor_landmarks", service_class=ccs.RecolorLandmarks)

# def assign_landmarks_handler(request):
#     agent_pose = gmi.Pose(request.pose)
#     poses = list()
#     ids = list()
#     targets = list()
#     coverages = list()
#     capacity = request.capacity
#     LOCK.acquire()
#     try:
#         ids = []
#         for idx, lmk in database.items():
#             if not lmk.cleared and lmk.assigned_agent is None:
#                 ival = fp(agent_pose, lmk.POSE)[0]
#                 if len(ids) < request.capacity:
#                     ids.append(idx)
#                     poses.append(database[idx].POSE)
#                     targets.append(database[idx].TARGET_COVERAGE)
#                     coverages.append(database[idx].coverage)
#         #id_ = rdm.choice([key for key in database.keys() if not database[key].cleared and database[key].assigned_agent is None])
#         for id_ in ids:
#             database[id_].assigned_agent = request.name
#             rp.logwarn("@Cloud: Landmark {} assigned to {}".format(id_, request.name))
#             recolor_landmarks_proxy.call([id_], [request.name])
#         LOCK.release()
#         lmk = database[id_]
#         return ccs.AssignLandmarksResponse(poses=poses, ids=ids, target_coverages=targets, coverages=coverages)
#     except:
#         LOCK.release()
#         return ccs.AssignLandmarksResponse()
    # for id_, lmk in database.items():
    #     if not lmk.cleared:
    #         rp.logwarn("@Cloud: Landmark {} assigned to {}".format(id_, request.name))
    #         recolor_landmarks_proxy.call([id_], ["red"])
    #         LOCK.release()
    #         return ccs.AssignLandmarksResponse(poses=[lmk.POSE], ids=[id_], target_coverages=[lmk.TARGET_COVERAGE], coverages=[lmk.coverage])
    # LOCK.release()
    # return ccs.AssignLandmarksResponse()
    # for key, landmark in database.items():
    #     if not landmark.cleared:
    #         poses.append(landmark.POSE)
    #         ids.append(key)
    #         targets.append(landmark.TARGET_COVERAGE)
    #         coverages.append(landmark.coverage)
    #         landmark.assigned_agent = request.name
    #         rp.logwarn("@Cloud: Landmark {} assigned to {}".format(key, request.name))
    # LOCK.release()
    # recolor_landmarks_proxy.call(database.keys(), ["red"]*len(database.keys()))
    # return ccs.AssignLandmarksResponse(poses=poses, ids=ids, target_coverages=targets, coverages=coverages)

# rp.Service(name="assign_landmarks", service_class=ccs.AssignLandmarks, handler=assign_landmarks_handler, buff_size=10)

def cloud_access_handler(request):
    LOCK.acquire()
    for id_, contribution in zip(request.ids, request.contributions):
        database[id_].coverage += contribution
        database[id_].assigned_agent = None
    lmks_to_assign = list()
    for index, lmk in database.items():
        sns_pose = gmi.Pose(request.pose)
        if lmk.coverage < lmk.TARGET_COVERAGE and lmk.assigned_agent is None:
            idx = 0
            while idx < len(lmks_to_assign) and FOOTPRINT(sns_pose, lmk.POSE) < FOOTPRINT(sns_pose, database[lmks_to_assign[idx]].POSE):
                idx += 1
            if idx < len(lmks_to_assign):
                lmks_to_assign.insert(idx, index)
                lmks_to_assign = lmks_to_assign[0:request.capacity]
            elif len(lmks_to_assign) < request.capacity:
                lmks_to_assign.append(index)
    # idx = 0
    # while len(ids_to_assign) < request.capacity and idx < len(database):
    #     lmk = database[idx]
    #     if lmk.coverage < lmk.TARGET_COVERAGE and lmk.assigned_agent is None:
    #         ids_to_assign.add(idx)
    #     idx += 1
    poses = list()
    targets = list()
    coverages = list()
    for idx in lmks_to_assign:
        poses.append(database[idx].POSE)
        targets.append(database[idx].TARGET_COVERAGE)
        coverages.append(database[idx].coverage)
        database[idx].assigned_agent = request.name
    LOCK.release()
    recolor_landmarks_proxy.call(request.ids, ["black"]*len(request.ids), [0.25]*len(request.ids))
    recolor_landmarks_proxy.call(lmks_to_assign, [request.name]*len(lmks_to_assign), [0.5]*len(lmks_to_assign))
    return ccs.CloudAccessResponse(lmks_to_assign, poses, targets, coverages)

rp.Service(name="cloud_access", service_class=ccs.CloudAccess, handler=cloud_access_handler)


# rp.wait_for_service(service="cancel_landmarks")
# cancel_landmarks_proxy = rp.ServiceProxy(name="cancel_landmarks", service_class=ccs.CancelLandmarks)

# def return_landmarks_handler(request):
#     cleared_landmarks = list()
#     uncleared_landmarks = list()
#     for id_, new_coverage in zip(request.ids, request.coverages):
#         database[id_].coverage = new_coverage
#         if new_coverage > database[id_].TARGET_COVERAGE:
#             database[id_].cleared = True
#             cleared_landmarks.append(id_)
#         else:
#             uncleared_landmarks.append(id_)
#             database[id_].assigned_agent = None
#             database[id_].coverage = 0.0
#         rp.logwarn("@Cloud: Landmark {} returned by {}".format(id_, database[id_].assigned_agent))
#         database[id_].assigned_agent = None
#     #cancel_landmarks_proxy.call(cleared_landmarks)
#     recolor_landmarks_proxy.call(uncleared_landmarks, ["black"]*len(uncleared_landmarks))
#     #recolor_landmarks_proxy.call(cleared_landmarks, ["green"]*len(cleared_landmarks))
#     return ccs.ReturnLandmarksResponse()
#
# rp.Service(name="return_landmarks", service_class=ccs.ReturnLandmarks, handler=return_landmarks_handler, buff_size=10)




# proxy = rp.ServiceProxy(name="add_landmarks", service_class=ccs.AddLandmarks)
# request = ccs.AddLandmarksRequest()
# request.poses = [gmi.Pose(position=gmi.Point(0,1,0), orientation=gmi.UnitQuaternion()), gmi.Pose(position=gmi.Point(0,1,1), orientation=gmi.UnitQuaternion())]
# request.target_coverages = [10.0, 5.0]
# proxy.call(request)


pub = rp.Publisher("/coverage_errors", sms.Float64MultiArray, queue_size=10)
total_pub = rp.Publisher("/total_coverage_error", sms.Float64, queue_size=10)
while not rp.is_shutdown():
    LOCK.acquire()
    data = [0.0]*len(database)
    total = 0.0
    for index, lmk in database.items():
        err = max((lmk.TARGET_COVERAGE - lmk.coverage, 0.0))
        data[index] = err
        total += err
    msg = sms.Float64MultiArray(data=data)
    pub.publish(msg)
    total_pub.publish(total)
    LOCK.release()
    RATE.sleep()
