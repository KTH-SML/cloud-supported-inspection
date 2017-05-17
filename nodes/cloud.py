#! /usr/bin/env python

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import cloud_coverage.srv as ccs

import rospy as rp
import threading as thd

import recordclass as rcc
import random as rdm

import numpy as np
import cloud_coverage.footprints as ftp



Landmark = rcc.recordclass('Landmark', ['POSE', 'TARGET_COVERAGE', 'coverage', 'assigned_agent', 'cleared'])
Landmark.__new__.__defaults__ = 0.0, None, False

fp = ftp.CompositeFootprint()

database = dict()
counter = 0

LOCK = thd.Lock()




rp.init_node("cloud")



















rp.wait_for_service(service="draw_landmarks")
draw_landmarks_proxy = rp.ServiceProxy(name="draw_landmarks", service_class=ccs.DrawLandmarks)

def add_landmarks_handler(request):
    global counter
    LOCK.acquire()
    start_counter = counter
    draw_landmarks_request = ccs.DrawLandmarksRequest()
    for pose, target in zip(request.poses, request.target_coverages):
        database[counter] = Landmark(gmi.Pose(pose), target)
        counter += 1
    ids = list(range(start_counter, counter))
    draw_landmarks_proxy.call(request.poses, ids, ["black"]*len(ids))
    LOCK.release()
    return ccs.AddLandmarksResponse(ids)

rp.Service(name="add_landmarks", service_class=ccs.AddLandmarks, handler=add_landmarks_handler)






rp.wait_for_service(service="recolor_landmarks")
recolor_landmarks_proxy = rp.ServiceProxy(name="recolor_landmarks", service_class=ccs.RecolorLandmarks)

def assign_landmarks_handler(request):
    agent_pose = gmi.Pose(request.pose)
    poses = list()
    ids = list()
    targets = list()
    coverages = list()
    LOCK.acquire()
    try:
        id_ = None
        val = 0.0
        for idx, lmk in database.items():
            if not lmk.cleared and lmk.assigned_agent is None:
                ival = fp(agent_pose, lmk.POSE)[0]
                if ival > val:
                    id_ = idx
                    val = ival
        #id_ = rdm.choice([key for key in database.keys() if not database[key].cleared and database[key].assigned_agent is None])
        database[id_].assigned_agent = request.name
        rp.logwarn("@Cloud: Landmark {} assigned to {}".format(id_, request.name))
        recolor_landmarks_proxy.call([id_], [request.name])
        LOCK.release()
        lmk = database[id_]
        return ccs.AssignLandmarksResponse(poses=[lmk.POSE], ids=[id_], target_coverages=[lmk.TARGET_COVERAGE], coverages=[lmk.coverage])
    except:
        LOCK.release()
        return ccs.AssignLandmarksResponse()
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

rp.Service(name="assign_landmarks", service_class=ccs.AssignLandmarks, handler=assign_landmarks_handler, buff_size=10)






rp.wait_for_service(service="cancel_landmarks")
cancel_landmarks_proxy = rp.ServiceProxy(name="cancel_landmarks", service_class=ccs.CancelLandmarks)

def return_landmarks_handler(request):
    cleared_landmarks = list()
    uncleared_landmarks = list()
    for id_, new_coverage in zip(request.ids, request.coverages):
        database[id_].coverage = new_coverage
        if new_coverage > database[id_].TARGET_COVERAGE:
            database[id_].cleared = True
            cleared_landmarks.append(id_)
        else:
            uncleared_landmarks.append(id_)
            database[id_].assigned_agent = None
            database[id_].coverage = 0.0
        rp.logwarn("@Cloud: Landmark {} returned by {}".format(id_, database[id_].assigned_agent))
        database[id_].assigned_agent = None
    #cancel_landmarks_proxy.call(cleared_landmarks)
    recolor_landmarks_proxy.call(uncleared_landmarks, ["black"]*len(uncleared_landmarks))
    #recolor_landmarks_proxy.call(cleared_landmarks, ["green"]*len(cleared_landmarks))
    return ccs.ReturnLandmarksResponse()

rp.Service(name="return_landmarks", service_class=ccs.ReturnLandmarks, handler=return_landmarks_handler, buff_size=10)




# proxy = rp.ServiceProxy(name="add_landmarks", service_class=ccs.AddLandmarks)
# request = ccs.AddLandmarksRequest()
# request.poses = [gmi.Pose(position=gmi.Point(0,1,0), orientation=gmi.UnitQuaternion()), gmi.Pose(position=gmi.Point(0,1,1), orientation=gmi.UnitQuaternion())]
# request.target_coverages = [10.0, 5.0]
# proxy.call(request)

rp.spin()
