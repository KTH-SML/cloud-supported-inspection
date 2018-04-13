#! /usr/bin/env python
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.cm as mcm

import geomthree.impl as gmi
import geometry_msgs.msg as gms
import cloud_coverage.srv as ccs
import rospy as rp

import threading as thd
import recordclass as rcc


mpl.rc('text', usetex=True)

Landmark = rcc.recordclass('Landmark', ['POSE', 'artists'])
Landmark.__new__.__defaults__ = None

IncomingLandmark = rcc.recordclass('IncomingLandmark', ['POSE'])

LANDMARK_LOCK = thd.Lock()
incoming_landmarks = dict()
landmarks = dict()
landmarks_to_cancel = list()
landmarks_to_recolor = dict()

Agent = rcc.recordclass('Agent', ['SUBSCRIBER', 'pose', 'artists'])
Agent.__new__.__defaults__ = None, None, None

AGENT_LOCK = thd.Lock()
agents = dict()
agents_to_cancel = list()

OFFSET = [float(elem) for elem in rp.get_param("/offset").split()]
#OFFSET = (16.5, -4.4)
COLORMAP = mcm.jet

plt.ion()
fig = plt.figure(figsize=(15,15))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel(r"$x$")
ax.set_ylabel(r"$y$")
ax.set_zlabel(r"$z$")
#ax.view_init(90, 0)
ax.set_autoscalex_on(False)
ax.set_xlim([-4+OFFSET[0], 4+OFFSET[0]])
ax.set_autoscaley_on(False)
ax.set_ylim([-4+OFFSET[1], 4+OFFSET[1]])
ax.set_zlim([0, 8])
#ax.autoscale_view()
ax.set_aspect("equal")
plt.draw()

rp.init_node("plotter")

FREQUENCY = 10.0
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1/FREQUENCY

pose = None






def draw_landmarks_handler(request):
    global incoming_landmarks
    LANDMARK_LOCK.acquire()
    for index, pose in enumerate(request.poses):
        incoming_landmarks[index] = IncomingLandmark(gmi.Pose(pose))
    LANDMARK_LOCK.release()
    return ccs.DrawLandmarksResponse()

rp.Service(name="draw_landmarks", service_class=ccs.DrawLandmarks, handler=draw_landmarks_handler)

def cancel_landmarks_handler(request):
    global landmarks_to_cancel
    LANDMARK_LOCK.acquire()
    landmarks_to_cancel += request.ids
    LANDMARK_LOCK.release()
    return ccs.CancelLandmarksResponse()

rp.Service(name="cancel_landmarks", service_class=ccs.CancelLandmarks, handler=cancel_landmarks_handler)


def recolor_landmarks_handler(request):
    global landmarks_to_recolor
    LANDMARK_LOCK.acquire()
    for id_, level in zip(request.ids, request.levels):
        landmarks_to_recolor[id_] = level
    LANDMARK_LOCK.release()
    return ccs.RecolorLandmarksResponse()

rp.Service(name="recolor_landmarks", service_class=ccs.RecolorLandmarks, handler=recolor_landmarks_handler, buff_size=10)



def pose_callback(msg, name):
    global agents
    AGENT_LOCK.acquire()
    agents[name].pose = gmi.Pose(msg)
    AGENT_LOCK.release()

def draw_agent_handler(req):
    global agents
    AGENT_LOCK.acquire()
    sub = rp.Subscriber(name=req.name+"/pose", data_class=gms.Pose, callback=pose_callback, callback_args=req.name)
    agents[req.name] = Agent(sub)
    AGENT_LOCK.release()
    return ccs.DrawAgentResponse()

rp.Service(name="draw_agent", service_class=ccs.DrawAgent, handler=draw_agent_handler)



def cancel_agent_handler(req):
    global agents_to_cancel
    AGENT_LOCK.acquire()
    agents_to_cancel.append(req.name)
    AGENT_LOCK.release()
    return ccs.CancelAgentResponse()

rp.Service(name="cancel_agent", service_class=ccs.CancelAgent, handler=cancel_agent_handler)


SNAPSHOT_TIMES = [2,20,40,60]
snapshot_index = 0
INITIAL_TIME = rp.get_time()

while not rp.is_shutdown():
    LANDMARK_LOCK.acquire()
    while incoming_landmarks:
        id_, incoming_landmark = incoming_landmarks.popitem()
        landmarks[id_] = Landmark(POSE=incoming_landmark.POSE, artists=incoming_landmark.POSE.draw(color=COLORMAP(1.0), show_x=False, show_y=False, show_z=False, alpha=0.75))
    processed_recolors = list()
    while landmarks_to_recolor:
        id_, level = landmarks_to_recolor.popitem()
        lmk = landmarks.get(id_, None)
        if not lmk is None:
            for artist in lmk.artists:
                artist.remove()
            lmk.artists = lmk.POSE.draw(color=COLORMAP(level), show_x=False, show_y=False, show_z=False, alpha=0.75)
    while landmarks_to_cancel:
        id_ = landmarks_to_cancel.pop()
        lmk = landmarks.get(id_, None)
        if not lmk is None:
            for artist in lmk.artists:
                artist.remove()
            landmarks.pop(id_)
    LANDMARK_LOCK.release()
    AGENT_LOCK.acquire()
    for name in agents_to_cancel:
        agents[name].SUBSCRIBER.unregister()
        if not agents[name].artists is None:
            for artist in agents[name].artists:
                artist.remove()
        agents.pop(name)
    agents_to_cancel = list()
    for name, agent in agents.items():
        if not agent.pose is None:
            if not agent.artists is None:
                [artist.remove() for artist in agent.artists]
                agent.artists = None
            agent.artists = agent.pose.draw(color="black")
            agent.pose = None
    AGENT_LOCK.release()
    if snapshot_index < len(SNAPSHOT_TIMES) and rp.get_time()-INITIAL_TIME > SNAPSHOT_TIMES[snapshot_index]:
        plt.savefig("./snapshot"+str(snapshot_index)+".pdf")
        rp.logwarn("Snaphot taken!")
        snapshot_index += 1
    plt.draw()
    RATE.sleep()
