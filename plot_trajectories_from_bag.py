import rosbag as rb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

trajectories = dict()

bag = rb.Bag('ecc17.bag')
for topic, msg, time in bag.read_messages(topics=['/BLUE/pose', '/RED/pose', '/GREEN/pose', '/MAGENTA/pose']):
    if not topic in trajectories:
        trajectories[topic] = list()
    trajectories[topic].append([time.secs, msg.position.x, msg.position.y, msg.position.z])
bag.close()

fig = plt.figure()
plt.grid(True)
ax = fig.gca()
for topic, traj in trajectories.items():
    ts = [point[0] for point in traj]
    xs = [point[1] for point in traj]
    ys = [point[2] for point in traj]
    zs = [point[3] for point in traj]
    ax.plot(ts, xs, color=topic.split('/')[1])
plt.show()
