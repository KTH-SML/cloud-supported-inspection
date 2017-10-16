import rosbag as rb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

trajectories = dict()

bag = rb.Bag(sys.argv[1])
for topic, msg, time in bag.read_messages(topics=['/BLUE/pose', '/RED/pose', '/GREEN/pose', '/MAGENTA/pose']):
    if not topic in trajectories:
        trajectories[topic] = list()
    trajectories[topic].append([time.secs, msg.position.x, msg.position.y, msg.position.z])
bag.close()

fig = plt.figure()
plt.grid(True)
ax = fig.gca(projection="3d")
ax.set_xlabel(r"$x$")
ax.set_ylabel(r"$y$")
ax.set_zlabel(r"$z$")
for topic, traj in trajectories.items():
    ts = [point[0] for point in traj]
    xs = [point[1] for point in traj]
    ys = [point[2] for point in traj]
    zs = [point[3] for point in traj]
    ax.plot(xs, ys, zs=zs, color=topic.split('/')[1])
plt.show()
