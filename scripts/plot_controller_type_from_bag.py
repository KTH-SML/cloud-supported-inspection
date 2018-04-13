import rosbag as rb
import matplotlib.pyplot as plt
import sys


bag = rb.Bag(sys.argv[1])
database = dict()

for topic, msg, time in bag.read_messages(topics=['/BLUE/controller_type', '/GREEN/controller_type', '/RED/controller_type', '/MAGENTA/controller_type']):
    if not topic in database:
        database[topic] = (list(), list())
    database[topic][0].append(time.secs+time.nsecs*1e-9)
    database[topic][1].append(float(msg.data))
bag.close()

for topic, data in database.items():
    init_time = data[0][0]
    for index, time in enumerate(data[0]):
        data[0][index] -= init_time


fig, axes = plt.subplots(len(database), sharex=True, figsize=(6,4))
index = 0
for topic, data in database.items():
    plt.sca(axes[index])
    axes[index].plot(data[0], data[1])
    axes[index].set_ylim([-0.1, 1.1])
    axes[index].grid(True)
    plt.yticks([0.0, 1.0], [r"pose", r"cov"])
    index += 1
plt.savefig("controller_type.pdf")
plt.show()
