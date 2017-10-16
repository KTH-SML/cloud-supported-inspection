import rosbag as rb
import matplotlib.pyplot as plt
import sys


bag = rb.Bag(sys.argv[1])
times = list()
errors = list()
for topic, msg, time in bag.read_messages(topics=['/total_coverage_error']):
    times.append(time.secs+time.nsecs*1e-9)
    errors.append(msg.data*1e-3)
bag.close()

INITIAL_TIME = times[0]
for index, time in enumerate(times):
    times[index] = time-INITIAL_TIME


plt.figure()
plt.grid(True)
plt.plot(times, errors)
plt.show()
