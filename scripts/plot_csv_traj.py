import math as m
import matplotlib as mpl
import matplotlib.pyplot as plt

import sys

filename = sys.argv[1]

ts=list()
xs=list()
ys=list()
zs=list()
with open("./"+filename) as infile:
    line = infile.readline()
    while line:
        line = infile.readline()
        if line:
            t,x,y,z,yaw = [float(elem) for elem in line.split(",")]
            xs.append(x)
            ys.append(y)
            zs.append(z)
            ts.append(t)

OFFSET = (16.2, -3.5)
#OFFSET = (0,0)

fig = plt.figure(figsize=(15,15))
ax = fig.gca()
ax.set_xlabel(r"$x$")
ax.set_ylabel(r"$y$")
#ax.set_zlabel(r"$z$")
#ax.view_init(90, 0)
ax.set_autoscalex_on(False)
ax.set_xlim([-10.0+OFFSET[0], 10.0+OFFSET[0]])
ax.set_autoscaley_on(False)
ax.set_ylim([-10.0+OFFSET[1], 10.0+OFFSET[1]])
#ax.set_zlim([3.0, 12.0])
#ax.autoscale_view()
plt.plot(xs,ys)
ax.set_aspect("equal")


fig = plt.figure(figsize=(15,15))
ax = fig.gca()
ax.set_xlabel(r"$t$")
ax.set_ylabel(r"$x$")
plt.plot(ts,xs)

fig = plt.figure(figsize=(15,15))
ax = fig.gca()
ax.set_xlabel(r"$t$")
ax.set_ylabel(r"$y$")
plt.plot(ts,ys)

fig = plt.figure(figsize=(15,15))
ax = fig.gca()
ax.set_xlabel(r"$t$")
ax.set_ylabel(r"$z$")
plt.plot(ts,zs)
plt.show()
