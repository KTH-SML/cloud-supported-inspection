import geomthree.impl as gmi
import math as m
import rospy as rp
import matplotlib.pyplot as plt
import numpy as np
#import matplotlib2tikz as m2t
#plt.switch_backend('TKagg')


class _AbstractFootprint:

    def __init__(self):
        raise NotImplementedError()

    def __call__(self, sensor, landmark):
        raise NotImplementedError()

    def contour_plot(self, xlim=(-1,1), ylim=(-1,1), num=30, levels=(0.2, 0.5, 1-0.5**2, 1-0.5**3, 1-0.5**4, 1-0.5**5)):
        sns = gmi.Pose()
        qx = np.linspace(xlim[0], xlim[1], num=num)
        qy = np.linspace(ylim[0], ylim[1], num=num)
        values = np.zeros((num,num))
        for index in range(num):
            for jndex in range(num):
                lmk = gmi.Pose(gmi.Point(qx[index],qy[jndex],0), sns.orientation)
                values[jndex,index] = self(sns, lmk)[0]
        mqx, mqy = np.meshgrid(qx, qy)
        ctp = plt.contour(mqx, mqy, values, levels=levels, cmap="jet")
        cbar = plt.colorbar(ctp, ticks=levels, orientation='horizontal')
        return ctp, cbar




class CompositeFootprint(_AbstractFootprint):

    def __init__(self, best_distance=1.0, front_gain=0.05, rear_gain=0.75):
        self._BEST_DISTANCE = best_distance
        self._FRONT_GAIN = front_gain
        self._REAR_GAIN = rear_gain

    def __call__(self, sensor, landmark):
        nh = sensor.orientation.xh
        mh = landmark.orientation.xh
        RG, FG = self._REAR_GAIN, self._FRONT_GAIN
        vec = sensor.position + self._BEST_DISTANCE*mh - landmark.position
        nrm = vec.norm
        pos_grad = vec
        if nrm > 0.0:
            pos_grad += (nrm*mh - (vec*mh)/nrm*vec)
        ori_grad = nh.cross(mh)
        penalty = (2-nh*mh)*((RG+FG)*vec.norm**2 + (RG-FG)*vec.norm*vec*nh)
        value = m.exp(-penalty)
        return value, -pos_grad, ori_grad




class SplineFootprint(_AbstractFootprint):

    def __init__(self, peak=1.0, radius=5.0):
        self._PEAK = peak
        self._RADIUS = radius
        self._RISE_COEFFICIENTS = (0.0, 0.0, 3/peak**2, -2/peak**3)
        self._DROP_COEFFICIENTS = ((radius**3-3*peak*radius**2)/(radius-peak)**3, 6*peak*radius/(radius-peak)**3, -3*(radius+peak)/(radius-peak)**3, 2/(radius-peak)**3)

    def spline(self, x):
        if x < 0.0:
            return 0.0
        if 0.0 < x < self._PEAK:
            return sum([coeff*x**index for index, coeff in enumerate(self._RISE_COEFFICIENTS)])
        if self._PEAK < x < self._RADIUS:
            return sum([coeff*x**index for index, coeff in enumerate(self._DROP_COEFFICIENTS)])
        return 0.0

    def __call__(self, sensor, landmark):
        p = sensor.position
        nh = sensor.orientation.xh
        q = landmark.position
        mh = landmark.orientation.xh
        vec = q-p
        if (vec*nh) <= 0.0 or (nh*mh) <= 0.0:
            return 0.0, gmi.Vector(), gmi.Vector()
        return self.spline(vec.norm)*(vec*nh)/vec.norm*(nh*mh), gmi.Vector(), gmi.Vector()








if __name__ == "__main__":
    fpt = SplineFootprint()
    print fpt(gmi.Pose(), gmi.Pose(gmi.Point(), gmi.UnitQuaternion()))
    print fpt(gmi.Pose(), gmi.Pose(gmi.Point(1,0,0), gmi.UnitQuaternion()))
    print fpt(gmi.Pose(), gmi.Pose(gmi.Point(2,0,0), gmi.UnitQuaternion()))
    plt.figure()
    x = np.linspace(-1,8,200)
    plt.plot(x, [fpt.spline(xi) for xi in x])
    plt.grid(linestyle="dashed")
    plt.xlabel("$x$")
    plt.ylabel("$f$")
    m2t.save("spline.tex")

    plt.figure()
    ctp = fpt.contour_plot(xlim=(-1,5), ylim=(-3,3))
    plt.grid(linestyle="dashed")
    plt.xlabel("$q_x$")
    plt.ylabel("$q_y$")
    m2t.save("contour.tex")

    plt.show()
