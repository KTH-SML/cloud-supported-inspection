import geomthree.impl as gmi
import math as m
import rospy as rp
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mp3
import numpy as np
#import matplotlib2tikz as m2t
#plt.switch_backend('TKagg')


class _AbstractFootprint:

    def __init__(self):
        pass

    def __call__(self, sensor, landmark):
        raise NotImplementedError()

    def generate_plot_data(self, xlim=(-1,1), ylim=(-1,1), num=50):
        sns = gmi.Pose()
        qx = np.linspace(xlim[0], xlim[1], num=num)
        qy = np.linspace(ylim[0], ylim[1], num=num)
        values = np.zeros((num,num))
        for index in range(num):
            for jndex in range(num):
                lmk = gmi.Pose(gmi.Point(qx[index],qy[jndex],0), sns.orientation)
                values[jndex,index] = self(sns, lmk)[0]
        mqx, mqy = np.meshgrid(qx, qy)
        return mqx, mqy, values

    def contour_plot(self, mqx, mqy, values, levels=(0.2, 0.5, 1-0.5**2, 1-0.5**3, 1-0.5**4, 1-0.5**5), cmap="jet"):
        ctp = plt.contour(mqx, mqy, values, levels=levels, cmap=cmap)
        cbar = plt.colorbar(ctp, ticks=levels, orientation='horizontal')
        plt.grid()

    def wireframe_plot(self, mqx, mqy, values):
        mp3.Axes3D(plt.gcf()).plot_wireframe(mqx, mqy, values)
        plt.grid()


    def __mul__(self, other):
        def fun(sen, lan):
            val1, pg1, og1 = self(sen, lan)
            val2, pg2, og2 = other(sen, lan)
            val = val1*val2
            pg = pg1*val2 + pg2*val1
            og = og1*val2 + og2*val1
            return val, pg, og
        return ProductFootprint(fun)



class ProductFootprint(_AbstractFootprint):

    def __init__(self, fun):
        _AbstractFootprint.__init__(self)
        self.__FUN = fun

    def __call__(self, sen, lan):
        return self.__FUN(sen, lan)





class EggFootprint(_AbstractFootprint):

    def __init__(self, small_radius=1.0, big_radius=3.0):
        self.__SMALL_RADIUS = small_radius
        self.__BIG_RADIUS = big_radius

    def __call__(self, sns, lmk):
        R = self.__BIG_RADIUS
        r = self.__SMALL_RADIUS
        q = lmk.position
        p = sns.position
        nh = sns.orientation.xh
        vec = (q-p-r*nh)
        xi = vec*nh
        zeta2_mi2 = vec.norm**2 - xi**2
        if xi > 0:
            val = 1.0 - vec.norm**2 / R**2
            pg = 2*vec/R**2
            og = 2*vec/R**2
            if val < 0.0:
                return 0.0, gmi.Vector(), gmi.Vector()
            else:
                return val, pg, gmi.Vector()
        else:
            val = 1.0 - xi**2/r**2 - zeta2_mi2/R**2
            pg = 2*xi*(1/r**2-1/R**2)*nh + 2/R**2*vec
            og = (2*xi*(1/R**2-1/r**2)+2/R**2)*vec
            if val < 0.0:
                return 0.0, gmi.Vector(), gmi.Vector()
            else:
                return val, pg, gmi.Vector()



class AlignmentFootprint(_AbstractFootprint):

    def __init__(self):
        _AbstractFootprint.__init__(self)

    def __call__(self, sns, lmk):
        return max((0.0, sns.orientation.xh*lmk.orientation.xh)), gmi.Vector(), sns.orientation.xh.cross(lmk.orientation.xh)



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

    efp = EggFootprint(big_radius=4.0)
    alg = AlignmentFootprint()
    efp = efp*efp*alg
    mqx, mqy, values = efp.generate_plot_data(xlim=(-1,5), ylim=(-5,5))
    plt.figure()
    efp.wireframe_plot(mqx, mqy, values)

    plt.figure()
    efp.contour_plot(mqx, mqy, values)

    # fpt = SplineFootprint()
    # mqx, mqy, values = fpt.generate_plot_data(xlim=(-1,5), ylim=(-3,3))
    # #plt.figure()
    # #fpt.contour_plot(mqx, mqy, values)
    # plt.figure()
    # fpt.wireframe_plot(mqx, mqy, values)
    #
    # fpt2 = CompositeFootprint(best_distance=1.0, front_gain=0.05, rear_gain=0.75)
    #
    # plt.figure()
    # mqx, mqy, values = fpt2.generate_plot_data(xlim=(-1,5), ylim=(-3,3))
    # fpt2.wireframe_plot(mqx, mqy, values)
    #
    # pfpt = fpt*fpt2
    #
    # plt.figure()
    # mqx, mqy, values = pfpt.generate_plot_data(xlim=(-1,5), ylim=(-3,3))
    # pfpt.wireframe_plot(mqx, mqy, values)

    # print fpt(gmi.Pose(), gmi.Pose(gmi.Point(), gmi.UnitQuaternion()))
    # print fpt(gmi.Pose(), gmi.Pose(gmi.Point(1,0,0), gmi.UnitQuaternion()))
    # print fpt(gmi.Pose(), gmi.Pose(gmi.Point(2,0,0), gmi.UnitQuaternion()))
    # plt.figure()
    # x = np.linspace(-1,8,200)
    # plt.plot(x, [fpt.spline(xi) for xi in x])
    # plt.grid(linestyle="dashed")
    # plt.xlabel("$x$")
    # plt.ylabel("$f$")
    # m2t.save("spline.tex")
    #
    # plt.figure()
    # ctp = fpt.contour_plot(xlim=(-1,5), ylim=(-3,3))
    # plt.grid(linestyle="dashed")
    # plt.xlabel("$q_x$")
    # plt.ylabel("$q_y$")
    # m2t.save("contour.tex")

    plt.show()
