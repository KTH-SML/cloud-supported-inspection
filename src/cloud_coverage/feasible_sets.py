import geomthree.impl as gmi
import math as m
import numpy as np

class _FeasibleSet:

    def __init__(self):
        pass

    def indicator_function(self, pose):
        raise NotImplementedError()

    def outward_orthogonal(self, point):
        raise NotImplementedError()





class FeasibleCylinder(_FeasibleSet):

    def __init__(self, radius=1.0, x0=0.0, y0=0.0):
        self._RADIUS = radius
        self._X0 = x0
        self._Y0 = y0
        _FeasibleSet.__init__(self)

    def indicator_function(self, point):
        return np.linalg.norm([point.x-self._X0, point.y-self._Y0]) - self._RADIUS

    def outward_orthogonal(self, point):
        vec = gmi.Vector(point.x-self._X0, point.y-self._Y0, 0.0)
        return vec/vec.norm


class FeasibleHalfCylinder(_FeasibleSet):

        def __init__(self, radius = 1.0):
            self._RADIUS = radius
            _FeasibleSet.__init__(self)

        def indicator_function(self, point):
            if -self._RADIUS < point.x < self._RADIUS:
                return np.linalg.norm([point.x, point.y]) - self._RADIUS
            else: return point.y

        def outward_orthogonal(self, point):
            if -self._RADIUS < point.x < self._RADIUS:
                vec = gmi.Vector(point.x, point.y, 0.0)
                return vec/vec.norm
            else: return Vector(0,1,0)




class FeasibleSphere(_FeasibleSet):

    def __init__(self, radius = 1.0):
        self._RADIUS = radius
        _FeasibleSet.__init__(self)

    def indicator_function(self, center, point):
        return (point - center).norm - self._RADIUS

    def outward_orthogonal(self, center, point):
        vec = point - center
        return vec/vec.norm
