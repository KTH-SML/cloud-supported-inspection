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

    def __init__(self, radius = 1.0):
        self._RADIUS = radius
        _FeasibleSet.__init__(self)

    def indicator_function(self, point):
        return np.linalg.norm([point.x, point.y]) - self._RADIUS

    def outward_orthogonal(self, point):
        vec = gmi.Vector(point.x, point.y, 0.0)
        return vec/vec.norm



class FeasibleSphere(_FeasibleSet):

    def __init__(self, radius = 1.0):
        self._RADIUS = radius
        _FeasibleSet.__init__(self)

    def indicator_function(self, center, point):
        return (point - center).norm - self._RADIUS

    def outward_orthogonal(self, center, point):
        vec = point - center
        return vec/vec.norm
