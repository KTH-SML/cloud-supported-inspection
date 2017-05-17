import geomthree.impl as gmi
import recordclass as rcc



Landmark = rcc.recordclass('Landmark', ['POSE', 'TARGET_COVERAGE', 'coverage'])
Landmark.__new__.__defaults__ = 0.0,



if __name__ == "__main__":
    lmk = Landmark(POSE=gmi.Pose(), ID=3, TARGET_COVERAGE=3.0)
    print lmk
