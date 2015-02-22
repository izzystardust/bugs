import threading
import sys

class Dist:
    def __init__(self):
        self.m = threading.Lock()
        self.left = 0
        self.front = 0

    def update(self, vals, range_min, range_max):
        # these magic numbers were acquired from Alan Beadle
        # straight ahead is 540, 40 index range should be enough
        # left chosen to look slightly back to get in front of wall before turning

        def getmin(a, b):
            in_rng = lambda x: range_min <= x <= range_max
            vsp = filter(in_rng, vals[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        newfront = getmin(500, 581)
        newleft = getmin(740, 851)

        self.m.acquire()
        self.left = newleft
        self.front = newfront
        self.m.release()

    def get(self):
        self.m.acquire()
        l = self.left
        f = self.front
        self.m.release()
        return (f, l)
