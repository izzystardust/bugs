import threading
import math
import sys

class Location:
    def __init__(self):
        self.m = threading.Lock()
        self.x = None
        self.y = None
	self.t = None

    def update_location(self, x, y, t):
        self.m.acquire()
        self.x = x
        self.y = y
	self.t = t
        self.m.release()

    def current_location(self):
        self.m.acquire()
        x = self.x
        y = self.y
	t = self.t
        self.m.release()
        return (x, y, t)

    def distance(self, x, y):
        (x0, y0, _) = self.current_location()
        if x0 == None or y0 == None:
            return sys.maxint
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    def required_heading(self, x, y):
            pass
