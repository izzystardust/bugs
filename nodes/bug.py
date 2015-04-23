#!/usr/bin/env python

import math
import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from location import Location, necessary_heading
from dist import Dist

current_location = Location()
current_dists = Dist()

delta = .1
WALL_PADDING = .5

# These constants are passed to bug.go()
STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

# charger (x, y) locations
chargers = [
    (0, 14),
    (2, 2),
    (-18, 3),
    (6, -10),
]

# returns the euclidean_distance between (x1, y1) and (x2, y2)
def euclidean_distance(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def closest_in(points, x, y):
    ds = map(lambda p: euclidean_distance(p, (x, y)), points)
    return filter(lambda p: p[0] == min(ds), zip(ds, points))[0][1]

def init_listener():
    # Set up all of the ROS subscriptions we'll listen to
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)
    rospy.Subscriber('base_scan', LaserScan, lambda d: current_dists.update(d))

def location_callback(data):
    p = data.pose.pose.position
    # transform.euler_from_quaternion() takes a tuple
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2] # (x, y, z)[2] in [-pi, pi]
    current_location.update_location(p.x, p.y, t)


class Bug:
    def __init__(self, tx, ty):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.initial = (None, None)
        self.target = (tx, ty)
        self.bat = 100
        self.speed = .5
        self.state = "GO_UNTIL_OBSTACLE"
        self.states = {
            'GO_UNTIL_OBSTACLE': lambda x: x.go_until_obstacle(),
            'FOLLOW_WALL': lambda x: x.follow_wall(),
            'GOTO_CHARGER': lambda x: x.goto_charger(),
        }
        self.temp_loc = (None, None) # Used to store target when seeking charger

    def go_until_obstacle(self):
        (front, _) = current_dists.get()
        if front <= WALL_PADDING:
            return "FOLLOW_WALL"
        if current_location.facing_point(*self.target):
            self.go(STRAIGHT, self.speed)
        elif current_location.faster_left(*self.target):
            self.go(LEFT, self.speed)
        else:
            self.go(RIGHT, self.speed)
        return "GO_UNTIL_OBSTACLE"

    def follow_wall(self):
        if current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"
        if not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT, self.speed)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT, self.speed)
            elif left > WALL_PADDING + .1:
                self.go(LEFT, self.speed)
            else:
                self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"
        else:
            return "GO_UNTIL_OBSTACLE"

    def goto_charger(self):
        self.temp_loc = self.target
        self.target = closest_in(chargers, *current_location.current_location()[0:2])
        print "Going to charger at ", self.target
        return "GO_UNTIL_OBSTACLE"

    def return_to_start(self):
        self.temp_loc = self.target
        self.target = self.initial
        print "Two minutes exceeded. Returning to start location."

    def goto_dest(self):
        self.target = self.temp_loc
        self.bat = 100
        print "Returning to path to target charged."

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        g = current_location.global_to_local(necessary_heading(x, y, *self.target))
        at = current_dists.at(g)
        (_, left) = current_dists.get()
        return at > 10 and left > 10

    def go(self, direction, speed):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = speed
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            pass
        self.pub.publish(cmd)

    def step(self):
        self.state = self.states[self.state](self) # did I stutter?
        if self.bat < 30 and self.target not in chargers:
            self.state = "GOTO_CHARGER"
        if (self.target in chargers
            and current_location.distance(*self.target) <= delta+.1):
            self.goto_dest()
        rospy.sleep(.1)

    def decr_battery(self):
        self.bat -= 1
        print "Bat:", self.bat

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: rosrun bugs bug.py X Y"
        sys.exit(1)

    (tx, ty) = map(float, sys.argv[1:3])
    print "Setting target:", (tx, ty)
    bug = Bug(tx, ty)
    init_listener()
    print "Calibrating sensors..."
    # This actually just lets the sensor readings propagate into the system
    rospy.sleep(1)
    print "Calibrated"
    (ix, iy, _) = current_location.current_location()
    bug.initial = (ix, iy)
    rospy.Timer(rospy.Duration(10), lambda _: bug.decr_battery())
    rospy.Timer(rospy.Duration(120), lambda _: bug.return_to_start())

    while current_location.distance(*bug.target) > delta:
        bug.step()
