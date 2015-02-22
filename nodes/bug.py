#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform
import pycsp.greenlets as csp

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from location import Location, necessary_heading
from dist import Dist

current_location = Location()
current_dists = Dist()

delta = .1
WALL_PADDING = .5

GO_ST = 0
GO_LF = 1
GO_RT = 2
MSG_STOP = 3


def init_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)
    rospy.Subscriber('base_scan', LaserScan, sensor_callback)

def location_callback(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)

def sensor_callback(data):
    current_dists.update(data)

@csp.process
def mover(cin):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    while True:
        c, got = csp.AltSelect(
                csp.InputGuard(cin),
                csp.TimeoutGuard(seconds=0.01))

        if c != cin:
            pass
        elif got == GO_ST:
            cmd = Twist()
            cmd.linear.x = 1
        elif got == GO_LF:
            cmd = Twist()
            cmd.angular.z = 0.25
        elif got == GO_RT:
            cmd = Twist()
            cmd.angular.z = -0.25
        elif got == MSG_STOP:
            cmd = Twist()
            pub.publish(cmd)
            return

        pub.publish(cmd)

@csp.process
def bug_algorithm(out):
    init_listener()
    while current_location.distance(tx, ty) > delta:
        face_target(out)
        hit_wall = go_until_obstacle(out)
        if hit_wall:
            follow_wall(out)
    out(MSG_STOP)

def go_until_obstacle(out):
    print "Going forward until destination or obstacle"
    while current_location.distance(tx, ty) > delta:
        (frontdist, _) = current_dists.get()
        if frontdist <= WALL_PADDING:
            return True

        if current_location.facing_point(tx, ty):
            out(GO_ST)
        elif current_location.faster_left(tx, ty):
            out(GO_LF)
        else:
            out(GO_RT)
        rospy.sleep(.01)
    return False

def face_target(out):
    print "Turning to face destination"
    while not current_location.facing_point(tx, ty):
        if current_location.faster_left(tx, ty):
            out(GO_LF)
        else:
            out(GO_RT)
        rospy.sleep(.01)

def follow_wall(out):
    print "Following wall"
    while current_dists.get()[0] <= WALL_PADDING:
        out(GO_RT)
        rospy.sleep(.01)
    while not should_leave_wall():
        (front, left) = current_dists.get()
        if front <= WALL_PADDING:
            out(GO_RT)
        elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
            out(GO_ST)
        elif left > WALL_PADDING + .1:
            out(GO_LF)
        else:
            out(GO_RT)
        rospy.sleep(.01)

def go_a_little(out):
    for i in range(0, 10):
        out(GO_ST)
        rospy.sleep(.1)

def should_leave_wall():
    (x, y, t) = current_location.current_location()
    dir_to_go = current_location.global_to_local(necessary_heading(x, y, tx, ty))
    at = current_dists.at(dir_to_go)
    print "At", dir_to_go, ":", at
    if at > 3:
        return True
    return False

# Parse arguments
algorithm = sys.argv[1]
algorithms = ["bug0", "bug1", "bug2"]
if algorithm not in algorithms:
    print "First argument should be one of ", algorithms, ". Was ", algorithm
    sys.exit(1)

if len(sys.argv) < 4:
    print "Usage: rosrun bugs bug.py ALGORITHM X Y"
    sys.exit(1)
(tx, ty) = map(float, sys.argv[2:4])
print "Setting target: (", tx, ", ", ty, ")"


C = csp.Channel()
csp.Parallel(
        bug_algorithm(C.writer()),
        mover(C.reader()),
)


print "There: distance is ", current_location.distance(tx, ty)
print "At", current_location.current_location()
