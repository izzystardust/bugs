#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations
import pycsp.greenlets as csp

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from location import Location

current_location = Location()

delta = 1.0

GO_ST = 0
GO_RT = 1
GO_LF = 2
MSG_STOP = 3


def init_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)

def location_callback(data):
    p = data.pose.pose.position
    q = data.pose.pose.orientation
    t = transformations.euler_from_quaternion(q)
    current_location.update_location(p.x, p.y, q)
    #print data.pose.pose.orientation

@csp.process
def mover(cin):
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('buggy')
    cmd = Twist()
    while True:
        c, got = csp.AltSelect(
                csp.InputGuard(cin),
                csp.TimeoutGuard(seconds=0.01)
                )
        if c != cin:
            pass
        elif got == GO_ST:
            cmd = Twist()
            cmd.linear.x = 1
        elif got == GO_LF:
            cmd = Twist()
            cmd.angular.z = 0.5
        elif got == GO_RT:
            cmd = Twist()
            cmd.angular.z = -0.5
        elif got == MSG_STOP:
            cmd = Twist()
            pub.publish(cmd)
            return

        pub.publish(cmd)

@csp.process
def bug_algorithm(out):
    init_listener()
    while current_location.distance(tx, ty) > delta:
        out(GO_ST)
        rospy.sleep(.01)

algorithm = sys.argv[1]
algorithms = ["bug0", "bug1", "bug2"]
if algorithm not in algorithms:
    print "First argument should be one of ", algorithms, ". Was ", algorithm
    sys.exit(1)

(tx, ty) = map(float, sys.argv[2:4])
print "Setting target: (", tx, ", ", ty, ")"

C = csp.Channel()
csp.Parallel(
        bug_algorithm(C.writer()),
        mover(C.reader()),
)


print "Not there yet (", current_location.distance(tx, ty), ")"
print "There: distance is ", current_location.distance(tx, ty)
print "At", current_location.current_location()
