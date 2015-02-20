#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from location import Location

current_location = Location()

delta = 1.0

def init_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)

def location_callback(data):
    p = data.pose.pose.position
    q = data.pose.pose.orientation
    t = transformations.euler_from_quaternion(q)
    current_location.update_location(p.x, p.y, q)
    #print data.pose.pose.orientation


algorithm = sys.argv[1]
algorithms = ["bug0", "bug1", "bug2"]
if algorithm not in algorithms:
    print "First argument should be one of ", algorithms, ". Was ", algorithm
    sys.exit(1)

(tx, ty) = map(float, sys.argv[2:4])
print "Setting target: (", tx, ", ", ty, ")"

init_listener()

while current_location.distance(tx, ty) > delta:
    print "Not there yet (", current_location.distance(tx, ty), ")"
    rospy.sleep(1)
print "There: distance is ", current_location.distance(tx, ty)
print "At", current_location.current_location()
