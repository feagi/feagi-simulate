#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def callback(dt):
    print '-------------------------------------------'
    print 'Range data at 0 deg:   {}'.format(dt.ranges[0])
    print 'Range data at 15 deg:  {}'.format(dt.ranges[15])
    print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    print '-------------------------------------------'


rospy.init_node('obstacle_avoidance_node') # Initializes a node
sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution
