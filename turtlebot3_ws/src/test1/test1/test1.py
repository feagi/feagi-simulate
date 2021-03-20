#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def callback(dt):
    print('-------------------------------------------')
    print('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print('Range data at 345 deg: {}'.format(dt.ranges[345]))
    print('-------------------------------------------')
    thr1 = 0.8 # Laser scan range threshold
    thr2 = 0.8
    if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: # Checks if there are obstacles in front and
                                                                         # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)
        move.linear.x = 0.5 # go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)
    else:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.5 # rotate counter-clockwise
        if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
            move.linear.x = 0.5
            move.angular.z = 0.0
    self.cmd_vel_pub.publish(move) # publish the move object

qos = QoSProfile(depth=10)
move = Twist() # Creates a Twist message type object
rclpy.init(args=sys.argv)
node = rclpy.create_node('obstacle_avoidance_node') # Initializes a node
pub = node.create_publisher(Twist, "/cmd_vel", qos)  # Publisher object which will publish "Twist" type messages
# on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

self.scan_sub = move.create_subscription(
    LaserScan,
    'scan',
    self.callback,
    qos_profile=qos_profile_sensor_data)# Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rclpy.spin(node) # Loops infinitely until someone stops the program execution

