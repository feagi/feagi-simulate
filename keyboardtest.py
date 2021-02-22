#! /usr/bin/env python

#modified the code, Keyboard_teleop.py, to adapt the exercise and test the goal.
# The original code of this modified code is at https://github.com/WPI-Humanoid-Robotics-Lab/drcsim/blob/master/drcsim_gazebo/scripts/keyboard_teleop.py
#To modify this code is to learn how to use the code and understand how to reacts to arrow keys only. 

import roslib; roslib.load_manifest('drcsim_gazebo')

from atlas_msgs.msg import WalkDemoAction, \
                           WalkDemoActionGoal, \
                           WalkDemoGoal, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorStepParams, \
                           AtlasBehaviorStandParams, \
                           AtlasBehaviorManipulateParams
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Range #for proximity ranger
import actionlib
import math
import screen #added this line
import rospy
import select
import sys
import termios
import tty
import curses #added this line to understand the arrow keys
import os #added this line for getch


class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch #This is to read the arrow keys


class AtlasTeleop(object):

    # Keyboard teleop bindings		      #This is the map of actions
    dynamic_dir = {'u': {"forward":1, "lateral":0, "turn": 1}, \
                  'i': {"forward":1, "lateral":0, "turn": 0}, \
                  'o': {"forward":1, "lateral":0, "turn":-1}, \
                  'j': {"forward":0, "lateral":1, "turn": 0}, \
                  'k': {"forward":0, "lateral":0, "turn": 0}, \
                  'l': {"forward":0, "lateral":-1, "turn": 0}, \
                  'm': {"forward":0, "lateral":0, "turn": 0.5}, \
                  ',': {"forward":-1, "lateral":0, "turn": 0}, \
                  '.': {"forward":0, "lateral":0, "turn":-0.5}}


    # BDI Controller bindings
    params = {"Forward Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Stride Length Interval":{"value":0.05, "min":0, "max":1, \
                                "type":"float"},
              "Lateral Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Step Height":{"value":0, "min":-1, "max":1, "type":"float"}, \
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Walk Sequence Length":{"value":5, "min":1, "max":sys.maxint, \
                                "type":"int"},
              "Stride Width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "In Place Turn Size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "Turn Radius":{"value":2, "min":0.01, "max":100, "type":"float"},
              "Swing Height":{"value":0.1, "min":0, "max":1, "type":"float"}}   #this is like the speed of movement. 


    def init(self):
        self.static_step_count = 0
        #scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

#    def scan_callback(self, msg):
#	range_ahead = msg.ranges[len(msg.ranges)/2]
#	print "range ahead: %0.1f" % range_ahead


        # Saves terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Creates the SimpleActionClient, passing the type of the action
        # () to the constructor.
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', \
          WalkDemoAction)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, queue_size=1)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
          String, None, False, True, queue_size=1)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("Waiting for atlas/bdi_control")
        self.client.wait_for_server()

    def fini(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

#This function is to "print" the action by moving until the process complete
#then it will pause and go back to idle.
    def run(self):
        try:
            self.init()
            self.print_usage()
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

#This function doesn't do anything
    def beep(self):
        curses.flash()

#This is for output in terminal when you run this code
    def print_usage(self):
        msg = """
        Keyboardtest.py (Modified) for AtlasSimInterface 1.1.0
        --------------------------------------------------
        Dynamic linear movement:

                       arrow up
           arrow left-         -arrow right
                       arow down



        """
        self.loginfo(msg)

    # Publishes commands to reset robot to a standing position
    def reset_to_standing(self):
        self.mode.publish("harnessed")
        self.control_mode.publish("Freeze")
        self.control_mode.publish("StandPrep")
        rospy.sleep(2.0)
        self.mode.publish("nominal")
        rospy.sleep(0.3)
        self.control_mode.publish("Stand") #This will display output on gazebo terminal.

    # Builds a trajectory of step commands.
    # Param forward: 1 forward, -1 backward or 0 if no forward component
    # Param lateral: 1 left, -1 right, 0 if no lateral component
    # Param turn: 1 Counter clockwise turn, -1 clockwise turn
    def dynamic_twist(self, forward, lateral, turn):
        self.is_static = False
        steps = self.build_steps(forward, lateral, turn)

        # 0 for full BDI control, 255 for PID control
        k_effort = [0]*28

        for step in steps:
            self.debuginfo("foot: " + str(step.foot_index) + \
              " [" + str(step.pose.position.x) + \
              ", " + str(step.pose.position.y) + "]")

        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps, \
          AtlasBehaviorStepParams(), AtlasBehaviorStandParams(), \
          AtlasBehaviorManipulateParams(),  k_effort )

        self.client.send_goal(walk_goal)


        # should make a callback that subscribes to the actionlib results topic
        # rather than a blocking wait here, so user can dispatch new goals
        # while last goal is being executed.
        # self.client.wait_for_result(\
        #   rospy.Duration(self.params["Stride Duration"]["value"] * \
        #                  len(steps) + 5))

    def static_twist(self, forward, lateral, turn):
        self.is_static = True
        steps = self.build_steps(forward, lateral, turn)

        if forward != 0:
          idx = self.static_step_count % 2
        else:
          idx = lateral % 2

        # step needs index to be 1
        steps[idx].step_index = 1
        steps[idx].swing_height = 0.05

        # 0 for full BDI control, 255 for PID control
        k_effort = [0]*28

        stand_goal = WalkDemoGoal(Header(), WalkDemoGoal.STEP, None, \
              AtlasBehaviorStepParams(steps[idx], False), AtlasBehaviorStandParams(), \
              AtlasBehaviorManipulateParams(),  k_effort )

        self.client.send_goal(stand_goal)
        # self.client.wait_for_result()

        self.static_step_count = self.static_step_count + 1
        rospy.sleep(0.3)
        # for step in steps:
        #     step.step_index = 1
        #     self.debuginfo("step: " + str(step))
        #     walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.STEP, None, \
        #       AtlasBehaviorStepParams(step, False), AtlasBehaviorStandParams(), \
        #       AtlasBehaviorManipulateParams(),  k_effort )
        #
        #
        #     self.client.send_goal(walk_goal)
        #     result_status = self.client.wait_for_result(rospy.Duration(5))
        #     if result_status != 0:
        #         result = self.client.get_result()
        #         rospy.sleep(4)
        #         if result.success == False:
        #             self.loginfo("Static walk failed: \n" + "Goal: \n " + str(walk_goal) + "\nResult: " + str(result))
        #             break
        #     #if self.client.get_result() != SUCCEEDED:
        #     #    self.loginfo("Static walk trajectory timed out, cancelling")
        #     #    break
	#This is for robot to walking properly
    def build_steps(self, forward, lateral, turn):
        L = self.params["Forward Stride Length"]["value"]
        L_lat = self.params["Lateral Stride Length"]["value"]
        R = self.params["Turn Radius"]["value"]
        W = self.params["Stride Width"]["value"]
        X = 0
        Y = 0
        theta = 0
        dTheta = 0

        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + \
            self.params["Stride Width"]["value"]/2)))
        else:
            dTheta = turn * self.params["In Place Turn Size"]["value"]
        steps = []


        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + \
            self.params["Stride Width"]["value"]/2)))
        else:
            dTheta = turn * self.params["In Place Turn Size"]["value"]
        steps = []

        # This home step doesn't currently do anything, but it's a
        # response to bdi not visiting the first step in a trajectory
        # home_step = AtlasBehaviorStepData()

        # If moving right, first dummy step is on the left
        # home_step.foot_index = 1*(lateral < 0)
        # home_step.pose.position.y = 0.1
        # steps.append(home_step)
        prevX = 0
        prevY = 0

        # Builds the sequence of steps needed
        for i in range(self.params["Walk Sequence Length"]["value"]):
            # is_right_foot = 1, when stepping with right
            is_even = i%2
            is_odd = 1 - is_even
            is_right_foot = is_even
            is_left_foot = is_odd

            # left = 1, right = -1
            foot = 1 - 2 * is_right_foot

            if self.is_static:
                theta = (turn != 0) * dTheta
                if turn == 0:
                    X = (forward != 0) * (forward * L)
                    Y = (lateral != 0) * (is_odd * lateral * L_lat) + \
                        foot * W / 2
                elif forward != 0:
                    # Radius from point to foot (if turning)
                    R_foot = R + foot * W/2

                    # turn > 0 for CCW, turn < 0 for CW
                    X = forward * turn * R_foot * math.sin(theta)
                    Y = forward * turn * (R - R_foot*math.cos(theta))

                    self.debuginfo("R: " + str(R) + " R_foot:" + \
                    str(R_foot) + " theta: " + str(theta) +  \
                    " math.sin(theta): " + str(math.sin(theta)) + \
                    " math.cos(theta) + " + str(math.cos(theta)))
                elif turn != 0:
                    X = turn * W/2 * math.sin(theta)
                    Y = turn * W/2 * math.cos(theta)
            else:
                theta += (turn != 0) * dTheta
                if turn == 0:
                    X = (forward != 0) * (X + forward * L)
                    Y = (lateral != 0) * (Y + is_odd * lateral * L_lat) + \
                        foot * W / 2
                elif forward != 0:
                    # Radius from point to foot (if turning)
                    R_foot = R + foot * W/2

                    # turn > 0 for CCW, turn < 0 for CW
                    X = forward * turn * R_foot * math.sin(theta)
                    Y = forward * turn * (R - R_foot*math.cos(theta))

                    self.debuginfo("R: " + str(R) + " R_foot:" + \
                    str(R_foot) + " theta: " + str(theta) +  \
                    " math.sin(theta): " + str(math.sin(theta)) + \
                    " math.cos(theta) + " + str(math.cos(theta)))
                elif turn != 0:
                    X = turn * W/2 * math.sin(theta)
                    Y = turn * W/2 * math.cos(theta)


            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()

            # One step already exists, so add one to index
            step.step_index = i

            # Alternate between feet, start with left
            step.foot_index = is_right_foot

            #If moving laterally to the left, start with the right foot
            if (lateral > 0):
                step.foot_index = is_left_foot

            step.duration = self.params["Stride Duration"]["value"]

            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = self.params["Step Height"]["value"]

            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]

            step.swing_height = self.params["Swing Height"]["value"]
            steps.append(step)

        # Add final step to bring feet together
        is_right_foot = 1 - steps[-1].foot_index
        is_even = is_right_foot
        # foot = 1 for left, foot = -1 for right
        foot = 1 - 2 * is_right_foot

        if turn == 0:
            Y = Y + foot * W
        elif forward != 0:
            self.debuginfo("R: " + str(R) + " R_foot:" + \
            str(R_foot) + " theta: " + str(theta) +  \
           " math.sin(theta): " + str(math.sin(theta)) + \
           " math.cos(theta) + " + str(math.cos(theta)))

            # R_foot is radius to foot
            R_foot = R + foot * W/2
            #turn > 0 for counter clockwise
            X = forward * turn * R_foot * math.sin(theta)
            Y = forward * turn * (R - R_foot*math.cos(theta))
        else:
            X = turn * W/2 * math.sin(theta)
            Y = turn * W/2 * math.cos(theta)

        Q = quaternion_from_euler(0, 0, theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = is_right_foot
        step.duration = self.params["Stride Duration"]["value"]
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = self.params["Step Height"]["value"]
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["Swing Height"]["value"]

        steps.append(step)

        return steps 


    def datasensor():
        pub = rospy.Publisher('range', Range,queue_size=10)
        #rospy.init_node('walking_client')
        ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
        min_range = 2.0
        max_range = 2.0
        while not rospy.is_shutdown():
            for rg in ranges:
                r = Range()
                r.header.stamp = rospy.Time.now()
                r.header.frame_id = "/head"
                r.radiation_type = 0
                r.field_of_view = 0.1
                r.min_range = min_range
                r.max_range = max_range
                r.range = rg
                pub.publish(r)
                rospy.sleep(1.0) ##to see if it being inside the astlas class works


   # Puts teleop into edit param mode
    def edit_params(self):
        # Reset terminal to normal settings so you can see input
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("")

        # Find the longest param, and use that to justify strings
        maxLength = -1
        for key in self.params.keys():
            if len(key) > maxLength:
                maxLength = len(key)
        for i in range(len(self.params)):
            param_name = self.params.keys()[i]
            print(str(i) + " : " + param_name.ljust(maxLength + 1) + \
              str(self.params[param_name]["value"]))
        print("X : Exit")
        hasNumber = False
        selection = -1

        # Get the input, and check if it's valid
        while selection < 0 or selection >= len(self.params):
            var = raw_input("Enter number of param you want to change: ")

            if var == 'x' or var == 'X':
                self.print_usage()
                return
            try:
                selection = int(var)
            except ValueError:
                selection = -1

        param = self.params.keys()[selection]
        value = 0
        valid = False

        # Keep asking for input while the value is not valid, or it is outside
        # of the acceptable range
        while not valid:
            var = raw_input("New value for " + param + " [min: " +
            str(self.params[param]["min"]) + ", max: " +
            str(self.params[param]["max"]) + ", type: " +
            str(self.params[param]["type"]) + "]? ")
            try:
                if (self.params[param]["type"] is "float"):
                    value = float(var)
                elif (self.params[param]["type"] is "int"):
                    value = int(var)
                valid = (value >= self.params[param]["min"] and \
                         value <= self.params[param]["max"])
            except ValueError:
                valid = False

        self.params[param]["value"] = value
        self.edit_params()

    # Used to print items to screen, while terminal is in funky mode
    def loginfo(self, str):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        #rospy.loginfo(str)
        print(str)
        tty.setraw(sys.stdin.fileno())

    # Used to print debug items to screen while terminal is funky
    def debuginfo(self, str):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.logdebug(str)
        tty.setraw(sys.stdin.fileno())

    # For everything that can't be a binding, use if/elif instead
    def process_key(self, ch):
        if self.dynamic_dir.has_key(ch):
            dir = self.dynamic_dir[ch]
            self.dynamic_twist(dir["forward"], dir["lateral"], dir["turn"])
        elif ch == 'e' or ch == 'E':
            self.edit_params()
        elif ch == 'r' or ch == 'R':
            self.reset_to_standing()
        elif ch == 'h' or ch == 'H':
            self.print_usage()
        elif ch == 'q' or ch == 'Q' or ord(ch) == 3:
            self.loginfo("Quitting")
            rospy.signal_shutdown("Shutdown")
        elif ch == '=' or ch == '+':
            self.params["Forward Stride Length"]["value"] += \
                self.params["Stride Length Interval"]["value"]
            self.loginfo("Forward Stride Length: " + \
                         str(self.params["Forward Stride Length"]["value"]))
        elif ch == '-' or ch == "_":
            self.params["Forward Stride Length"]["value"] -= \
                self.params["Stride Length Interval"]["value"]
            self.loginfo("Forward Stride Length: " + \
                         str(self.params["Forward Stride Length"]["value"]))
        try:
            if (int(ch) >= self.params["Walk Sequence Length"]["min"] and \
                int(ch) <= self.params["Walk Sequence Length"]["max"]):
                self.params["Walk Sequence Length"]["value"] = int(ch)
                self.loginfo("Walk Sequence Length: " + \
                  str(self.params["Walk Sequence Length"]["value"]))
        except ValueError:
            pass

    # Get input from the terminal
    def get_key(self):
        #tty.setraw(sys.stdin.fileno())
        #select.select([sys.stdin], [], [], 0)
	print ("Press R to reset the body. Please press any arrow to move the robot")
	key = 'z' #just to initalize the variable.
	#print key #This is for debug use. You may delete this line. 
	inkey = _Getch() #to read the arrow keys
	k=inkey()
	if k=='\x1b[A': #step forward using Getch() and Curse
		key = 'i' #This will go back to 48 lines in this code
	elif k=='\x1b[B': #step backward
		key = ','
	elif k =='\x1b[C':
		key = 'm' #turn left
	elif k=='\x1b[D':
		key = 'a' #turn right
	elif k=='r':
		self.reset_to_standing()
	elif k=='q':
            self.loginfo("Quitting")
            rospy.signal_shutdown("Shutdown")
        return key #It sends the value to 107 lines

#class lazer:

#	def __init__(selff):
#		selff.scan_sub = rospy.Subscriber('scan', LaserScan, selff.callback)
#		

#	def callback(selff,msg):
		#laser_data
#		msg = selff.LaserScan()
#		print msg
#Comment this out to not use laserscan


    def callback(data):
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

#def listener():
    #rospy.init_node('listener', anonymous=True)
    #rospy.init_node('walking_client') #Adding this to see if it works
    #rospy.Subscriber("ultrasound", Range, callback)
    #rospy.spin()


#def walking_client():
#    pub = rospy.Publisher('range', Range,queue_size=10)
#    #rospy.init_node('walking_client')
#    ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
#    min_range = 2.0
#    max_range = 2.0
#    while not rospy.is_shutdown():
#        for rg in ranges:
#            r = Range()
#            r.header.stamp = rospy.Time.now()
#            r.header.frame_id = "/head"
#            r.radiation_type = 0
#            r.field_of_view = 0.1
#            r.min_range = min_range
#            r.max_range = max_range
#            r.range = rg
#            pub.publish(r)
#            rospy.sleep(1.0)
#if __name__ == '__main__':
#    try:
#        walking_client()
 #   except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    rospy.init_node('walking_client')
    #rospy.Subscriber("ultrasound", Range, callback)
    #rosp	y.init_node('range_ahead') #comment this out to keep init_node one
    #scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
    #test = lazer()
    #test.callback(,msg)
    teleop = AtlasTeleop()
    #try:
    #    walking_client()
    #except rospy.ROSInterruptException: pass
    #test.callback(eas)
   # print "egg"
    #listener() #adding this to display the number
    teleop.run()
   # test = lazer()
#    print "apple"
 #   print test.callback(teleop,msg)


