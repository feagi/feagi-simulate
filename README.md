After you install codes from turtlebot3, be sure to make a backup each of line:
/opt/ros/foxy/lib/turtlebot3_example
/opt/ros/foxy/lib/python3.8/site-packages/turtlebot3_example/turtlebot3_obstacle_detection
/opt/ros/foxy/lib/python3.8/site-packages/turtlebot3_teleop
/opt/ros/foxy/lib/turtlebot3_teleop

You will have some mixed codes in those folders.

once you download folders from github,

be sure to run this line;
sudo cp -R ~/feagi-simulate/turtlebot3_teleop /opt/ros/foxy/lib/python3.8/site-packages/turtlebot3_teleop #this includes turtlebot3_obstacle_detection.py and turtlebot3_teleop.py
sudo cp -R ~/feagi-simulate/turtblebot3_example_launch /opt/ros/foxy/lib/turtlebot3_teleop #this is the launch where it included the turtlebot3_obstacle_detection



Once you complete it,
open a new terminal
source your ros
run gazebo

open the second terminal
source your ros
run this line;
ros2 run turtlebot3_teleop teleop_keyboard

The goal is to control the robot and move the robot until it approach to any object, it will halts and print a message, "Obstacles are detected nearby. Robot stopped."

The flowchart of this should display all goal of this sample code.

![image](https://user-images.githubusercontent.com/65916520/111640188-91d49c80-87c1-11eb-8a25-d9cf4381b1c7.png)


The lidar reading coming from turtlebot3_obstacle_detection and the keyboard control is from turtlebot3_teleop
