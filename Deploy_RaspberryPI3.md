#To download ROS2 on 20.04 LTS Focal Ubuntu arm64 in the Raspberry pi model b+:
1. dpkg --print-architecture
* Verify the match package from the [link](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211)
2. Select your right package for your computer. It will be in your download folder by default. 
3. run it in your terminal following those list:
-sudo apt update 
-sudo apt install curl gnupg2 lsb-release
-curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
-sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
-mkdir -p ~/ros2_foxy
-cd ~/ros2_foxy

4.Depends on where you put the package. Assume you still have it in Download directory:
-tar xf ~/Downloads/ros2-foxy-20201211-linux-focal-arm64.tar.bz2 
>if it's in your ros2_foxy folder, run this:
	-tar xf ros2-foxy-20201211-linux-focal-arm64.tar.bz2

5. Once it completed:
-sudo apt update
-sudo apt install -y python3-rosdep
-sudo rosdep init
-rosdep update
-rosdep install --from-paths ~/ros2_foxy/ros2-linux/share --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

6. Add Python 3 libaries:
-sudo apt install -y libpython3-dev python3-pip
-pip3 install -U argcomplete

7. Lastly: To test some demos:
-. ~/ros2_foxy/ros2-linux/setup.bash
-ros2 run demo_nodes_cpp talker
-ros2 run demo_nodes_py listener 
>(in new terminal after you source again)


#Download Feagi
1. git clone git@github.com:feagi/feagi-core.git
2. cd feagi-core/
3. pip3 install virtualenv
4. virtualenv -p /usr/bin/python3 environName
5. source ./environName/bin/activate
>You need to re-open terminal or reload your terminal after #4.
6. pip3 install -r requirements.txt
7. sudo mkdir /mnt/ramdisk
8. sudo mount -t tmpfs -o rw,size=1000M tmpfs /mnt/ramdisk
9. python3 ./src/cython_libs/cython_setup.py build_ext --inplace
10. sudo wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
11. echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
12. sudo apt-get update
13. sudo apt-get install -y mongodb-org
14. sudo apt-get install libatlas-base-dev
15. cd src/
16. python3 main.py
