# Requirement:
1. Python 3 
2. 32 GB SD
3. Raspberry PI 3+ (Older may run but heavily lagged)
4. Since Python 3 is required, Noetic (ROS1) or Foxy (ROS2 and preferably) 


# Recommendation:
1. Make a room for 1 GB in disksize in Deploy.md instruction.
2. HIGHLY recommend to use Foxy since FEAGI relies on accurate data in real time heavily.
3. HIGHLY recommend to use SSH.
4. HIGHLY recommend to not use GUI since you just need data and codes. Using GUI will result the extra slowness and lags.

# To download ROS2 on 20.04 LTS Focal Ubuntu arm64 in the Raspberry pi 3 model b+:
1. dpkg --print-architecture
* Verify the match package from the [link](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211)

2. Follow the [instruction](https://docs.ros.org/en/foxy/Installation/Linux-Install-Binary.html)

3. Once you verify you were able to run talker and listener on Raspberry PI, you are good to go next step.

# Download Feagi
The instruction is followed by [here](https://github.com/feagi/feagi-core/blob/develop/DEPLOY.md)


