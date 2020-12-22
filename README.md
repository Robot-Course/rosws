# Robot Course - ROS Module

Source code of ROS Module for Robot Course, based on ROS 2.

## Install

1. `git clone --recurse-submodules https://github.com/Robot-Course/rosws.git`
2. `cd rosws`
3. `colcon build`
4. `source install/local_setup.bash`
5. `echo 'source install/local_setup.bash' >> ~/.bashrc`

## Run

`ros2 launch robot robot.launch.py lidar:=true` to start lidar only.

or

`ros2 launch robot robot.launch.py imu:=true motor:=true` to start imu and motor together.

etc.