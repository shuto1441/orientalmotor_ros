# common_ros_packages

## Requirement
* Ubuntu 18.04
* ROS melodic


## package list:

- The ros linux wrapper for oriental motor
- The ros linux wrapper for dmk high speed camera

## installation:
```bash
mkdir ~/catkin_ws -p
cd ~/catkin_ws
wstool init src
wstool set -u -t src https://github.com/ut-hnl-lab/common_ros_packages.git
wstool merge -t src src/common_ros_packages/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

### additional installation for dmk high speed camera

Please follow [building-tiscamera](https://github.com/ut-hnl-lab/common_ros_packages/tree/main/dmk_high_speed_cam_ros#building-tiscamera).

## usage

Please check README.md in each ros package.
