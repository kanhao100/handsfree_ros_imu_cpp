# handsfree_ros_imu_cpp

#### Introduction
This is the C++ version of [hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu), a ROS driver package for the IMU sensor of Handsfree, which broadcasts the data sent by the IMU sensor through serial port via ROS topic. Due to the extremely low efficiency of the ROS driver package provided by the manufacturer [hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu), it has been optimized and modified to the C++ version, greatly reducing unnecessary CPU usage.

#### Requirement
```
sudo apt-get update
sudo apt-get install ros-$(echo $ROS_DISTRO)-ros-base
sudo apt-get install ros-$(echo $ROS_DISTRO)-roscpp
sudo apt-get install ros-$(echo $ROS_DISTRO)-rospy
sudo apt-get install ros-$(echo $ROS_DISTRO)-std-msgs
sudo apt-get install ros-$(echo $ROS_DISTRO)-serial
```

#### Installation

1. Create a catkin workspace:
```
cd ~/
mkdir handsfree_ros_workspace
cd handsfree_ros_workspace
mkdir src
cd src
git clone 
cd ..
```
2. Build and configure environment variables:
```
catkin_make
source ./devel/setup.sh
```

#### Usage

1. Run the driver only:
```
roslaunch handsfree_ros_imu_cpp handsfree_imu.launch imu_type:=b6
```
or
```
rosrun handsfree_ros_imu_cpp hfi_b6
```

2. Run RVIZ visualization only:
```
roslaunch handsfree_ros_imu_cpp view_rviz.launch imu_type:=b6
```

3. Run both driver and RVIZ visualization:
```
roslaunch handsfree_ros_imu_cpp rviz_and_imu.launch imu_type:=b6
```

#### TODO
- Add support for A9 sensors, but I don't have these sensors. Pull Request is welcomed!
