# handsfree_ros_imu_cpp

#### Introduction
This is the C++ version of [hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu), a ROS driver package for the IMU sensor of Handsfree, which broadcasts the data sent by the IMU sensor through serial port via ROS topic. Due to the extremely low efficiency of the ROS driver package provided by the manufacturer [hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu), it has been optimized and modified to the C++ version, greatly reducing unnecessary CPU usage.

[English Version](https://gitee.com/kanhao100/handsfree_ros_imu_cpp/blob/master/README.en.md)

#### 介绍
这是[hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu)的C++版本，是handsfree的IMU传感器ROS驱动包，负责将IMU传感器通过串口发送的数据通过ROS Topic广播。由于厂家提供的ROS驱动包[hansfree_ros_imu](https://gitee.com/HANDS-FREE/handsfree_ros_imu)的运行效率极为低下，因此对其进行了优化并修改为了C++版本，极大幅度地降低了不必要的CPU占用率。

#### 依赖安装
```
sudo apt-get update
sudo apt-get install ros-$(echo $ROS_DISTRO)-ros-base
sudo apt-get install ros-$(echo $ROS_DISTRO)-roscpp
sudo apt-get install ros-$(echo $ROS_DISTRO)-rospy
sudo apt-get install ros-$(echo $ROS_DISTRO)-std-msgs
sudo apt-get install ros-$(echo $ROS_DISTRO)-serial
```

#### 安装教程

1.  创建catkin工作空间
```
cd ~/
mkdir handsfree_ros_workspace
cd handsfree_ros_workspace
mkdir src
cd src
git clone https://gitee.com/kanhao100/handsfree_ros_imu_cpp.git
cd ..
```
2.  编译和配置环境变量
```
catkin_make
source ./devel/setup.sh
```

#### 使用说明

1.  只运行单节点
```
roslaunch handsfree_ros_imu_cpp handsfree_imu.launch imu_type:=b6
```
或者
```
rosrun handsfree_ros_imu_cpp hfi_b6
```

2.  只运行RVIZ可视化
```
roslaunch handsfree_ros_imu_cpp view_rviz.launch imu_type:=b6
```

3.  同时运行驱动和RVIZ可视化
```
roslaunch handsfree_ros_imu_cpp rviz_and_imu.launch imu_type:=b6
```

#### TODO
- 添加A9传感器的支持，但我手上并没有相关传感器，欢迎Pull Request！





