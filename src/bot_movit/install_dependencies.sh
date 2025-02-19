#!/bin/bash

echo "Updating system package lists..."
sudo apt update && sudo apt upgrade -y

echo "Installing required system dependencies..."
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool python3-rosinstall-generator

echo "Initializing rosdep if not already initialized..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

echo "Installing MoveIt2 dependencies..."
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-moveit\
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    ros-humble-rclpy \
    ros-humble-controller-manager \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-warehouse \
    ros-humble-moveit-setup-assistant \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-tf2-ros \
    ros-humble-warehouse-ros-mongo \
    ros-humble-std-msgs

echo "Installing missing dependencies using rosdep..."
cd ~/Desktop/naor/study/MoveIt2_try
rosdep install --from-paths src --ignore-src -r -y

echo "Building ROS 2 workspace..."
colcon build --symlink-install

echo "Installation complete!"
