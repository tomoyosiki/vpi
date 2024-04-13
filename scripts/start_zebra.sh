#!/bin/bash

# Check for the command-line argument
if [ "$1" == "--first-start" ]; then
    echo "Configuring network settings..."
    sudo ip addr flush dev eth0
    sudo ip addr add 192.168.2.100/24 dev eth0
    sudo ip link set dev eth0 down
    sudo ip link set dev eth0 up
fi

# Source the ROS environment
source /opt/ros/melodic/setup.bash  # Change 'noetic2' to your ROS version

# Assuming you have different ROS workspaces or packages in separate folders
# Replace '/path/to/folder' with the actual folder paths where your launch files are located

# Launch the first node
cd /home/zebra/ros_ws
source devel/setup.bash  # Source the setup script for this specific ROS workspace if necessary
if [ "$1" == "--first-start" ]; then
    rosrun hunter_bringup bringup_can2usb.bash &
fi
roslaunch hunter_bringup hunter_robot_base.launch &

# Launch the second node
cd /mnt/ros_ws
source devel/setup.bash  # Source the setup script for this specific ROS workspace if necessary
roslaunch velodyne_pointcloud VLP16_points.launch &
roslaunch ndt_localizer ndt_localizer.launch &

# Repeat for other nodes, adjusting paths and package names as necessary

# Wait for all background jobs to finish
wait
