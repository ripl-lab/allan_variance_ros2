#! /bin/bash

# Convert a ROS1 bag to a ROS2 bag using the MCAP format

# Usage: ./convert_ros1_bag_to_ros2_mcap.sh <ros1_bag_file> <ros2_mcap_filename> <mcap_config_yaml>

# Example: ./convert_ros1_bag_to_ros2_mcap.sh my_ros1_bag.bag my_ros2_mcap.mcap ../config/ros2_mcap_config.yaml

# This script will convert the ROS1 bag file to a ROS2 bag file using the MCAP format
# The ROS2 bag file will be named my_ros2_mcap.mcap located in the output directory

# Check if the ROS1 bag file is provided
if [ -z "$1" ]; then
    echo "Error: ROS1 bag file not provided"
    echo "Usage: ./convert_ros1_bag_to_ros2_mcap.sh <ros1_bag_file> <ros2_mcap_file> <mcap_config_yaml>"
    exit 1
fi

# Check if the ROS2 mcap file is provided
if [ -z "$2" ]; then
    echo "Error: ROS2 mcap file not provided"
    echo "Usage: ./convert_ros1_bag_to_ros2_mcap.sh <ros1_bag_file> <ros2_mcap_file> <mcap_config_yaml>"
    exit 1
fi

# Check if the mcap config yaml file is provided
if [ -z "$3" ]; then
    echo "Error: mcap config yaml file not provided"
    echo "Usage: ./convert_ros1_bag_to_ros2_mcap.sh <ros1_bag_file> <ros2_mcap_file> <mcap_config_yaml>"
    exit 1
fi

# Check if the ROS1 bag file exists
if [ ! -f "$1" ]; then
    echo "Error: ROS1 bag file not found: $1"
    exit 1
fi

temp_db3_bag_dir=temp_ros2_bags

ros1_bag_file=$1
ros2_mcap_file=$2
mcap_config_yaml=$3

# strip the .mcap extension from the ros2_mcap_file
ros2_mcap_file_dir=$(dirname $ros2_mcap_file)/$(basename $ros2_mcap_file .mcap)

rosbags-convert --src $ros1_bag_file --dst $temp_db3_bag_dir

# Make a temporary copy of the mcap config yaml file and change the uri to the ros2_mcap_file
mcap_config_yaml_temp=$temp_db3_bag_dir/mcap_config.yaml
cp $mcap_config_yaml $mcap_config_yaml_temp
sed -i "s|uri: .*|uri: $ros2_mcap_file_dir|" $mcap_config_yaml_temp

ros2 bag convert -i $temp_db3_bag_dir -o $mcap_config_yaml_temp

# Remove the temporary directory
rm -rf $temp_db3_bag_dir
