# RealSense ROS 2 Integration

The following instructions are for recording IMU data from a RealSense D435i camera using ROS 2 on an Ubuntu 22.04 machine.

Skip to step 3 if you already have the dependencies installed or are using the [devcontainer](../.devcontainer/README.md).

Prerequisites:
- Ubuntu 22.04
- ROS 2 Humble (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Install ros-humble-rosbag2-storage-mcap (https://github.com/ros2/rosbag2/tree/rolling/rosbag2_storage_mcap). `sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap`
- RealSense D435i

1. Install RealSense SDK: Install the Intel RealSense SDK, which provides the necessary drivers and tools to interface with the camera. You can find installation instructions on the Intel RealSense GitHub page (https://github.com/IntelRealSense/librealsense). Specifically, follow the instructions for Ubuntu 22.04: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md which are also included below.

```bash
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils
    sudo apt-get install librealsense2-dev
    sudo apt-get install librealsense2-dbg
```

2. Install RealSense ROS 2 Wrapper: This allows you to use the camera with ROS 2. You can install it from source or binaries. Here are the basic steps to install from source included below: (https://github.com/IntelRealSense/realsense-ros)

```bash
   # Source your ROS2 setup (if not already in your .bashrc to be sourced automatically)
   source /opt/ros/humble/setup.bash

   # Add ROS2 repository
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt install curl # if you haven't already installed curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

   # Install ROS2 RealSense packages
   sudo apt install ros-humble-realsense2-*

   # Create a workspace or alternatively use your existing workspace
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   # Clone the RealSense ROS 2 wrapper
   git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

   cd ~/ros2_ws
   sudo apt-get install python3-rosdep -y
   sudo rosdep init
   rosdep update
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

   # Build the workspace
   colcon build

   # Source the workspace
   source install/local_setup.bash
```

3. Launch the RealSense Node: Use the provided launch files to start the RealSense node, which will publish the camera data, including IMU data, to ROS 2 topics.

```bash
   # Ensure the workspace is built
   cd ~/ros2_ws && colcon build

   # Source your workspace
   source ~/ros2_ws/install/local_setup.bash

   # Launch the RealSense node
   ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=2
```

4. Check Available Topics: Use the following command to list all available topics and verify that the IMU data topic is being published:

```bash
   ros2 topic list
```

5. Echo IMU Data: Use the ros2 topic echo command to view the IMU data being published.

```bash
   ros2 topic echo /camera/camera/imu
```

6. Record IMU Data: Use the ros2 bag record command to record the IMU data to an MCAP file.

```bash
   # Record IMU data to an mcap file (ie. for 3000 seconds)
   ros2 bag record /camera/camera/imu -s mcap -d 3000
```
