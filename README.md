# Allan Variance ROS2

- [Prerequisites](#prerequisites)
- [Record IMU data](#record-imu-data)
- [Reorganize ROS messages by timestamp](#reorganize-ros-messages-by-timestamp)
- [Run the Allan Variance computation tool](#run-the-allan-variance-computation-tool)
- [Visualize the Allan Deviation and Generate Kalibr Config](#visualize-the-allan-deviation-and-generate-kalibr-config)
- [Convert ROS1 bags to ROS2 mcaps](#convert-ros1-bag-to-ros2-mcap)
- [How to record data from the RealSense D435i camera](#how-to-record-data-from-the-realsense-d435i-camera)
- [Example mcap](#example-mcap)
- [Kalibr](#kalibr)
- [Allan Variance ROS Evaluation](#allan-variance-ros-evaluation)
- [IMU Noise Simulator](#imu-noise-simulator)
- [Author](#author)

## Prerequisites

If you do not have Ubuntu 22.04 or just don't want to install the dependencies locally, you can use the devcontainer to easily build and run the tool from any machine. See the [devcontainer README](.devcontainer/README.md) for more information.

Otherwise, you can install the following dependencies:

- Ubuntu 22.04
- ROS 2 Humble (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Install ros-humble-rosbag2-storage-mcap (https://github.com/ros2/rosbag2/tree/rolling/rosbag2_storage_mcap). `sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap`
- MCAP CLI (https://mcap.dev/guides/cli)

If not using the devcontainer and you want to record data from the RealSense D435i camera, see the [RealSense ROS2 Integration](docs/realsense_ros2.md) documentation to install the RealSense ROS2 wrapper and RealSense SDK.

## ROS2 package which loads a mcap file of IMU data and computes Allan Variance parameters
The purpose of this tool is to read a long sequence of IMU data and compute the Angle Random Walk (ARW), Bias Instability and Gyro Random Walk for the gyroscope as well as Velocity Random Walk (VRW), Bias Instability and Accel Random Walk for the accelerometer.

This tool is a ROS2 compatible iteration on the original [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros).

The mcap is expected to have the imu data in the message type [sensor_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html).

While there are many open source tools which do the same thing, this package has the following features:

- Fully ROS compatable. Simply record an `mcap` file and provide it as input. No conversion required.
- Written in Python making use of the `mcap` library means the `mcap` file is processed at maximum speed. No need to play back the mcap file.
- Designed for [Kalibr](https://github.com/ethz-asl/kalibr). Will produce an `imu.yaml` file.

This tool is designed for Ubuntu 22.04. Attempting to use on another distro or version may require some code changes. See the [devcontainer README](.devcontainer/README.md) for more information.

## How to use

### Record IMU data

Place your IMU on some damped surface and record your IMU data to an mcap file. You must record **at least** 3 hours of data. The longer the sequence, the more accurate the results.

To record an `mcap` file from the RealSense D435i camera, see the [RealSense ROS2 Integration](docs/realsense_ros2.md) documentation for more information to do so from within this repository.

### Reorganize ROS messages by timestamp (Recommended)

To make the Allan Variance computation tool run faster, you can reorganize the ROS messages by timestamp. This is done by running the `cookmcap.py` script.

  ``python3 src/cookmcap.py --input mcaps/original_mcap.mcap --output mcaps/cooked_mcap.mcap``

### Run the Allan Variance computation tool

This will compute the Allan Deviation for the IMU and generate a CSV.

  ``python3 src/allan_variance_computer.py --input_mcap mcaps/cooked_mcap/cooked_mcap.mcap --output_dir allan_variance --imu_config_yaml config/realsense_d435i.yaml``

### Visualize the Allan Deviation and Generate Kalibr Config

The next step is to visualize the plots and get parameters. For this run:

  ``python3 src/allan_variance_analysis.py --data allan_variance/allan_variance.csv``

  If you have a config file to set the topic and update rate, you can pass it in as well:

  ``python3 src/allan_variance_analysis.py --data allan_variance/allan_variance.csv --config config/realsense_d435i.yaml``

  Press `space` to go to next figure.

### Convert ROS1 bag to ROS2 mcap

If you have a ROS1 bag file, you can convert it to a ROS2 mcap file using the `convert_ros1_bag_to_ros2_mcap.sh` script. Note to run this script you must have installed the prerequisites with ROS2 or are in the [devcontainer](.devcontainer/README.md).

Run the script with the following arguments, replacing with the path to your ROS1 bag file, the name of the output ROS2 mcap file.

The mcap config yaml file should always be the provided `config/ros2_mcap_config.yaml` to convert all topics using standard mcap settings. This can be replaced with another config file if you have specific settings, see documentation for [ros2 bag convert](https://github.com/ros2/rosbag2?tab=readme-ov-file#converting-bags) for more information.

  ``scripts/convert_ros1_bag_to_ros2_mcap.sh bags/original_ros1_bag.bag mcaps/ros2_mcap_name.mcap config/ros2_mcap_config.yaml``

## How to record data from the RealSense D435i camera

See the [RealSense ROS2 Integration](docs/realsense_ros2.md) documentation for more information.

### Example mcap

3 hour log of [Realsense D435i IMU](https://drive.google.com/file/d/1ApjhpfRzOuL5jNdfbprn1sUrLSbCMdLX/view?usp=sharing) with timestamps already re-arranged.(Download and unzip the file)

![Acceleration](/figs/realsense_acceleration.png)
![Gyroscope](/figs/realsense_gyro.png)

Example terminal output:

```
ACCELEROMETER:
X Velocity Random Walk:  0.00333 m/s/sqrt(s)  0.19983 m/s/sqrt(hr)
Y Velocity Random Walk:  0.01079 m/s/sqrt(s)  0.64719 m/s/sqrt(hr)
Z Velocity Random Walk:  0.00481 m/s/sqrt(s)  0.28846 m/s/sqrt(hr)
X Bias Instability:  0.00055 m/s^2  7173.28800 m/hr^2
Y Bias Instability:  0.00153 m/s^2  19869.01200 m/hr^2
Z Bias Instability:  0.00052 m/s^2  6701.58000 m/hr^2
X Accel Random Walk:  0.00008 m/s^2/sqrt(s)
Y Accel Random Walk:  0.00020 m/s^2/sqrt(s)
Z Accel Random Walk:  0.00007 m/s^2/sqrt(s)
GYROSCOPE:
X Angle Random Walk:  0.00787 deg/sqrt(s)  0.47215 deg/sqrt(hr)
Y Angle Random Walk:  0.00987 deg/sqrt(s)  0.59204 deg/sqrt(hr)
Z Angle Random Walk:  0.00839 deg/sqrt(s)  0.50331 deg/sqrt(hr)
X Bias Instability:  0.00049 deg/s  1.76568 deg/hr
Y Bias Instability:  0.00136 deg/s  4.88153 deg/hr
Z Bias Instability:  0.00088 deg/s  3.15431 deg/hr
X Rate Random Walk:  0.00007 deg/s/sqrt(s)
Y Rate Random Walk:  0.00028 deg/s/sqrt(s)
Z Rate Random Walk:  0.00011 deg/s/sqrt(s)
```

## Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is a useful collection of tools for calibrating cameras and IMUs. For IMU calibration it needs the noise parameters of the IMU generated in a yaml file. `allan_variance_ros` automatically generates this file file as `imu.yaml`:

```
#Accelerometer
accelerometer_noise_density: 0.006308226052016165 
accelerometer_random_walk: 0.00011673723527962174 

#Gyroscope
gyroscope_noise_density: 0.00015198973532354657 
gyroscope_random_walk: 2.664506559330434e-06 

rostopic: '/sensors/imu' #Make sure this is correct
update_rate: 400.0 #Make sure this is correct

```
## Allan Variance ROS Evaluation

### IMU Noise Simulator

Thanks to [@kekeliu-whu](https://github.com/kekeliu-whu) who contributed an IMU noise simulator is based on the [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model). You can generate a rosbag of simulated IMU noise and run allan_variance_ros to verify the tool is working.
As shown in PR https://github.com/ori-drs/allan_variance_ros/pull/24 accuracy is quite good.

This tool is adapted in this repository to work with ROS2 and the MCAP format as a python script.

### To generate imu data with simulated noise

`python3 src/imu_simulator.py --yaml_config_file config/simulation/imu_simulator.yaml --output_mcap mcaps/imu_simulator.mcap`

A simulation config file is provided in `allan_variance_ros2/config/simulation/imu_simulator.yaml`

### To test Allan Variance ROS on simulated mcap file

  `python3 src/allan_variance_computer.py --input_mcap mcaps/imu_simulator/imu_simulator.mcap --output_dir allan_variance --imu_config_yaml config/sim.yaml`

  `python3 src/allan_variance_analysis.py --data allan_variance/allan_variance.csv`

A config file is provided in `allan_variance_ros2/config/sim.yaml`

## Author

[Jai Prajapati](https://www.ripl-lab.com/)

If you use this package for academic work, please consider using the citation below:

```
@software{AllanVarianceRos2,
  author       = {Jai Prajapati},
  title        = {Allan Variance ROS2},
  month        = Feb,
  year         = 2025,
  publisher    = {RIPL Lab, University of Waterloo},
  version      = {1.0},
  url          = {https://github.com/ripl-lab/allan_variance_ros2}
}
```

## References

- [Indirect Kalman Filter for 3D Attitude Estimation, Trawny & Roumeliotis](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
- [An introduction to inertial navigation, Oliver Woodman](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf) 
- [Characterization of Errors and Noises in MEMS Inertial Sensors Using Allan Variance Method, Leslie Barreda Pupo](https://upcommons.upc.edu/bitstream/handle/2117/103849/MScLeslieB.pdf?sequence=1&isAllowed=y)
- [Kalibr IMU Noise Documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [Allan Variance ROS](https://github.com/ori-drs/allan_variance_ros)
