"""
@file   imu_simulator.py.

@brief  Class to simulate an IMU with noise and write to a ros2 mcap file.

@author Jai Prajapati
"""

import argparse
import os
import time
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class IMUSimulator:
    """Class to simulate an IMU with noise and write to a ros2 mcap file."""

    def __init__(self, yaml_config_file: str, output_mcap: str):
        """
        Initialize the IMUSimulator with configuration file, output mcap file, and IMU topic.

        :param yaml_config_file: Path to the YAML configuration file.
        :param output_mcap: Path to the output mcap file.
        :param imu_topic: Topic name for the IMU data.
        """
        self.yaml_config_file = yaml_config_file
        self.output_mcap = output_mcap
        # Get the directory of the output mcap file
        self.output_mcap_dir = os.path.join(Path(output_mcap).parent, Path(output_mcap).stem)
        # If starts with ./, remove the ./
        if self.output_mcap_dir.startswith('./'):
            self.output_mcap_dir = self.output_mcap_dir[2:]
        self.writer = rosbag2_py.SequentialWriter()
        self.imu_measurement = Imu
        self.imu_measurement_type = 'sensor_msgs/msg/Imu'
    
    def run(self):
        """Run the IMU simulator to generate and write IMU data with noise."""
        # Load the yaml config file
        self.load_yaml_config()

        # Open the writer and add a connection for writing IMU data
        self.writer.open(
            rosbag2_py.StorageOptions(uri=self.output_mcap_dir, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.rostopic_, type=self.imu_measurement_type, serialization_format="cdr"
            )
        )


        # Calculate the time step based on the update rate
        dt = 1 / self.update_rate_  

        # Get the current time which the mcap file will use as the start time
        start_time = time.time()

        # Initialize biases and real values for accelerometer and gyroscope
        accelerometer_bias = np.array([self.accelerometer_bias_init_, self.accelerometer_bias_init_, self.accelerometer_bias_init_])
        gyroscope_bias = np.array([self.gyroscope_bias_init_, self.gyroscope_bias_init_, self.gyroscope_bias_init_])
        accelerometer_real = np.array([0.0, 0.0, 0.0])
        gyroscope_real = np.array([0.0, 0.0, 0.0])

        for i in range(int(self.sequence_time_ * self.update_rate_)):
            # Update biases with random walk
            accelerometer_bias += self.random_normal_distribution_vector(self.accelerometer_random_walk_) * np.sqrt(dt)
            gyroscope_bias += self.random_normal_distribution_vector(self.gyroscope_random_walk_) * np.sqrt(dt)

            # Simulate measurements with noise
            accelerometer_measure = accelerometer_real + accelerometer_bias + self.random_normal_distribution_vector(self.accelerometer_noise_density_) / np.sqrt(dt)
            gyroscope_measure = gyroscope_real + gyroscope_bias + self.random_normal_distribution_vector(self.gyroscope_noise_density_) / np.sqrt(dt)

            # Calculate timestamp in nanoseconds
            timestamp_ns = int((i * dt + start_time) * 1e9)
            imu_measurement = np.array([timestamp_ns, accelerometer_measure[0], accelerometer_measure[1], accelerometer_measure[2], gyroscope_measure[0], gyroscope_measure[1], gyroscope_measure[2]])
            self.write_imu_measurement(imu_measurement)

    def load_yaml_config(self):
        """Load the YAML configuration file and set parameters for the simulation."""
        with open(self.yaml_config_file, 'r') as file:
            self.yaml_config = yaml.safe_load(file)

        # Load parameters from the YAML file
        self.accelerometer_noise_density_ = self.yaml_config.get("accelerometer_noise_density", 0.0)
        self.accelerometer_random_walk_ = self.yaml_config.get("accelerometer_random_walk", 0.0)
        self.accelerometer_bias_init_ = self.yaml_config.get("accelerometer_bias_init", 0.0)

        self.gyroscope_noise_density_ = self.yaml_config.get("gyroscope_noise_density", 0.0)
        self.gyroscope_random_walk_ = self.yaml_config.get("gyroscope_random_walk", 0.0)
        self.gyroscope_bias_init_ = self.yaml_config.get("gyroscope_bias_init", 0.0)

        self.rostopic_ = self.yaml_config.get("rostopic", "/imu/data_raw")
        self.update_rate_ = self.yaml_config.get("update_rate", 100)
        self.sequence_time_ = self.yaml_config.get("sequence_time", 0)
        print("rostopic: ", self.rostopic_)
        print("update_rate: ", self.update_rate_)
        print("sequence_time: ", self.sequence_time_)

    def random_normal_distribution_vector(self, sigma: float) -> np.array:
        """
        Generate a random vector with normal distribution.

        :param sigma: Standard deviation for the normal distribution.
        :return: Random vector with normal distribution.
        """
        rng = np.random.default_rng()
        nd = rng.normal(0, 1, 3)  # Generate a vector of 3 normally distributed random numbers
        return sigma * nd

    def write_imu_measurement(self, imu_measurement: np.array):
        """
        Write an IMU measurement to the mcap file.

        :param imu_measurement: numpy array containing the measurement data.
        """
        # Create an Imu message with the measurement data
        imu_msg: Imu = self.imu_measurement(
            header=Header(stamp=Time(sec=int(imu_measurement[0] / 1e9), nanosec=int(imu_measurement[0] % 1e9))),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            angular_velocity=Vector3(x=imu_measurement[4], y=imu_measurement[5], z=imu_measurement[6]),
            angular_velocity_covariance=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            linear_acceleration=Vector3(x=imu_measurement[1], y=imu_measurement[2], z=imu_measurement[3]),
            linear_acceleration_covariance=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )

        # Write the message to the mcap file
        self.writer.write(self.rostopic_, serialize_message(imu_msg), int(imu_measurement[0]))

    def __del__(self):
        """Destructor to ensure the mcap file is closed properly."""
        self.close()

    def close(self):
        """Close the mcap file."""
        del self.writer

def parse_args():
    """
    Parse the command line arguments for the IMU simulator.

    :return: Parsed arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml_config_file", type=str, required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--output_mcap", type=str, required=True, help="Path to the output mcap file.")
    return parser.parse_args()

def main():
    """Run the IMU simulator."""
    args = parse_args()
    imu_simulator = IMUSimulator(args.yaml_config_file, args.output_mcap)
    imu_simulator.run()

if __name__ == "__main__":
    main()
