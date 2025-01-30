"""
@file   allan_variance_computer.py.

@brief  Script to calculate the Allan variance of an IMU.
        The script reads IMU data from an MCAP file, processes it, and computes the Allan variance.
        Results are saved to a CSV file.

@author Jai Prajapati
"""

import argparse
import os

import numpy as np
import yaml
from tqdm import tqdm

from mcap_utils import read_messages


class AllanVarianceComputer:
    """Class to compute the Allan variance of an IMU."""

    def __init__(self, input_mcap, output_dir, imu_config_yaml):
        """
        Initialize the AllanVarianceComputer with input MCAP file, output directory, and IMU configuration YAML.
        
        :param input_mcap: Path to the input MCAP file containing IMU data.
        :param output_dir: Directory where the output CSV file will be saved.
        :param imu_config_yaml: Path to the YAML configuration file for the IMU.
        """
        self.input_mcap = input_mcap
        self.output_dir = output_dir
        self.imu_config_yaml = imu_config_yaml
        self.imu_output_csv_file = os.path.join(output_dir, "allan_variance.csv")

    def run(self):
        """
        Run the Allan variance computation process.

        Loads the YAML configuration, reads IMU data, and computes the Allan variance.
        """
        self.load_yaml_config()

        imu_measurements = []  # List to store IMU measurements
        
        imu_counter = 0
        skipped_imu_counter = 0
        first_msg = True
        first_time = None
        last_imu_time = None

        # Initialize the progress bar
        with tqdm(total=self.sequence_time, desc="Loading IMU data") as pbar:
            # Read messages from the input MCAP file
            for topic, msg, _msg_typename, timestamp in read_messages(self.input_mcap):
                if topic == self.imu_topic:
                    imu_counter += 1

                    # Skip messages based on the configured skip rate
                    if imu_counter % self.imu_skip_ != 0:
                        continue

                    # Initialize first message timestamp
                    if first_msg:
                        first_msg = False
                        first_time = timestamp
                        last_imu_time = timestamp
                        continue

                    # Update the progress bar
                    pbar.update((timestamp - last_imu_time) / 1e9)  # Assuming timestamp is in nanoseconds

                    # Check for out-of-order messages
                    if timestamp < last_imu_time:
                        skipped_imu_counter += 1
                        print(f"IMU out of order. Current(ns): {timestamp - first_time}, Last(ns): {last_imu_time - first_time} {skipped_imu_counter} dropped")
                        continue

                    last_imu_time = timestamp

                    # Convert gyro measurements from radians to degrees and store them
                    imu_measurement = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                                msg.angular_velocity.x * 180 / np.pi, msg.angular_velocity.y * 180 / np.pi, msg.angular_velocity.z * 180 / np.pi])

                    imu_measurements.append(imu_measurement)

        imu_measurements = np.array(imu_measurements)

        print(f"Loaded {len(imu_measurements)} IMU measurements")

        if len(imu_measurements) == 0:
            print("No IMU measurements found, is your topic correct?")
            return

        # Compute the Allan variance
        self.compute_allan_variance(imu_measurements)

    def compute_allan_variance(self, imu_measurements):
        """
        Compute the Allan variance of the IMU measurements and save results to a CSV file.
        
        :param imu_measurements: Numpy array of IMU measurements.
        """
        averages_map = {}

        # Range of periods to sample from (0.1s to 1000s)
        period_min = 1
        period_max = 10000

        overlap = 0.0

        # Open CSV file for writing Allan variance results
        with open(self.imu_output_csv_file, "w") as imu_output_csv_file_writer:

            # Initialize the progress bar for period computation
            with tqdm(total=period_max - period_min, desc="Computing periods") as period_pbar:
                # Compute averages for each period
                for period in range(period_min, period_max):

                    averages_map[period] = []
                    averages = []

                    period_time = period * 0.1

                    max_bin_size = int(period_time * self.measure_rate)
                    overlap = int(np.floor(max_bin_size * overlap))

                    # Calculate averages for each bin
                    for i in range(0, len(imu_measurements) - max_bin_size, max_bin_size - overlap):
                        current_average = np.mean(imu_measurements[i:i + max_bin_size], axis=0)
                        averages.append(current_average)

                    averages_map[period] = averages
                    period_pbar.update(1)  # Update the progress bar

            # Calculate Allan variance and deviation for each period
            for period in range(period_min, period_max):
                averages = averages_map[period]
                if len(averages) == 0:
                    continue

                period_time = period * 0.1
                num_averages = len(averages)

                allan_variance = np.zeros(6)
                for i in range(num_averages - 1):
                    allan_variance += np.square(averages[i + 1] - averages[i])

                allan_variance /= 2 * (num_averages - 1)

                allan_deviation = np.sqrt(allan_variance)

                # Write results to CSV
                imu_output_csv_file_writer.write(f"{period_time} {allan_deviation[0]} {allan_deviation[1]} {allan_deviation[2]} {allan_deviation[3]} {allan_deviation[4]} {allan_deviation[5]}\n")

    def load_yaml_config(self):
        """Load the YAML configuration file and set parameters for IMU data processing."""
        with open(self.imu_config_yaml, 'r') as file:
            self.yaml_config = yaml.safe_load(file)

        # Load parameters from the YAML file
        self.imu_topic = self.yaml_config.get("imu_topic", "/imu/data_raw")
        self.imu_rate = self.yaml_config.get("imu_rate", 100)
        self.measure_rate = self.yaml_config.get("measure_rate", 100)
        self.sequence_time = self.yaml_config.get("sequence_time", 0)

        # Calculate the skip rate for IMU messages
        self.imu_skip_ = int(self.imu_rate / self.measure_rate)

def parse_args():
    """
    Parse command line arguments for the script.
    
    :return: Parsed arguments containing input MCAP file, output directory, and IMU configuration YAML path.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_mcap", type=str, required=True, help="Input MCAP filepath")
    parser.add_argument("--output_dir", type=str, required=True, help="Output directory for csv file of allan variance")
    parser.add_argument("--imu_config_yaml", type=str, required=True, help="Path to the IMU configuration yaml file")
    args = parser.parse_args()
    return args

def main():
    """
    Calculate the Allan variance of an IMU.

    Initializes the AllanVarianceComputer and runs the computation.
    """
    args = parse_args()
    input_mcap = args.input_mcap
    output_dir = args.output_dir
    imu_config_yaml = args.imu_config_yaml

    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    allan_variance_computer = AllanVarianceComputer(input_mcap, output_dir, imu_config_yaml)
    allan_variance_computer.run()

if __name__ == "__main__":
    main()
