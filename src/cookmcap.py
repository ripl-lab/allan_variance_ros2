#!/usr/bin/env python3

"""
@file   cookmcap.py.

@brief  Script to reorganize a ROS2 mcap file by message timestamps.

@author Jai Prajapati
"""

import argparse
import os
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message

from mcap_utils import read_messages


def parse_args():
    """
    Parse command line arguments.

    Returns
    -------
        argparse.Namespace: Parsed command line arguments.
    """
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", type=str, default=None, help="Input MCAP filepath")
    parser.add_argument("--output", type=str, default=None, help="Output MCAP filename, will be placed in a directory with this same name")

    args = parser.parse_args()
    return args

def main():
    """Reorganize the MCAP file by message timestamps."""
    args = parse_args()
    input_mcap = args.input
    output_mcap = args.output

    # Get the directory of the output mcap file
    output_mcap_dir = os.path.join(Path(output_mcap).parent, Path(output_mcap).stem)
    # If starts with ./, remove the ./
    if output_mcap_dir.startswith('./'):
        output_mcap_dir = output_mcap_dir[2:]

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_mcap_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    created_topics = set()

    for topic, msg, msg_typename, _timestamp in read_messages(input_mcap):
        if topic not in created_topics:
            # Create a new topic in the output bag if it doesn't exist
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic, type=msg_typename, serialization_format="cdr"
                )
            )
            created_topics.add(topic)

        # Determine the message timestamp based on the message type
        if msg_typename == "sensor_msgs/msg/Imu":
            msg_timestamp_nanosec = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        elif msg_typename == "tf2_msgs/msg/TFMessage":
            msg_timestamp_nanosec = msg.transforms[0].header.stamp.sec * 1e9 + msg.transforms[0].header.stamp.nanosec
        else:
            continue

        # Write the message to the output bag with the correct timestamp
        writer.write(topic, serialize_message(msg), int(msg_timestamp_nanosec))

    del writer

if __name__ == "__main__":
    main()

