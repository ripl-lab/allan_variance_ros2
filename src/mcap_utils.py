"""
@file   mcap_utils.py.

@brief  Utility functions for working with MCAP files.

@author Jai Prajapati
"""

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_messages(input_mcap: str):
    """
    Read messages from the input MCAP file.

    Args:
        input_mcap (str): Path to the input MCAP file.

    Yields
    ------
        tuple: A tuple containing topic, message, message type name, and timestamp.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_mcap, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        """
        Get the type name of a given topic.

        Args:
            topic_name (str): Name of the topic.

        Returns
        -------
            str: Type name of the topic.

        Raises
        ------
            ValueError: If the topic is not found in the bag.
        """
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg_typename = typename(topic)
        msg = deserialize_message(data, msg_type)
        yield topic, msg, msg_typename, timestamp
    del reader
