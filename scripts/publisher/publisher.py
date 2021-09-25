#!/usr/bin/env python3.8

import rospy
from typing import Dict, Any, Callable

from utils import map_config_to_ros_objects, get_data_source


class Publisher:
    """ Wrapper class that when pinged publishes a message """

    def __init__(self, data_src: Any, publisher_imp: Any, msg_rate: Any, stop_callback: Callable) -> None:
        """
        Initializes the publisher wrapper object.

        :param data_src: Any object that follows the interface of the Source class in sources.py
        :param publisher_imp: Any object that has a publish() method that publishes messages
        :param msg_rate: Any object that has a sleep() method
        :param stop_callback: Callable object that breaks the __call__ generator  function

        :return None
        """
        self.data_src = data_src
        self.publisher_imp = publisher_imp
        self.msg_rate = msg_rate
        self.stop_callback = stop_callback

    def __call__(self) -> str:
        """
        Generator function used to get data from some source like object and send
        the data.

        :return Produced message
        """
        while not self.stop_callback():
            self.msg_rate.sleep()
            msg_data = self.data_src.get_data()
            msg_data = str(msg_data)
            self.publisher_imp.publish(msg_data)

            yield msg_data


def run_publisher(publisher_cfg: Dict[str, Any]) -> None:
    """
    Main function that manages the creation and initialization of all objects that are needed to
    publish messages using the ROS publisher.

    :param publisher_cfg: Publisher configuration dictionary

    :return None
    """
    # Create the ROS publisher
    cfg = map_config_to_ros_objects(publisher_cfg)
    ros_publisher = rospy.Publisher(cfg['topic_name'], cfg['msg_type'], queue_size=cfg['queue_size'])

    # Init ROS node
    rospy.init_node('talker', anonymous=True)

    # Set additional ROS dependencies
    rate = rospy.Rate(cfg['rate'])
    shutdown_callback = rospy.is_shutdown

    # Get a data source
    data_source = get_data_source(cfg['data_source'])

    # Wrap the ROS publisher with a wrapper class
    publisher = Publisher(data_source, ros_publisher, rate, shutdown_callback)

    # Send data until interruption
    try:
        for response in publisher():
            print(f"Publisher msg ==> {response}")
    except rospy.ROSInterruptException:
        pass
