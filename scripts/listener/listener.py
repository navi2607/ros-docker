#!/usr/bin/env python3.8

import rospy
from typing import Any, Dict

from utils import map_config_to_ros_objects, get_data_sink


class Listener:
    """ Wrapper class that listens to any incoming messages """

    def __init__(self, listener_imp: Any) -> None:
        """
        Initializes the listener wrapper object.

        :param listener_imp: Any object that has a listen() method that accepts incoming messages
        """
        self.listener_imp = listener_imp

    def listen(self) -> None:
        """ Start to listen on incomming messages. """
        self.listener_imp.listen()


def run_listener(listener_cfg: Dict[str, Any]) -> None:
    """
    Main function that manages the creation and initialization of all objects that are needed to
    listen for incoming using the ROS Subscriber.

    :param listener_cfg:
    """
    # Create the ROS listener
    cfg = map_config_to_ros_objects(listener_cfg)

    # Get a data source
    data_sink = get_data_sink(cfg['data_sink'])

    ros_listener = rospy.Subscriber(cfg['topic_name'], cfg['msg_type'], data_sink)

    # Must init ROS
    rospy.init_node('talker', anonymous=True)

    # Spin up ROS server
    rospy.spin()

    # Wrap the ROS listener with a wrapper class and listen for incoming messages
    listener = Listener(ros_listener)
    listener.listen()
