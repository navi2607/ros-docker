#!/usr/bin/env python3.8

import rospy
from typing import Dict, Any, Callable

from utils import map_config_to_ros_objects, get_data_source


class Publisher:
    """ Wrapper class that when pinged publishes a message """

    def __init__(self, publisher_imp: Any, msg_rate: Any, stop_callback: Callable) -> None:
        """
        Initializes the publisher wrapper object.

        :param publisher_imp: Any object that has a publish() method that publishes messages
        :param msg_rate: Any object that has a sleep() method
        :param stop_callback: Callable object that breaks the __call__ generator  function

        :return None
        """
        self.publisher_imp = publisher_imp
        self.msg_rate = msg_rate
        self.stop_callback = stop_callback

        self.data_src = None
        self.default_data_src = None

        self.msg = None

    def __call__(self) -> str:
        """
        Generator function used to get data from some source like object and send
        the data.

        :return Produced message
        """
        self.msg = self.default_data_src.get_data()
        self._update_msg()

        while not self.stop_callback():
            self.msg_rate.sleep()
            self._update_msg()

            msg_data = str(self.msg)
            self.publisher_imp.publish(msg_data)

            yield msg_data

    def _update_msg(self) -> None:
        """
        Helper function that updates the current msg value

        :return: None
        """
        temp_msg = self.data_src.get_data()

        if temp_msg is not None and self.msg != temp_msg:
            self.msg = temp_msg


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
    default_source = get_data_source(cfg['default_source'])
    data_source, *other = get_data_source(cfg['data_source'])

    # Wrap the ROS publisher with a wrapper class
    publisher = Publisher(ros_publisher, rate, shutdown_callback)

    publisher.data_src = data_source
    publisher.default_data_src = default_source

    # Send data until interruption
    try:
        for response in publisher():
            print(f"Publisher msg ==> {response}")
    except rospy.ROSInterruptException:
        pass
    finally:
        # This is specific if the source is a background process
        try:
            other[0].stop()
            data_source.close()
        except Exception:
            pass
