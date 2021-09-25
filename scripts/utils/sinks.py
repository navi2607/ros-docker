import rospy
import sys
from typing import Dict, Any, TypeVar, Generic


DataSink = TypeVar('DataSink')
Data = TypeVar('Data', dict, Any)


def get_data_sink(sink_cfg: Dict[str, Any]) -> Generic[DataSink]:
    """
    Factory method for the data sinks

    :param sink_cfg: dictionary like configuration
    :return: a data sink object implementation
    """
    try:
        sink_type = sink_cfg.get('type', sink_cfg)
    except AttributeError:  # If the config is not a dict
        sink_type = sink_cfg

    if sink_type == 'console':
        return Console()
    else:
        raise ValueError(f"Data sink {sink_type} not supported.")


class Console:
    """ Simple class that prints the data content on the console """

    def __call__(self, data: Data) -> None:
        """
        Every time the object is called it prints the data to the console

        :return: None
        """
        print(f"Listener received msg ==> {data}", file=sys.stdout)
