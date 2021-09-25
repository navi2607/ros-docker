import std_msgs
from typing import Dict, Any


ros_object_mapping = {
    "std_msgs.msg.String": std_msgs.msg.String,
}


def map_config_to_ros_objects(cfg: Dict[str, str]) -> Dict[str, Any]:
    """
    Maps any string that represents a class reference to the actual class

    :param cfg: Input dictionary where keys and values are strings
    :return: Output dictionary where keys are strings and values some specific class
    """
    processed_cfg = dict()
    for k, v in cfg.items():
        # Configuration can have nested dict, isinstance() takes care of that
        if not isinstance(v, dict) and v in ros_object_mapping.keys():
            processed_cfg[k] = ros_object_mapping[v]
        else:
            processed_cfg[k] = v

    return processed_cfg



