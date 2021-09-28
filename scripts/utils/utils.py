from std_msgs.msg import String
from typing import Dict, Any, TypeVar

Server = TypeVar('Server')
Handler = TypeVar('Handler')


ros_object_mapping = {
    "std_msgs.msg.String": String,
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


class ServerConfig:

    def __init__(self) -> None:
        """ Initialize server config object"""
        self._server_type = None
        self._handler_type = None
        self._address = None
        self._port = None

    @property
    def server_type(self) -> Server:
        """ Server type getter """
        return self._server_type

    @server_type.setter
    def server_type(self, server_type: Server) -> None:
        """ Server type setter """
        self._server_type = server_type

    @property
    def handler_type(self) -> Handler:
        """ Handler type getter """
        return self._handler_type

    @handler_type.setter
    def handler_type(self, handler_type: Handler) -> None:
        """ Handler type setter """
        self._handler_type = handler_type

    @property
    def address(self) -> str:
        """ Address type getter """
        return self._address

    @address.setter
    def address(self, address: str):
        """ Address type setter """
        self._address = address

    @property
    def port(self) -> int:
        """ Port type getter """
        return self._port

    @port.setter
    def port(self, port: int) -> None:
        """ Port type setter """
        self._port = port
