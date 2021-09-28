import abc
import http.server

import yaml
from typing import Dict, Any, TypeVar, Generic

import utils.com as com

DataSource = TypeVar('DataSource')
Data = TypeVar('Data', dict, Any)
ServerConfig = TypeVar('ServerConfig', dict, Any)
Socket = TypeVar('Socket')


def get_data_source(source_cfg: Dict[str, Any]) -> Generic[DataSource]:
    """
    Factory method for the data sources

    :param source_cfg: dictionary like configuration
    :return: a data source object implementation
    """
    try:
        source_type = source_cfg.get('type')
    except AttributeError:  # If the config is not a dict
        source_type = source_cfg

    if source_type == 'YAML':
        yaml_file = source_cfg.get('file', None)

        return YamlSource(yaml_file)
    elif source_type == 'Server':
        # Run the server that handles the requests
        socket = com.UnixDomainSocket('MainProgram')
        command = ['python3.8', '/tmp/scripts/server.py']
        server = com.Subprocess('server', command, socket)

        try:
            server.spawn()
            # Get address from server
            while socket.socket_address is None:
                address = server.get_data('address')
                if address:
                    socket.socket_address = address
                    break

            socket.connect()
        except Exception:
            server.stop()
            raise
        finally:
            print(f"Server '{server.name}' started")

        return socket, server

    else:
        raise ValueError(f"Data source {source_type} not supported.")


class Source(abc.ABC):
    """
    Abstract data source class
    """

    @abc.abstractmethod
    def get_data(self) -> Data:
        """
        Abstract method that fetches data from some source
        """
        pass


class FileSource(Source):
    """
    Abstract data source class specific for file like sources
    """

    @abc.abstractmethod
    def get_data(self) -> Data:
        """
        Abstract method that fetches data from some source. This method
        uses the parse_file method to get the data from a file.
        """
        pass

    @abc.abstractmethod
    def parse_file(self) -> Data:
        """
        Abstract method that parse data from the file
        """
        pass


class YamlSource(FileSource):

    def __init__(self, yaml_file: str) -> None:
        self.yaml_file = yaml_file

    def get_data(self) -> Data:
        """
        Fetch data from a yaml file

        :return Data that has the file content
        """
        return self.parse_file()

    def parse_file(self) -> Data:
        """
        Opens the file and parses the file content.

        :return dictionary like data
        """
        with open(self.yaml_file, 'r') as yaml_file:
            data = yaml.load(yaml_file, Loader=yaml.FullLoader)

        return data


class ServerSource(Source):

    @abc.abstractmethod
    def get_data(self) -> Data:
        pass

    @abc.abstractmethod
    def spin_up(self) -> None:
        pass


class SimpleServer(ServerSource):

    def __init__(self, config: ServerConfig, conn: Socket) -> None:
        """
        Initializes the server

        :param config: Server configuration
        :param conn: Socket that accepts a client connection
        """
        self.server_type = config.server_type
        self.handler_type = config.handler_type
        self.address = (config.address, config.port)
        self.conn = conn

        self.server_instance = None

    def spin_up(self) -> None:
        """ Method that starts the server. After that the server can receive requests. """

        if self.server_instance is None:
            self.server_instance = self.server_type(self.address, self.handler_type)

        self.handler_type.put_socket = self.conn  # This I dont like
        self.server_instance.serve_forever()

    def get_data(self):
        pass


class PutHandler(http.server.SimpleHTTPRequestHandler):

    put_socket = None

    def _set_response(self) -> None:
        """ Returns a response to a request """
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_POST(self) -> None:
        """ Method that parses a POST request and pushes the request to a socket gateway """

        content_length = int(self.headers['Content-Length'])  # <--- Gets the size of data
        post_data = self.rfile.read(content_length)  # <--- Gets the data itself
        self._set_response()
        self.put_socket.send_to_client(post_data)

    def handle(self) -> None:
        """ Helper method that handles requests. """
        self.close_connection = True

        self.handle_one_request()
        while not self.close_connection:
            self.handle_one_request()