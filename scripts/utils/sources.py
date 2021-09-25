import yaml
import abc
from typing import Dict, Any, TypeVar, Generic

DataSource = TypeVar('DataSource')
Data = TypeVar('Data', dict, Any)


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
