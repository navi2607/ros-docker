import sys
import sqlite3
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
    if sink_type == 'db':
        conn = sqlite3.connect('/tmp/db/message.db')
        cursor = conn.cursor()

        table_ddl = """CREATE TABLE IF NOT EXISTS message (
                msg VARCHAR(512)
            )"""

        cursor.execute(table_ddl)
        conn.commit()
        conn.close()

        return SqlLite('/tmp/db/message.db')
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


class SqlLite:

    def __init__(self, db) -> None:
        self.db = db

    def __call__(self, data: Data) -> None:
        self.insert(data)

    def insert(self, data):
        table_dml = """ INSERT INTO message (msg) VALUES (?) """

        conn = sqlite3.connect(self.db)
        cursor = conn.cursor()
        cursor.execute(table_dml, (str(data),))
        conn.commit()
        conn.close()