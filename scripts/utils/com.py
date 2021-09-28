import io
import os
import socket
import subprocess
import string
import time
import random
from typing import Sequence


def get_default_socket_address(length=10) -> str:
    """
    This function generates a random name string with the length defined by the input parameter

    :param length: The length of the file name
    """
    random_name = str()
    for _ in range(length):
        random_name += random.choice(string.ascii_uppercase + string.digits)

    # The file will be created in the linux /tmp folder by default
    return os.path.join('/tmp', random_name)


class UnixDomainSocket:

    def __init__(self, name, socket_address: str = None) -> None:
        """ Initialize a unix domain socket"""

        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.name = name
        self.socket_address = socket_address

        self.connection = None
        self.client_address = None

    def open(self, num_of_connections: int = 1) -> None:
        """ Open a new connection """

        if self.socket_address is None:
            raise AttributeError("Socket address must be set.")

        print(20 * '*')
        print('\n')
        print(f"Binding socket to {self.socket_address}")
        self.socket.bind(self.socket_address)
        self.socket.listen(num_of_connections)
        print(20 * '*')
        print('\n')

    def accept_incoming_client(self) -> None:
        """ Accept any new incoming connection"""

        self.connection, self.client_address = self.socket.accept()
        print(f"Client with the address {self.client_address} connected.")
        print('\n')
        print(20*'*')

    def connect(self) -> None:
        """ Connect to a opened server port """
        if self.socket_address is None:
            raise ValueError("Socket address must be set.")

        print(f"Connecting to address {self.socket_address}")
        try:
            self.socket.connect(self.socket_address)
        except socket.error:
            raise
        finally:
            self.socket.setblocking(False)  # Socket is non blocking by default

        print(f"Connected to address {self.socket_address}")

    def close(self) -> None:
        """ Close socket connection """

        print(f"Closing connection to {self.socket_address}")
        self.socket.close()
        print(f"Cleaning up the socket file: {self.socket_address}")

        try:
            os.unlink(self.socket_address)
        except OSError:
            if os.path.exists(self.socket_address):
                raise

        print(f"File {self.socket_address} deleted")
        self.socket_address = None

    def send_to_client(self, data: bytes) -> None:
        """ Server socket method that send all data received from some source """
        self.connection.sendall(data)

    def get_data(self) -> bytes:
        """ Client socket method that fetches the data from the server port """
        data = None
        try:
            data = self.socket.recv(500)
        except socket.error:
            pass

        return data


class Subprocess:

    def __init__(self, name: str, command: Sequence, conn: UnixDomainSocket) -> None:
        """
        Initialize a subprocess instance

        :param name: Some random name for the process
        :param command: A sequence that has the specific command and arguments for the same command
        :param conn: A gateway that can be used to communicate with the underlying subprocess
        """

        self.name = name
        self.command = command
        self.conn = conn

        self.sub_process_instance = None
        self.process_stdin = None
        self.process_stdout = None
        self.process_stderr = None

    def spawn(self) -> None:
        """ Start a subprocess from the command set up in the initialization step """

        if self.sub_process_instance is None:
            self._spawn_process()
        else:
            print(f"Process still running.")
            self.stop()
            self._spawn_process()

        self.process_stdin = io.BytesIO()
        self.process_stdout = io.BytesIO()
        self.process_stderr = io.BytesIO()

    def _spawn_process(self) -> None:
        """ Helper function to the spawn() method, this one actually starts the subprocess """

        self.sub_process_instance = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

        time.sleep(1)
        response = self.sub_process_instance.poll()
        if response is None:
            print(f"Process with name '{self.name}' started successfully")

    def stop(self) -> None:
        """ Stop the referenced subprocess """

        try:
            print(f"Try to end process gracefully.")
            outs, errs = self.sub_process_instance.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"Process hasn't finnish gracefully, killing process.")
            self.sub_process_instance.kill()
        finally:
            print("Process stopped.")

    def is_alive(self) -> None:
        """ Method that polls the subprocess. """

        return True if self.sub_process_instance.poll() is None else self.sub_process_instance.poll()

    def get_data(self, what: str) -> None:
        """
        Method that fetches specific data from the subprocess. Data that is fetched is parametrized with the what argument

        :param what: String that is used to get specific data from subprocess
        """

        time.sleep(0.5)
        if self.sub_process_instance.poll():
            print(f"Try to receive data but process is not running")
            return

        what_bytes = bytes(what, encoding='utf8')
        what_bytes += b'\n'

        self.sub_process_instance.stdin.write(what_bytes)
        self.sub_process_instance.stdin.flush()

        while True:
            time.sleep(0.5)
            data = self.sub_process_instance.stdout.readline()
            if b'address' in data:  # If the received data is an address, next line should be the address
                address = self.sub_process_instance.stdout.readline()
                output = address.decode('utf-8').replace('\n', '')
                break

        return output
