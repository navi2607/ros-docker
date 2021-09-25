import io
import os
import socket
import subprocess
import string
import sys
import time
import random


def get_default_socket_address(length=10):
    random_name = str()
    for _ in range(length):
        random_name += random.choice(string.ascii_uppercase + string.digits)

    # The file will be created in the linux /tmp folder by default
    return os.path.join('/tmp', random_name)


class UnixDomainSocket:

    def __init__(self, name, socket_address=None):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.name = name
        self.socket_address = socket_address

        self.connection = None
        self.client_address = None

    def open(self, num_of_connections=1):
        if self.socket_address is None:
            raise AttributeError("Socket address must be set.")

        print(20 * '*')
        print('\n')
        print(f"Binding socket to {self.socket_address}")
        self.socket.bind(self.socket_address)
        self.socket.listen(num_of_connections)
        print(20 * '*')
        print('\n')

    def accept_incoming_client(self):
        self.connection, self.client_address = self.socket.accept()
        print(f"Client with the address {self.client_address} connected.")
        print('\n')
        print(20*'*')

    def connect(self):
        if self.socket_address is None:
            raise ValueError("Socket address must be set.")

        print(f"Connecting to address {self.socket_address}")
        try:
            self.socket.connect(self.socket_address)
        except socket.error:
            raise

        print(f"Connected to address {self.socket_address}")

    def close(self):
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

    def send_to_client(self, data):
        self.connection.sendall(data)

    def get_data(self):
        data = self.socket.recv(500)
        print(data, file=sys.stdout)

        return data


class Subprocess:

    def __init__(self, name, command, conn):
        self.name = name
        self.command = command
        self.conn = conn

        self.sub_process_instance = None
        self.process_stdin = None
        self.process_stdout = None
        self.process_stderr = None

    def spawn(self):
        if self.sub_process_instance is None:
            self._spawn_process()
        else:
            print(f"Process still running.")
            self.stop()
            self._spawn_process()

        self.process_stdin = io.BytesIO()
        self.process_stdout = io.BytesIO()
        self.process_stderr = io.BytesIO()

    def _spawn_process(self):
        self.sub_process_instance = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

        time.sleep(1)
        response = self.sub_process_instance.poll()
        if response is None:
            print(f"Process with name '{self.name}' started successfully")

    def stop(self):
        try:
            print(f"Try to end process gracefully.")
            outs, errs = self.sub_process_instance.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"Process hasn't finnish gracefully, killing process.")
            self.sub_process_instance.kill()
        finally:
            print("Process stopped.")

    def is_alive(self):
        return True if self.sub_process_instance.poll() is None else self.sub_process_instance.poll()

    def get_data(self, what):
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
