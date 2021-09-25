#!/usr/bin/env python3.8

import http.server
import sys

from utils.com import UnixDomainSocket, get_default_socket_address
from utils.utils import ServerConfig
from utils.sources import PutHandler, SimpleServer

if __name__ == '__main__':
    # Set some global config for server
    config = ServerConfig()
    config.server_type = http.server.HTTPServer
    config.handler_type = PutHandler
    config.address = '0.0.0.0'
    config.port = 5432

    # Create a random file for the unix domain socket
    socket_address = get_default_socket_address()
    socket = UnixDomainSocket('Server', socket_address)

    # Write the server unix domain socket address to output
    sys.stdout.write('address\n')
    sys.stdout.write(socket_address + '\n')

    # Wait for a client to connect
    socket.open()
    socket.accept_incoming_client()

    # Start the server
    server = SimpleServer(config, socket)
    server.spin_up()
