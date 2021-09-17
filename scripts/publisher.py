#!/usr/bin/env python3.8

import rospy
import yaml
import std_msgs
import http
import http.server


def source_factory(src_type, cfg):
    if src_type == "Yaml":
        return YamlSource(**cfg)
    elif src_type == "HTTP":
        return SimpleServer()
    else:
        raise TypeError("Source type unknown")


class Source:

    def parse_input(self):
        raise NotImplementedError


class YamlSource(Source):

    def __init__(self, yaml_file):
        self.yaml_file = yaml_file

    def parse_input(self):
        return yaml.parse(self.yaml_file, Loader=yaml.FullLoader)


class PutHandler(http.server.BaseHTTPRequestHandler):

    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.post_data = None

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])  # <--- Gets the size of data
        self.post_data = self.rfile.read(content_length)  # <--- Gets the data itself

        self._set_response()
        # self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))


class SimpleServer(Source):

    def __init__(self, address='localhost', port=8080):
        self.address = address
        self.port = port
        self.server = http.server.HTTPServer((self.address, self.port), PutHandler)
        self.server.serve_forever()

    def parse_input(self):
        return


class PublisherConfig:

    def __init__(self, **kwargs):
        self.source = kwargs.get('source', None)
        self.topic_name = kwargs.get('topic_name', None)
        self.msg_type = kwargs.get('msg_type', None)
        self.queue_size = kwargs.get('queue_size', None)
        self.rate = kwargs.get('rate', None)

    def __iter__(self):
        return ((attr, val) for attr, val in self.__dict__.items())


class Publisher:

    def __init__(self, source, publisher, rate):
        self.source = source_factory(source.get('type'), source.get('cfg', dict()))
        self.publisher = publisher
        self.rate = rate

    def __call__(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publisher.publish("Hello World")
            yield "Hello World"

    @classmethod
    def create(cls, config):
        """ Create publisher instance """
        for attr, val in config:
            if val is None:
                raise Exception("Config must be fully defined.")

        publisher = rospy.Publisher(config.topic_name, config.msg_type, queue_size=config.queue_size)

        return cls(config.source, publisher, config.rate)


def run_publisher(publisher):
    rospy.init_node('talker', anonymous=True)
    for response in publisher():
        rospy.logdebug(response)


if __name__ == "__main__":
    config = {
        'source': {'type': 'HTTP'},
        'topic_name': 'chatter',
        'msg_type': std_msgs.msg.String,
        'queue_size': 10,
        'rate': 10
    }

    publisher = Publisher.create(PublisherConfig(**config))

    try:
        run_publisher(publisher)
    except rospy.ROSInterruptException:
        pass
