#!/usr/bin/env python3.8

import rospy
import std_msgs


class Sink:

    def insert_output(self):
        raise NotImplementedError


class DB(Sink):

    def __init__(self, db_engine):
        self.db_engine = db_engine

    def insert_output(self):
        pass


def sink_factory(sink_type):
    if sink_type == "Sqlite":
        return DB
    else:
        raise TypeError("Sink type unknown")


class ListenerConfig:

    def __init__(self, **kwargs):
        self.sink = kwargs.get('sink', None)
        self.topic_name = kwargs.get('topic_name', None)
        self.msg_type = kwargs.get('msg_type', None)
        self.callback = kwargs.get('callback', None)

    def __iter__(self):
        return ((attr, val) for attr, val in self.__dict__.items())


class Listener:

    def __init__(self, sink, listener):
        self.sink = sink_factory(sink.get('type'))
        self.listener = listener

    def __call__(self):
        rospy.spin()

    @classmethod
    def create(cls, config):
        """ Create publisher instance """
        for attr, val in config:
            if val is None:
                raise Exception("Config must be fully defined.")

        listener = rospy.Subscriber(config.topic_name, config.msg_type, config.callback)

        return cls(config.sink, listener)


def run_listener(listener):
    listener()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == "__main__":
    config = {
        'sink': {'type': 'Sqlite'},
        'topic_name': 'chatter',
        'msg_type': std_msgs.msg.String,
        'callback': callback
    }

    rospy.init_node('listener', anonymous=True)
    listener = Listener.create(ListenerConfig(**config))

    run_listener(listener)

