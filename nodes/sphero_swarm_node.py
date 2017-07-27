#!/usr/bin/python


"""
A node to ease the use of multiple spheros
"""
import sys
import subprocess

import rospy
from sphero_swarm.msg import SpheroTwist, SpheroTurn, SpheroColor, SpheroBackLed, SpheroDisableStabilization, SpheroHeading, SpheroAngularVelocity
from sphero_swarm.srv import AddSubscriber, AddSubscriberResponse

LAUNCHCODE = "roslaunch sphero_swarm sphero.launch name_space:={0} sphero_address={1}"
PUB_TOPICS = ['cmd_vel', 'cmd_turn', 'set_color', 'set_back_led',
              'disable_stabilization', 'set_heading', 'set_angular_velocity']


class SpheroSwarmNode(object):

    def __init__(self):
        rospy.init_node("sphero_swarm_node")
        self.name_space = rospy.get_namespace()

        self._spheros = {}
        self._auto_reconnect = False
        self._refersh_rate = 10
        self._init_params()

        self._sphero_publishers = {}
        self._subscribers = {}
        self._init_sub()

        self.processes = {}

        self._add_sub_srv = None
        self._init_services()

    def _init_params(self):
        self._refersh_rate = rospy.get_param("~refersh_rate", 10)
        self._auto_reconnect = rospy.get_param("~auto_reconnect", False)
        self._spheros = rospy.get_param("sphero_swarm/swarm", {})
        if len(self._spheros) != len(set(self._spheros.values())):
            rospy.logerr("Each sphero needs a unique address")
            sys.exit(1)

    def _init_sub(self):
        self._sphero_publishers = {topic: {} for topic in PUB_TOPICS}
        self._subscribers['cmd_vel'] = rospy.Subscriber(
            'cmd_vel', SpheroTwist, self.forward_sub, callback_args='cmd_vel')
        self._subscribers['cmd_turn'] = rospy.Subscriber(
            'cmd_turn', SpheroTurn, self.forward_sub, callback_args='cmd_turn')
        self._subscribers['set_color'] = rospy.Subscriber(
            'set_color', SpheroColor, self.forward_sub, callback_args='set_color')
        self._subscribers['set_back_led'] = rospy.Subscriber(
            'set_back_led', SpheroBackLed, self.forward_sub, callback_args='set_back_led')
        self._subscribers['disable_stabilization'] = rospy.Subscriber(
            'disable_stabilization', SpheroDisableStabilization, self.forward_sub, callback_args='disable_stabilization')
        self._subscribers['set_heading'] = rospy.Subscriber(
            'set_heading', SpheroHeading, self.forward_sub, callback_args='set_heading')
        self._subscribers['set_angular_velocity'] = rospy.Subscriber(
            'set_angular_velocity', SpheroAngularVelocity, self.forward_sub, callback_args='set_angular_velocity')

    def _init_services(self):
        self.add_sub_srv = rospy.Service(
            'add_subscriber', AddSubscriber, self.add_sub)

    def add_sub(self, msg):
        return AddSubscriberResponse(0)

    def add_sphero(self, name, address):
        if name in self._spheros or address in self._spheros.values():
            return False

    def forward_sub(self, msg, topic):
        publisher = self._sphero_publishers[topic]
        if msg.name in publisher:
            publisher[msg.name].publish(msg.data)
