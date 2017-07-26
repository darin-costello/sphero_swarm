#!/usr/bin/python


"""
A node to ease the use of multiple spheros 
"""
import subprocess

import rospy
from sphero_swarm.srv import AddSubscriber, AddSubscriberResponse

LAUNCHCODE = "roslaunch sphero_swarm sphero.launch name_space:={0} sphero_address={1}"


class SpheroSwarmNode(object):

    def __init__(self):
        rospy.init_node("sphero_swarm_node")
        self.name_space = rospy.get_namespace()
        self.names = set()
        self.addresses = set()
        self.processes = {}
        self._cmd_vel_pub = {}
        self._cmd_turn_pub = {}
        self._color_pub = {}
        self._back_led_pub = {}

        self._init_pub()

        self._init_sub()

        self._add_sub_srv = None
        self._init_services()

    def _init_services(self):
        self.add_sub_srv = rospy.Service(
            'add_subscriber', AddSubscriber, self.add_sub)

    def add_sub(self, msg):
        return AddSubscriberResponse(0)

    def add_sphero(self, name, address):
        if name in self.names or address in self.addresses:
            return False
        else:

    rospy.Sub
