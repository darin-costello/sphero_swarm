#!/usr/bin/python


"""
A node to ease the use of multiple spheros
"""
import sys
from subprocess import Popen
from future.utils import viewitems
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32, Bool
from sphero_swarm.msg import SpheroSwarmTwist, SpheroSwarmTurn, SpheroSwarmColor, SpheroSwarmBackLed, SpheroSwarmDisableStabilization, SpheroSwarmHeading, SpheroSwarmAngularVelocity, SpheroSwarmOdom, SpheroSwarmImu, SpheroSwarmCollision
from sphero_swarm.srv import AddSphero, AddSpheroRequest, AddSpheroResponse
from sphero_swarm.srv import ListSphero, ListSpheroRequest, ListSpheroResponse
from sphero.msg import SpheroCollision

LAUNCHCODE = "roslaunch sphero_swarm sphero.launch name_space:={0} sphero_address={1}"
PUB_TOPICS = {'cmd_vel':  Twist,
              'cmd_turn': Float32,
              'set_color': ColorRGBA,
              'set_back_led': Float32,
              'disable_stabilization': Bool,
              'set_heading': Float32,
              'set_angular_velocity': Float32}


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

        self.odom_pub = None
        self.imu_pub = None
        self.collision_pub = None
        self._sphero_subscribers = {}
        self._init_pub()

        self.processes = {}
        self._start_all_spheros()

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
        self._subscribers['cmd_vel'] = rospy.Subscriber(
            'cmd_vel', SpheroSwarmTwist, self.forward_pub, callback_args='cmd_vel')
        self._subscribers['cmd_turn'] = rospy.Subscriber(
            'cmd_turn', SpheroSwarmTurn, self.forward_pub, callback_args='cmd_turn')
        self._subscribers['set_color'] = rospy.Subscriber(
            'set_color', SpheroSwarmColor, self.forward_pub, callback_args='set_color')
        self._subscribers['set_back_led'] = rospy.Subscriber(
            'set_back_led', SpheroSwarmBackLed, self.forward_pub, callback_args='set_back_led')
        self._subscribers['disable_stabilization'] = rospy.Subscriber(
            'disable_stabilization', SpheroSwarmDisableStabilization, self.forward_pub, callback_args='disable_stabilization')
        self._subscribers['set_heading'] = rospy.Subscriber(
            'set_heading', SpheroSwarmHeading, self.forward_pub, callback_args='set_heading')
        self._subscribers['set_angular_velocity'] = rospy.Subscriber(
            'set_angular_velocity', SpheroSwarmAngularVelocity, self.forward_pub, callback_args='set_angular_velocity')

    def _init_pub(self):
        self.odom_pub = rospy.Publisher('odom', SpheroSwarmOdom, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', SpheroSwarmImu, queue_size=1)
        self.collision_pub = rospy.Publisher(
            'collision', SpheroSwarmCollision, queue_size=1)

    def _start_all_spheros(self):
        spheros = self._spheros
        self._spheros = {}
        for (name, address) in viewitems(spheros):
            self.add_sphero(name, address)

    def _init_services(self):
        pass

    def add_sphero(self, name, address):
        if name in self._spheros or address in self._spheros.values():
            return False
        namespace = self.name_space + "/" + name
        launch = LAUNCHCODE.format(namespace + address)
        process = Popen(launch, shell=True)
        if process.poll() is not None:
            self.processes[name] = process
            self._spheros[name] = address
            sphero_pubs = {}
            for (pub_name, pub_type) in viewitems(PUB_TOPICS):
                sphero_pubs[pub_name] = rospy.Publisher(
                    namespace + "/" + pub_name, pub_type, queue_size=1)
            self._sphero_publishers[name] = sphero_pubs
            sphero_subs = {}
            sphero_subs['odom'] = rospy.Subscriber(
                namespace + '/odom', Odometry, queue_size=1)
            sphero_subs['imu'] = rospy.Subscriber(
                namespace + '/imu', Imu, queue_size=1)
            sphero_subs['collision'] = rospy.Subscriber(
                namespace + '/collison', SpheroCollision, queue_size=1)
            self._sphero_subscribers[name] = sphero_subs

    def forward_pub(self, msg, topic):
        if msg.name in self._sphero_publishers:
            self._sphero_publishers[msg.name][topic].publish(msg.data)

    def forward_odom(self, msg, name):
        message = SpheroSwarmOdom()
        message.name = name
        message.data = msg
        self.odom_pub.publish(message)

    def forward_imu(self, msg, name):
        message = SpheroSwarmImu()
        message.name = name
        message.data = msg
        self.imu_pub(message)

    def forward_collision(self, msg, name):
        message = SpheroSwarmCollision()
        message.name = name
        message.data = msg
        self.collision_pub(message)

    def spin(self):
        rate = rospy.Rate(self._refersh_rate)
        while not rospy.is_shutdown():
            for (name, process) in viewitems(self.processes):
                if process.poll is not None:
                    [x.unregister()
                     for x in self._sphero_publishers[name].values()]
                        [x.unregister()
                         for x in self._sphero_subscribers[name].values()]
                    del self._sphero_publishers[name]
                    del self._sphero_subscribers[name]
                if self._auto_reconnect:
                    self.add_sphero(name, self._spheros[name])
                else:
                    del self._spheros[name]
            rate.sleep()
