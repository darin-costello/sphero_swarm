#!/usr/bin/python


"""
A node to ease the use of multiple spheros
"""
import sys
from subprocess import Popen
from future.utils import viewitems
import rospy
from geometry_msgs.msg import Twist as GeometryTwist
from sensor_msgs.msg import Imu as SensorImu
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32, Bool

from sphero_swarm.msg import Turn, Color, BackLed, DisableStabilization, Heading, AngularVelocity, Odom, Collision, Twist, Imu
from sphero_swarm.srv import SpheroInfo, SpheroInfoResponse
from sphero_swarm.srv import ListSphero, ListSpheroResponse
from sphero.msg import SpheroCollision

LAUNCHCODE = "roslaunch sphero_swarm sphero.launch name_space:={0} sphero_address:={1} name:={2}"

LAUNCHCODE = "ROS_NAMESPACE={0} rosrun sphero sphero_node.py _bt_addr:={1} _name:={2}"
PUB_TOPICS = {'cmd_vel':  GeometryTwist,
              'cmd_turn': Float32,
              'set_color': ColorRGBA,
              'set_back_led': Float32,
              'disable_stabilization': Bool,
              'set_heading': Float32,
              'set_angular_velocity': Float32}


class SpheroSwarmNode(object):

    """
    A ros node for controlling a swarm of spheros
    """

    def __init__(self):
        rospy.init_node("sphero_swarm_node")
        self.name_space = rospy.get_namespace()
        rospy.logdebug("namespace : %s", self.name_space)
        rospy.on_shutdown(self.stop)
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

        self._add_sphero_srv = None
        self._remove_sphero_srv = None
        self._list_sphero_srv = None
        self._init_services()

    def _init_params(self):
        self._refersh_rate = rospy.get_param("~refersh_rate", 10)
        self._auto_reconnect = rospy.get_param("~auto_reconnect", True)
        self._spheros = rospy.get_param("sphero_swarm/swarm", {})
        if len(self._spheros) != len(set(self._spheros.values())):
            rospy.logerr("Each sphero needs a unique address")
            sys.exit(1)

    def _init_sub(self):
        self._subscribers['cmd_vel'] = rospy.Subscriber(
            'cmd_vel', Twist, self.forward_pub, callback_args='cmd_vel')
        self._subscribers['cmd_turn'] = rospy.Subscriber(
            'cmd_turn', Turn, self.forward_pub, callback_args='cmd_turn')
        self._subscribers['set_color'] = rospy.Subscriber(
            'set_color', Color, self.forward_pub, callback_args='set_color')
        self._subscribers['set_back_led'] = rospy.Subscriber(
            'set_back_led', BackLed, self.forward_pub, callback_args='set_back_led')
        self._subscribers['disable_stabilization'] = rospy.Subscriber(
            'disable_stabilization', DisableStabilization, self.forward_pub, callback_args='disable_stabilization')
        self._subscribers['set_heading'] = rospy.Subscriber(
            'set_heading', Heading, self.forward_pub, callback_args='set_heading')
        self._subscribers['set_angular_velocity'] = rospy.Subscriber(
            'set_angular_velocity', AngularVelocity, self.forward_pub, callback_args='set_angular_velocity')

    def _init_pub(self):
        self.odom_pub = rospy.Publisher('odom', Odom, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.collision_pub = rospy.Publisher(
            'collision', Collision, queue_size=1)

    def _start_all_spheros(self):
        spheros = self._spheros
        self._spheros = {}
        for (name, address) in viewitems(spheros):
            rospy.logdebug("starting %s", name)
            self.add_sphero(name, address)

    def _init_services(self):
        self._add_sphero_srv = rospy.Service(
            'add_sphero', SpheroInfo, self.add_sphero_srv)
        self._remove_sphero_srv = rospy.Service(
            'remove_sphero', SpheroInfo, self.remove_sphero_srv)
        self._list_sphero_srv = rospy.Service(
            'list_spheros', ListSphero, self.list_sphero_srv)

    def add_sphero_srv(self, info):
        """
        service call to add a new sphero
        """
        response = self.add_sphero(info.name, info.address)
        return SpheroInfoResponse(1) if response else SpheroInfoResponse(0)

    def remove_sphero_srv(self, info):
        """
        service call to remove sphero
        """
        self.remove_sphero(info.name)
        return SpheroInfoResponse(1)

    def list_sphero_srv(self, unused):
        """
        service call to list all spheros
        """
        return ListSpheroResponse(list(self._spheros.keys()))

    def add_sphero(self, name, address):
        """
        starts the given sphero
        """
        if name in self._spheros or address in self._spheros.values():
            return False
        namespace = self.name_space + name
        launch = LAUNCHCODE.format(namespace, address, name)
        process = Popen(launch, shell=True)
        if process.poll() is None:
            self.processes[name] = process
            self._spheros[name] = address
            sphero_pubs = {}
            for (pub_name, pub_type) in viewitems(PUB_TOPICS):
                sphero_pubs[pub_name] = rospy.Publisher(
                    namespace + "/" + pub_name, pub_type, queue_size=1)
            self._sphero_publishers[name] = sphero_pubs
            sphero_subs = {}
            sphero_subs['odom'] = rospy.Subscriber(
                namespace + '/odom', Odometry, self.forward_odom, callback_args=name, queue_size=1)
            sphero_subs['imu'] = rospy.Subscriber(
                namespace + '/imu', SensorImu, self.forward_imu, callback_args=name, queue_size=1)
            sphero_subs['collision'] = rospy.Subscriber(
                namespace + '/collision', SpheroCollision, self.forward_collision, callback_args=name, queue_size=1)
            self._sphero_subscribers[name] = sphero_subs
            return True
        else:
            rospy.logerr("NOT CONNECTED...ISH %s", process.poll())
            return False

    def remove_sphero(self, name):
        """
        removes and stops the given sphero
        """
        [x.unregister()
         for x in self._sphero_publishers[name].values()]
        [x.unregister()
         for x in self._sphero_subscribers[name].values()]
        del self._sphero_publishers[name]
        del self._sphero_subscribers[name]
        del self._spheros[name]
        process = self.processes[name]
        del self.processes[name]
        if process.poll() is not None:
            try:
                process.kill()
            except:
                pass

    def forward_pub(self, msg, topic):
        """
        forwards messages to spheros
        """
        if msg.name in self._sphero_publishers:
            self._sphero_publishers[msg.name][topic].publish(msg.data)

    def forward_odom(self, msg, name):
        """
        Combines sphero odom messages to single topic
        """
        message = Odom()
        message.name = name
        message.data = msg
        self.odom_pub.publish(message)

    def forward_imu(self, msg, name):
        """
        combines sphero imu message to single topci
        """
        message = Imu()
        message.name = name
        message.data = msg
        self.imu_pub.publish(message)

    def forward_collision(self, msg, name):
        """
        combines sphero collision messages to single topic
        """
        message = Collision()
        message.name = name
        message.data = msg
        self.collision_pub.publish(message)

    def spin(self):
        """
        spin call
        """
        rate = rospy.Rate(self._refersh_rate)
        while not rospy.is_shutdown():

            for (name, process) in viewitems(self.processes.copy()):
                if process.poll() is not None:
                    rospy.loginfo("%s is disconnected", name)
                    address = self._spheros[name]
                    self.remove_sphero(name)
                    if self._auto_reconnect:
                        self.add_sphero(name, address)
            rate.sleep()

    def stop(self):
        """
        Stops all the spheros
        """
        print "stopping"
        self._auto_reconnect = False
        for (name, process) in viewitems(self.processes):
            if process.poll() is not None:
                try:
                    process.kill()
                except:
                    pass


if __name__ == '__main__':
    rospy.logdebug("Starting Swarm")
    SWARM = SpheroSwarmNode()
    SWARM.spin()
    SWARM.stop()
