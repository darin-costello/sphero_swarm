#!/usr/bin/python

import rospy
import time
import numpy as np

from sphero_swarm.msg import Twist
from sphero_swarm.srv import ListSphero, ListSpheroRequest

STEP_LENGTH = 50


class SpheroSwarmRandomMove(object):

    def __init__(self):
        rospy.init_node('sphero_swarm_random_move', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.wait_for_service('list_spheros')
        list_spheros = rospy.ServiceProxy('list_spheros', ListSphero)
        self.spheros = list_spheros().name
        print "connected:"
        for name in self.spheros:
            print name

    def getTwist(self, name, move_id):

        twist = Twist()
        twist.name = name
        if move_id == 0:
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 1:
            twist.data.linear.x = 0
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 2:
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 3:
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = 0
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 4:
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = 0
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 5:
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 6:
            twist.data.linear.x = 0
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif move_id == 7:
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0

        return twist

    def randomMove(self):
        for name in self.spheros:
            move_id = np.random.randint(0, 8)
            twist = self.getTwist(name, move_id)
            self.cmdVelPub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            self.randomMove()
            time.sleep(3)


if __name__ == '__main__':

    w = SpheroSwarmRandomMove()
    w.run()
