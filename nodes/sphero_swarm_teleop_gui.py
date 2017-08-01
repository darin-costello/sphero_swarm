#!/usr/bin/python

import sys
import rospy
import math
from PyQt4 import QtGui, QtCore

from sphero_swarm.msg import Twist
from sphero_swarm.srv import ListSphero

STEP_LENGTH = 50


class SpheroSwarmTeleopForm(QtGui.QWidget):

    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480)
        self.spheros = {}
        self.initUI()

        rospy.init_node('sphero_swarm_teleop_gui', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmdVelCallback)
        rospy.wait_for_service('list_spheros')
        self.list_spheros = rospy.ServiceProxy('list_spheros', ListSphero)

    def initUI(self):

        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL(
            "sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.spheroLabel = QtGui.QLabel("Spheros:")
        self.spheroListWidget = QtGui.QListWidget()
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.refreshDevices)
        btnGridLayout = QtGui.QGridLayout()
        btnGridLayout.addWidget(self.refreshBtn, 0, 4)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.keyInstructLabel)
        layout.addWidget(self.cmdVelLabel)
        layout.addWidget(self.cmdVelTextbox)
        layout.addWidget(self.spheroLabel)
        layout.addWidget(self.spheroListWidget)
        layout.addLayout(btnGridLayout)
        self.setLayout(layout)

        self.setWindowTitle("Sphero Swarm Teleop")
        self.show()

    def keyPressEvent(self, e):
        twist = None

        print "key pressed"
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) == 0:
            return

        print "selected"

        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()
            twist.data.linear.x = 0
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = 0
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.data.linear.x = 0
            twist.data.linear.y = 0
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = 0
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.data.linear.x = -STEP_LENGTH
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.data.linear.x = 0
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.data.linear.x = STEP_LENGTH
            twist.data.linear.y = -STEP_LENGTH
            twist.data.linear.z = 0
            twist.data.angular.x = 0
            twist.data.angular.y = 0
            twist.data.angular.z = 0

        if twist != None:
            twist.name = str(selected_items[0].text())
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "(" + str(msg.name) + "),x=" + \
            str(msg.data.linear.x) + " y=" + str(msg.data.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text)

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def refreshDevices(self):
        self.spheroListWidget.clear()
        self.spheros = self.list_spheros().name

        print(self.spheros)

        for name in self.spheros:
            self.spheroListWidget.addItem(name)
        self.update()


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroSwarmTeleopForm()
    w.show()
    sys.exit(app.exec_())
