#!/usr/bin/python

import sys
import rospy
import math
import bluetooth
import time
from PyQt4 import QtGui, QtCore
from std_msgs.msg import ColorRGBA
from sphero_swarm.msg import BackLed, Color, Twist, Turn, Heading, DisableStabilization
from sphero_swarm.srv import SpheroInfo, SpheroInfoRequest, SpheroInfoResponse
from sphero_swarm.srv import ListSphero


class SpheroListItem(QtGui.QListWidgetItem):

    def __init__(self, name):
        super(QtGui.QListWidgetItem, self).__init__()
        self.name = name
        self.setText(str(self))

    def __repr__(self):
        return str(self.name)


class SpheroSwarmManagerWidget(QtGui.QWidget):

    def __init__(self, parentWindow):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parentWindow

        rospy.init_node('sphero_swarm_manager', anonymous=True)
        self.cmdTurnPub = rospy.Publisher('cmd_turn', Turn, queue_size=1)
        self.ledPub = rospy.Publisher('set_color', Color, queue_size=1)
        self.backLedPub = rospy.Publisher(
            'set_back_led', BackLed, queue_size=1)
        self.headingPub = rospy.Publisher(
            'set_heading', Heading, queue_size=1)
        self.disableStabilizationPub = rospy.Publisher(
            'disable_stabilization', DisableStabilization, queue_size=1)

        self.nameLabel = QtGui.QLabel("Name")
        self.nameLineEdit = QtGui.QLineEdit()
        self.btaddrLabel = QtGui.QLabel("Bluetooth Address")
        self.btaddrLineEdit = QtGui.QLineEdit()
        self.connectBtn = QtGui.QPushButton("Connect")
        self.connectBtn.clicked.connect(self.connectSphero)
        self.clearBtn = QtGui.QPushButton("Clear")
        self.clearBtn.clicked.connect(self.clearAddText)

        self.spheroLabel = QtGui.QLabel("Sphero List")
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.updateList)
        self.spheroListWidget = QtGui.QListWidget()

        self.disconnectBtn = QtGui.QPushButton("Disconnect")
        self.disconnectBtn.clicked.connect(self.disconnectSphero)
        self.disconnectAllBtn = QtGui.QPushButton("Disconnect All")
        self.disconnectAllBtn.clicked.connect(self.disconnectAllSpheros)
        self.testBtn = QtGui.QPushButton("Test")
        self.testBtn.clicked.connect(self.testSphero)
        self.testAllBtn = QtGui.QPushButton("Test All")
        self.testAllBtn.clicked.connect(self.testAllSpheros)

        layout = QtGui.QVBoxLayout()
        addLayout = QtGui.QHBoxLayout()
        addLayout.addWidget(self.nameLabel)
        addLayout.addWidget(self.nameLineEdit)
        addLayout.addWidget(self.btaddrLabel)
        addLayout.addWidget(self.btaddrLineEdit)
        addLayout.addWidget(self.connectBtn)
        layout.addLayout(addLayout)
        spheroLayout = QtGui.QGridLayout()
        spheroLayout.addWidget(self.spheroLabel, 0, 0)
        spheroLayout.addWidget(self.refreshBtn, 0, 6)
        layout.addLayout(spheroLayout)
        layout.addWidget(self.spheroListWidget)
        btnLayout = QtGui.QHBoxLayout()
        btnLayout.addWidget(self.testBtn)
        btnLayout.addWidget(self.testAllBtn)
        btnLayout.addWidget(self.disconnectBtn)
        btnLayout.addWidget(self.disconnectAllBtn)
        layout.addLayout(btnLayout)
        self.setLayout(layout)

    def updateList(self):
        self.spheroListWidget.clear()
        self.parentWindow.updateSpheroSwarm()

        print(self.parentWindow.sphero_dict)

        for name in self.parentWindow.sphero_dict:
            self.spheroListWidget.addItem(SpheroListItem(name))
        self.spheroListWidget.update()

    def connectSphero(self):
        if self.nameLineEdit.text().isEmpty():
            QtGui.QMessageBox.critical(self, "Message", "Name is null.")
            return
        if self.btaddrLineEdit.text().isEmpty():
            QtGui.QMessageBox.critical(
                self, "Message", "Bluetooth address is null.")
            return
        self.parentWindow.connectSphero(
            str(self.nameLineEdit.text()), str(self.btaddrLineEdit.text()))
        self.updateList()

    def clearAddText(self):
        self.nameLineEdit.setText("")
        self.nameLineEdit.setText("")
        self.update()

    def disconnectSphero(self):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                print "disconnect " + str(item.name)
                self.parentWindow.disconnectSphero(item.name)
            self.updateList()

    def disconnectAllSpheros(self):
        for i in range(self.spheroListWidget.count()):
            item = self.spheroListWidget.item(i)
            print "disconnect " + str(item.name)
            self.parentWindow.disconnectSphero(item.name)
        self.updateList()

    def testSphero(self):
        # print "test Sphero"
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                sphero_name = item.name
                print "TESTING " + str(sphero_name)
                print "disable stabilization"
                stab_data = DisableStabilization(sphero_name, True)
                self.disableStabilizationPub.publish(stab_data)
                time.sleep(1)
                print "set color to RED"
                color = Color(sphero_name, ColorRGBA(255, 0, 0, 255))
                self.ledPub.publish(color)
                time.sleep(1)
                print "set color to GREEN"
                color = Color(sphero_name, ColorRGBA(0, 255, 0, 255))
                self.ledPub.publish(color)
                time.sleep(1)
                print "set color to BLUE"
                color = Color(sphero_name, ColorRGBA(0, 255, 0, 255))
                self.ledPub.publish(color)
                time.sleep(1)
                print "set back led"
                color = Color(sphero_name, ColorRGBA(255, 255, 255, 255))
                self.ledPub.publish(color)
                light = BackLed(sphero_name, 255)
                self.backLedPub.publish(light)
                time.sleep(1)
                print "enable stablization"
                stab_data = DisableStabilization(sphero_name, False)
                self.disableStabilizationPub.publish(stab_data)
                time.sleep(1)
                print "set aiming"
                turning = Turn(sphero_name, 90)
                self.cmdTurnPub.publish(turning)
                heading = Heading(sphero_name, 0.0)
                self.headingPub.publish(heading)

    def testAllSpheros(self):
        print "TESTING ALL"
        print "disable stabilization"
        for sphero_name in self.parentWindow.sphero_dict:
            stab_data = DisableStabilization(sphero_name, True)
            self.disableStabilizationPub.publish(stab_data)
        time.sleep(1)
        print "set color to RED"
        for sphero_name in self.parentWindow.sphero_dict:
            color = Color(sphero_name, ColorRGBA(255, 0, 0, 255))
            self.ledPub.publish(color)
        time.sleep(1)
        print "set color to GREEN"
        for sphero_name in self.parentWindow.sphero_dict:
            color = Color(sphero_name, ColorRGBA(0, 255, 0, 255))
            self.ledPub.publish(color)
        time.sleep(1)
        print "set color to BLUE"
        for sphero_name in self.parentWindow.sphero_dict:
            color = Color(sphero_name, ColorRGBA(0, 0, 255, 255))
            self.ledPub.publish(color)
        time.sleep(1)
        print "set back led"
        for sphero_name in self.parentWindow.sphero_dict:
            color = Color(sphero_name, ColorRGBA(255, 255, 255, 255))
            self.ledPub.publish(color)
            light = BackLed(sphero_name, 255)
            self.backLedPub.publish(light)
        time.sleep(1)
        print "enable stablization"
        for sphero_name in self.parentWindow.sphero_dict:
            stab_data = DisableStabilization(sphero_name, False)
            self.disableStabilizationPub.publish(stab_data)
        time.sleep(1)
        print "set aiming"
        for sphero_name in self.parentWindow.sphero_dict:
            turning = Turn(sphero_name, 90)
            self.cmdTurnPub.publish(turning)
            heading = Heading(sphero_name, 0.0)
            self.headingPub.publish(heading)


class SpheroSwarmManagerForm(QtGui.QMainWindow):

    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(600, 400)
        self.spheros = []
        rospy.wait_for_service('list_spheros')
        self.list_spheros = rospy.ServiceProxy('list_spheros', ListSphero)
        self.initUI()

    def initUI(self):

        self.spheroMgr = SpheroSwarmManagerWidget(self)
        self.setCentralWidget(self.spheroMgr)

        self.setWindowTitle('Sphero Swarm')
        self.show()

    def updateSpheroSwarm(self):
        self.sphero_dict = self.list_spheros().name

    def connectSphero(self, name, btaddr):
        rospy.wait_for_service('add_sphero')
        try:
            add_sphero = rospy.ServiceProxy('add_sphero', SpheroInfo)
            resp1 = add_sphero(name, btaddr)
            if resp1.success > 0:
                print "%s has been added" % name
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        return False

    def disconnectSphero(self, name):
        rospy.wait_for_service('remove_sphero')
        try:
            remove_sphero = rospy.ServiceProxy('remove_sphero', SpheroInfo)
            resp1 = remove_sphero(name, "nothing")
            if resp1.success > 0:
                print "{} has been removed".format(name)
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        return False


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroSwarmManagerForm()
    w.show()
    sys.exit(app.exec_())
