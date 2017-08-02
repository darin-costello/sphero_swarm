#!/usr/bin/python

import sys
import rospy
import math
from PyQt4 import QtGui, QtCore
#from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool

from sphero_swarm.msg import BackLed, Color, Twist, Turn, Heading, DisableStabilization
from sphero_swarm.srv import ListSphero


class SpheroListItem(QtGui.QListWidgetItem):

    def __init__(self, name, rgb, back_led, disable_stabilization):
        super(QtGui.QListWidgetItem, self).__init__()
        self.name = name
        self.rgb = rgb
        self.back_led = back_led
        self.disable_stabilization = disable_stabilization
        self.setText(str(self))

    def __repr__(self):
        return str(self.name)


class LEDWidget(QtGui.QLabel):

    def __init__(self, size, rgb):
        super(QtGui.QLabel, self).__init__()
        pixmap = QtGui.QPixmap(size[0], size[1])
        self.setPixmap(pixmap)
        self.rgb = rgb
        self.setAutoFillBackground(True)
        self.setBackgroundRole(QtGui.QPalette.Base)

    def setRGB(self, r, g, b):
        self.rgb = [r, g, b]
        p = QtGui.QPalette()
        p.setColor(self.backgroundRole(), QtGui.QColor(
            self.rgb[0], self.rgb[1], self.rgb[2]))
        self.setPalette(p)
        self.update()

    def paintEvent(self, e):
        super(QtGui.QLabel, self).paintEvent(e)
        #qp = QtGui.QPainter()
        # qp.begin(self)
        # qp.end()


class DashboardWidget(QtGui.QWidget):

    def __init__(self, parent):
        super(QtGui.QWidget, self).__init__()
        self.parentWindow = parent

        self.initUI()

    def initUI(self):
        self.bk_label = QtGui.QLabel("back LED")
        self.bk_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.bk_sl.setMinimum(0)
        self.bk_sl.setMaximum(255)
        self.bk_sl.setValue(self.parentWindow.ledBack)
        self.bk_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.bk_sl.setTickInterval(1)

        self.r_label = QtGui.QLabel("R")
        self.r_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.r_sl.setMinimum(0)
        self.r_sl.setMaximum(255)
        self.r_sl.setValue(self.parentWindow.ledR)
        self.r_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.r_sl.setTickInterval(1)
        self.r_sl.valueChanged.connect(self.valuechange)

        self.g_label = QtGui.QLabel("G")
        self.g_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.g_sl.setMinimum(0)
        self.g_sl.setMaximum(255)
        self.g_sl.setValue(self.parentWindow.ledG)
        self.g_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.g_sl.setTickInterval(1)
        self.g_sl.valueChanged.connect(self.valuechange)

        self.b_label = QtGui.QLabel("B")
        self.b_sl = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.b_sl.setMinimum(0)
        self.b_sl.setMaximum(255)
        self.b_sl.setValue(self.parentWindow.ledB)
        self.b_sl.setTickPosition(QtGui.QSlider.TicksBelow)
        self.b_sl.setTickInterval(1)
        self.b_sl.valueChanged.connect(self.valuechange)

        self.led = LEDWidget([20, 20],
                             [self.parentWindow.ledR,
                              self.parentWindow.ledG,
                              self.parentWindow.ledB])

        self.btnUpdate = QtGui.QPushButton("Update")
        self.btnUpdate.clicked.connect(self.updateColorCallback)

        self.disableStabilizationRadioButton = QtGui.QRadioButton(
            "Disable Stabilization")
        self.disableStabilizationRadioButton.setChecked(False)
        self.disableStabilizationRadioButton.toggled.connect(
            self.handleDisableStabilizationCheck)

        self.degLabel = QtGui.QLabel("Degree:")
        self.degTextbox = QtGui.QLineEdit()
        self.degTextbox.setText("15")
        self.degTextbox.setFixedWidth(40)
        self.leftBtn = QtGui.QPushButton("Counter Clockwise")
        self.leftBtn.clicked.connect(self.leftRotate)
        self.rightBtn = QtGui.QPushButton("Clockwise")
        self.rightBtn.clicked.connect(self.rightRotate)

        self.selectedSpheroLabel = QtGui.QLabel("Selected = ")
        self.spheroListWidget = QtGui.QListWidget(self)
        self.spheroListWidget.itemClicked.connect(self.handleListClicked)
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.parentWindow.refreshDevices)
        btnGridLayout = QtGui.QGridLayout()
        btnGridLayout.addWidget(self.refreshBtn, 0, 4)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.led)
        r_layout = QtGui.QHBoxLayout()
        r_layout.addWidget(self.r_label)
        r_layout.addWidget(self.r_sl)
        layout.addLayout(r_layout)
        g_layout = QtGui.QHBoxLayout()
        g_layout.addWidget(self.g_label)
        g_layout.addWidget(self.g_sl)
        layout.addLayout(g_layout)
        b_layout = QtGui.QHBoxLayout()
        b_layout.addWidget(self.b_label)
        b_layout.addWidget(self.b_sl)
        layout.addLayout(b_layout)
        bk_layout = QtGui.QHBoxLayout()
        bk_layout.addWidget(self.bk_label)
        bk_layout.addWidget(self.bk_sl)
        layout.addLayout(bk_layout)
        btn_layout = QtGui.QHBoxLayout()
        btn_layout.addWidget(self.btnUpdate)
        layout.addLayout(btn_layout)

        layout.addWidget(self.disableStabilizationRadioButton)
        ctrlLayout = QtGui.QHBoxLayout()
        ctrlLayout.addWidget(self.degLabel)
        ctrlLayout.addWidget(self.degTextbox)
        ctrlLayout.addWidget(self.leftBtn)
        ctrlLayout.addWidget(self.rightBtn)
        layout.addLayout(ctrlLayout)

        layout.addWidget(self.selectedSpheroLabel)
        layout.addWidget(self.spheroListWidget)
        layout.addLayout(btnGridLayout)
        self.setLayout(layout)
        self.show()

    def handleListClicked(self, item):
        # print "clicked " + str(item.name)

        r = item.rgb[0]
        g = item.rgb[1]
        b = item.rgb[2]
        bk = item.back_led
        disable = item.disable_stabilization
        self.bk_sl.setValue(bk)
        self.r_sl.setValue(r)
        self.g_sl.setValue(g)
        self.b_sl.setValue(b)
        self.disableStabilizationRadioButton.setChecked(disable)
        self.selectedSpheroLabel.setText("Selected = " + str(item.name))
        self.spheroListWidget.update()
        self.update()

    def updateColorCallback(self):
        r = self.r_sl.value()
        g = self.g_sl.value()
        b = self.b_sl.value()
        back = self.bk_sl.value()
        self.updateWidgetColor(r, g, b)
        self.updateColor(r, g, b, back)
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                item.rgb[0] = r
                item.rgb[1] = g
                item.rgb[2] = b
                item.back_led = back

    def updateWidgetColor(self, r, g, b):
        self.led.setRGB(r, g, b)
        self.update()

    def updateColor(self, r, g, b, back):
        self.setLEDColor(r, g, b)
        self.setBackLED(back)

    def valuechange(self):
        self.led.setRGB(self.r_sl.value(),
                        self.g_sl.value(),
                        self.b_sl.value())
        self.led.update()

    def updateHeading(self):
        val = self.headingSlider.value()
        self.parentWindow.setHeading(val)
        # self.headingSlider.setValue(180)
        self.update()

    def leftRotate(self):
        deg = int(self.degTextbox.displayText())
        self.setHeading(360 - deg)

    def rightRotate(self):
        deg = int(self.degTextbox.displayText())
        self.setHeading(15)

    def headingChange(self, int):
        delta_val = self.headingSlider.value() - self.currentHeadingSliderValue
        self.currentHeadingSliderValue = self.headingSlider.value()

    def handleDisableStabilizationCheck(self):
        if self.disableStabilizationRadioButton.isChecked() == True:
            self.setDisableStabilization(True)
        else:
            self.setDisableStabilization(False)

    def setLEDColor(self, r, g, b):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setLEDColorByName(item.name, r, g, b)

    def setBackLED(self, val):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setBackLEDByName(item.name, val)

    def setHeading(self, val):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                self.parentWindow.setHeadingByName(item.name, val)

    def setDisableStabilization(self, on):
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) > 0:
            for item in selected_items:
                item.disable_stabilization = on
                self.parentWindow.setDisableStabilizationByName(item.name, on)


class SpheroDashboardForm(QtGui.QMainWindow):

    def __init__(self):
        super(QtGui.QMainWindow, self).__init__()
        self.resize(400, 600)

        self.spheros = []
        self.sphero_info = {}
        rospy.init_node('sphero_swarm_dashboard', anonymous=True)

        self.cmdTurnPub = rospy.Publisher('cmd_turn', Turn, queue_size=1)
        self.ledPub = rospy.Publisher('set_color', Color, queue_size=1)
        self.backLedPub = rospy.Publisher(
            'set_back_led', BackLed, queue_size=1)
        self.headingPub = rospy.Publisher(
            'set_heading', Heading, queue_size=1)
        self.disableStabilizationPub = rospy.Publisher(
            'disable_stabilization', DisableStabilization, queue_size=1)
        self.ledR = 0
        self.ledG = 0
        self.ledB = 0
        self.ledBack = 0

        self.initUI()

    def initUI(self):
        self.dashboard = DashboardWidget(self)
        self.setCentralWidget(self.dashboard)
        self.setWindowTitle("Sphero Swarm dashboard")
        self.show()

    def refreshDevices(self):
        rospy.wait_for_service('list_spheros')
        list_spheros = rospy.ServiceProxy('list_spheros', ListSphero)
        self.dashboard.spheroListWidget.clear()
        self.spheros = list_spheros().name

        print(self.spheros)

        for name in self.spheros:
            self.sphero_info[name] = [
                [self.ledR, self.ledG, self.ledB], self.ledBack, False]
            self.dashboard.spheroListWidget.addItem(SpheroListItem(
                name, [self.ledR, self.ledG, self.ledB], self.ledBack, False))
        self.update()

    def setLEDColorByName(self, name, r, g, b):
        color = Color(name, ColorRGBA(r, g, b, 255))
        self.ledPub.publish(color)

    def setBackLEDByName(self, name, val):
        light = BackLed(name, val)
        self.backLedPub.publish(light)

    def setHeadingByName(self, name, val):
        turning = Turn(name, val)
        self.cmdTurnPub.publish(turning)

        heading = Heading(name, 0.0)
        self.headingPub.publish(heading)

    def setDisableStabilizationByName(self, name, on):
        stab_data = DisableStabilization(name, on)
        self.disableStabilizationPub.publish(stab_data)


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroDashboardForm()
    w.show()
    sys.exit(app.exec_())
