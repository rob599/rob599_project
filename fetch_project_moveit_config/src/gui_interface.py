#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt


class Interface(QMainWindow):

    def __init__(self):
        super(Interface,self).__init__()
        self.gui_input_pub = rospy.Publisher('gui_input', String, queue_size = 10)
        self.setWindowTitle('Fetch Disinfectant Project')
        self.setGeometry(300, 300, 250, 50)
        self.initUI()


    def initUI(self):
        global app
        self.quit_button = QPushButton('Quit')
        self.planning   = QPushButton('Plan Path')
        self.execute     = QPushButton('Execute Path')
        self.init_pose   = QPushButton('Initial Pose')
        self.tuck_pose   = QPushButton('Tuck Arm')

        # A widget to hold everything
        widget = QWidget()
        self.setCentralWidget(widget)

        # A layout
        layout = QHBoxLayout()
        layout.addWidget(self.planning)
        layout.addWidget(self.execute)
        layout.addWidget(self.init_pose)
        layout.addWidget(self.tuck_pose)
        layout.addWidget(self.quit_button)
        widget.setLayout(layout)

        self.quit_button.clicked.connect(app.exit)
        self.planning.clicked.connect(self.publish_command)
        self.execute.clicked.connect(self.publish_command_b)
        self.init_pose.clicked.connect(self.publish_command_c)
        self.tuck_pose.clicked.connect(self.publish_command_d)
        self.show()

    def publish_command(self):
        self.gui_input_pub.publish("0")
        print(0)
    def publish_command_b(self):
        self.gui_input_pub.publish("1")
        print(1)
    def publish_command_c(self):
        self.gui_input_pub.publish("2")
        print(2)
    def publish_command_d(self):
        self.gui_input_pub.publish("3")
        print(3)

def run():
    rospy.init_node('gui_interface')
    global app
    app = QApplication(sys.argv)
    interface = Interface()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
