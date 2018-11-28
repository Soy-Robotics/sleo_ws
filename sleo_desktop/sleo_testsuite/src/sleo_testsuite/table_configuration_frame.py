# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Soy Robotics, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Soy Robotics, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Carl Zhang

import os

from python_qt_binding import loadUi
try:  # indigo
    from python_qt_binding.QtGui import QFrame, QVBoxLayout
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout, QTableWidgetItem
import QtCore     
import rospkg
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan
from plot_configuration_frame import PlotConfiguration
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


##############################################################################
# Classes
##############################################################################

class TableConfiguration(QFrame):

    def __init__(self, parent=None):

        QtCore.QMetaType.type("QVector<int>")
        super(TableConfiguration, self).__init__(parent)
        self._rospack = rospkg.RosPack()
        table_configuration_file = os.path.join(self._rospack.get_path('sleo_testsuite'),
                                               'resource/ui', 'table_configuration.ui')
        # loadUi(Table_configuration_file, self)
        loadUi(table_configuration_file, self)

        self.plot_config = PlotConfiguration()
        self.odom_subscriber = rospy.Subscriber(self.plot_config.get_odom_topic(), Odometry, self.odom_callback)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.cmd_subscriber = rospy.Subscriber(self.plot_config.get_cmd_topic(), Twist, self.cmd_callback)
        self.distance_subscriber = rospy.Subscriber('/moved_distance_topic', Float64, self.distance_callback)
        self.angle_subscriber = rospy.Subscriber('/moved_angle_topic', Float64, self.angle_callback)

        # Initialize
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0
        self.angular_speed = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.cmd_linear_x = 0
        self.cmd_angular_speed = 0

    def shutdown(self):
        '''
          Used to terminate the plugin 
        '''
        rospy.loginfo("Sleo TestSuite: table configuration frame shutdown")

    def stop(self):
        self.tableWidget.enable_timer(False)  # pause table rendering

    ###########################################################################
    # Qt callbacks
    ###########################################################################       
    def odom_callback(self, data):
        # get speed data
        linear = data.twist.twist.linear
        self.linear_x = str(round(linear.x,4))
        #get orientation speed data
        angular = data.twist.twist.angular
        self.angular_speed = str(round(angular.z,4))
        # get position data
        position = data.pose.pose.position
        self.position_x = str(round(position.x,4))
        self.position_y = str(round(position.y,4))
        # get orientation data
        # orientation = data.pose.pose.orientation
        # (r, p, y) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


        #print(self.orientation)
        self.label_9.setText(self.linear_x)
        self.label_12.setText(self.position_x)
        self.label_13.setText(self.position_y)
        self.label_15.setText(self.angular_speed)



    def cmd_callback(self, data):
        # get speed data
        linear = data.linear
        self.cmd_linear_x = str(round(linear.x,4))
        #get orientation speed data
        angular = data.angular
        self.cmd_angular_speed = str(round(angular.z,4))

        self.label_19.setText(self.cmd_linear_x)
        self.label_20.setText(self.cmd_angular_speed)
       
    def distance_callback(self, data):
        self.distance = str(round(data.data,4))
        self.label_14.setText(self.distance)

    def angle_callback(self, data):
        self.angle = str(round(data.data,4))
        self.label_21.setText(self.angle)

    def imu_callback(self, data):
        (r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.orientation = str(round(y,4))
        self.label_16.setText(self.orientation)