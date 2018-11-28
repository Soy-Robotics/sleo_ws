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
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout

import rospkg
import rospy

from rqt_plot.data_plot import DataPlot
from rqt_plot.plot_widget import PlotWidget


##############################################################################
# Classes
##############################################################################

class PlotConfiguration(QFrame):
    def __init__(self, parent=None):
        super(PlotConfiguration, self).__init__(parent)
        self._rospack = rospkg.RosPack()
        plot_configuration_file = os.path.join(self._rospack.get_path('sleo_testsuite'),
                                               'resource/ui', 'plot_configuration.ui')
        # loadUi(plot_configuration_file, self)
        loadUi(plot_configuration_file, self, {'DataPlot': DataPlot})
        self._plot_widget = PlotWidget()
        self._plot_widget.setWindowTitle("Plot Profile")
        self._plot_widget.topic_edit.setText("/sleo_velocity_controller/cmd_vel")
        self.horizontalLayout_4.addWidget(self._plot_widget)

        self._plot_widget.switch_data_plot_widget(self._widget_data_plot)

        self._old_cmd_topic = self._cmd_topic_comboBox.currentText()
        self._old_odom_topic = self._odom_topic_comboBox.currentText()

        # Get current topic list and fileter contains topics we will use
        self._topics_and_types = rospy.get_published_topics()
        for topic, type in self._topics_and_types:
            #print 'topic: '+ topic + ' type: ' + type + '\n'
            if type == 'geometry_msgs/Twist':
                self._cmd_topic_comboBox.addItem(topic)
            if type == 'nav_msgs/Odometry':
                self._odom_topic_comboBox.addItem(topic)


    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Sleo TestSuite: Plot configuration frame shutdown")

    def get_cmd_topic(self):
        if self._cmd_topic_comboBox.currentText() =='':
            rospy.logerr("Sleo Testsuite: current command topic is empty! Please checkout whether topic is open or not.")
        return self._cmd_topic_comboBox.currentText()

    def get_odom_topic(self):
        if self._odom_topic_comboBox.currentText() =='':
            rospy.logerr("Sleo Testsuite: current odom topic is empty! Please checkout whether topic is open or not.")
        return self._odom_topic_comboBox.currentText()

    def run(self, line_error_bool, Line_speed_error_bool, Angl_error_bool, Angl_speed_error_bool):

        self._plot_widget.clean_up_subscribers()

        if line_error_bool:
            ## whether can change to odom_linear_x_topic
            odom_linear_x_topic = '/moved_distance_topic'

            #self._plot_widget.topic_edit.setText(cmd_linear_x_topic)
            #self._plot_widget.topic_edit.setText(odom_speed_linear_x_topic)
            self._plot_widget._start_time = rospy.get_time()
            self._plot_widget.enable_timer(True)

            try:
                self._plot_widget.remove_topic(self._cmd_topic_comboBox.currentText())
            except KeyError:
                pass

            self._plot_widget.add_topic(odom_linear_x_topic)
            

        elif Line_speed_error_bool:
            cmd_linear_x_topic = self._cmd_topic_comboBox.currentText()+'/linear/x'
            ## whether can change to odom_linear_x_topic
            odom_speed_linear_x_topic = self._odom_topic_comboBox.currentText()+'/twist/twist/linear/x'

            self._plot_widget.topic_edit.setText(cmd_linear_x_topic)
            self._plot_widget.topic_edit.setText(odom_speed_linear_x_topic)
            self._plot_widget._start_time = rospy.get_time()
            self._plot_widget.enable_timer(True)

            try:
                self._plot_widget.remove_topic(self._cmd_topic_comboBox.currentText())
            except KeyError:
                pass

            self._plot_widget.add_topic(cmd_linear_x_topic)
            self._plot_widget.add_topic(odom_speed_linear_x_topic)


        elif Angl_error_bool:
            cmd_angular_z_topic = '/moved_angle_topic'

            #   self._plot_widget.topic_edit.setText(cmd_angular_z_topic)
            #   self._plot_widget.topic_edit.setText(odom_speed_linear_x_topic)
            self._plot_widget._start_time = rospy.get_time()
            self._plot_widget.enable_timer(True)

            try:
                self._plot_widget.remove_topic(self._cmd_topic_comboBox.currentText())
            except KeyError:
                pass

            self._plot_widget.add_topic(cmd_angular_z_topic)

        elif Angl_speed_error_bool:
            cmd_angular_z_topic = self._cmd_topic_comboBox.currentText()+'/angular/z'
            odom_angular_speed_z_topic = self._odom_topic_comboBox.currentText()+'/twist/twist/angular/z'

            #   self._plot_widget.topic_edit.setText(cmd_angular_z_topic)
            #   self._plot_widget.topic_edit.setText(odom_speed_linear_x_topic)
            self._plot_widget._start_time = rospy.get_time()
            self._plot_widget.enable_timer(True)

            try:
                self._plot_widget.remove_topic(self._cmd_topic_comboBox.currentText())
            except KeyError:
                pass

            self._plot_widget.add_topic(cmd_angular_z_topic)
            self._plot_widget.add_topic(odom_angular_speed_z_topic)


    def stop(self):
        self._plot_widget.enable_timer(False)  # pause plot rendering
