#!/usr/bin/env python
#
#
##############################################################################
# Imports
##############################################################################
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
from python_qt_binding.QtCore import QObject, Signal, Slot, pyqtSlot, QTimer
try:  # indigo
    from python_qt_binding.QtGui import QFrame, QVBoxLayout
except ImportError:  # kinetic+ (pyqt5)
    from python_qt_binding.QtWidgets import QFrame, QVBoxLayout
import rospkg
import rospy
from qt_gui_py_common.worker_thread import WorkerThread
from linear_speed import LinearSpeed


##############################################################################
# Classes
##############################################################################

class LinearSpeedFrame(QFrame):
    
    def __init__(self, parent=None):
        super(LinearSpeedFrame, self).__init__(parent)
        self._cmd_vel_topic_name = '/sleo_velocity_controller/cmd_vel'
        self._rospack = rospkg.RosPack()
        linear_speed_file = os.path.join(self._rospack.get_path('sleo_testsuite'),
                               'resource/ui', 'linear_speed.ui')
        loadUi(linear_speed_file, self)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self._config = None
        self._table_config = None
        self._motion = None
        self._motion_thread = None

    def init(self,plot_config, table_config):
        self._config = plot_config
        self._table_config = table_config
        self._motion = LinearSpeed(self._config.get_cmd_topic())
        self._motion.init(self.speed_spinbox.value())

    ##########################################################################
    # Motion Callbacks
    ##########################################################################
    def _run_finished(self):
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def stop(self):
        self._motion.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    ###########################################################################
    # Qt callbacks
    ###########################################################################
    @Slot()
    def on_start_button_clicked(self):
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self._motion_thread = WorkerThread(self._motion.execute_speed, self._run_finished)
        self._motion_thread.start()
        #rospy.loginfo("current:"+self._config.get_cmd_topic())
        self._motion.change_topic(self._config.get_cmd_topic())
        self._config.run(False, True, False, False)
        #self._table_config.table()

    @Slot()
    def on_stop_button_clicked(self):
        self.stop()

    @pyqtSlot(float)
    def on_speed_spinbox_valueChanged(self, value):
        # could use value, but easy to set like this
        self._motion.init(self.speed_spinbox.value())

    def shutdown(self):
        '''
          Used to terminate the plugin
        '''
        rospy.loginfo("Sleo TestSuite: Linear test shutdown")
        self._motion.shutdown()

