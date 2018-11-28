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
import sys
import threading
import xmlrpclib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, QTimer, Signal
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from python_qt_binding.QtWidgets import QWidget
import rospkg
import rospy
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil
from rqt_topic.topic_widget import TopicWidget
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rqt_plot.data_plot import DataPlot
from linear_error_frame import LinearErrorFrame
from angular_error_frame import AngularErrorFrame

from plot_configuration_frame import PlotConfiguration
from table_configuration_frame import TableConfiguration

class SleoTestsuiteWidget(QWidget):


    def __init__(self, parent, plugin_context):
        """
        @type parent: TestsuiteMain
        """
        super(SleoTestsuiteWidget, self).__init__()
        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('sleo_testsuite'),
                               'resource/ui', 'testsuite_top.ui')
        #loadUi(ui_file, self, {'DataPlot': DataPlot})
        #loadUi(ui_file, self, {'TopicWidget': TopicWidget})
        #loadUi(ui_file, self, {'LinearErrorFrame', LinearErrorFrame})
        loadUi(ui_file, self, {'PlotConfiguration', PlotConfiguration})

        self.linear_error_frame.init(self.plot_configuration_frame, self.table_configuration_frame)
        self.linear_speed_frame.init(self.plot_configuration_frame, self.table_configuration_frame)
        self.angular_error_frame.init(self.plot_configuration_frame, self.table_configuration_frame)
        self.angular_speed_frame.init(self.plot_configuration_frame, self.table_configuration_frame)

        # Custom widget classes don't show in QSplitter when they instantiated
        # in .ui file and not explicitly added to QSplitter like this. Thus
        # this is a workaround.

    def shutdown(self):
        """
        Overridden.

        Close threads.

        @raise RuntimeError:
        """


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This moveites this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='sleo_testsuite'))
