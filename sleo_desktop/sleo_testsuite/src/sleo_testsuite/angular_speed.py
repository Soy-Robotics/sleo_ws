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
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import copysign, sqrt, pow

class AngularSpeed(object):
    def __init__(self, cmd_vel_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.rate = rospy.Rate(50)

        self.ok = True
        self._stop = False
        self._running = False

        self._angular_speed = 0.5

    def init(self, angular_speed):
        self._angular_speed = angular_speed

    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        rospy.loginfo("Stopping the robot, Test finished...")
        self.cmd_vel_publisher.unregister()

    def change_topic(self, cmd_vel_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    def stop(self):
        self._stop = True

    def command(self, twist):
        if self._stop or rospy.is_shutdown():
            return False
        self.cmd_vel_publisher.publish(twist)
        self.rate.sleep()
        return True

    def go(self):
        twist = Twist()
        while self.ok:
            twist.angular.z = self._angular_speed
            if not self.command(twist):
                return False
        return True

    def execute_speed(self):
        if self._running:
            rospy.logerr("Sleo TestSuite: already executing linear error test, ignoring the request")
            return
        self._stop = False
        self._running = True
        while True:
            if not self.go() :
                break

        self._running = False
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)

