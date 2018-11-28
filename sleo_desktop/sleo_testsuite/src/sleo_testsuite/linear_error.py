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
from std_msgs.msg import Float64
from math import copysign, sqrt, pow

class LinearError(object):
    def __init__(self, cmd_vel_topic, odom_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        # publish the moved distance
        self.moved_distance_publisher = rospy.Publisher('/moved_distance_topic', Float64, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.rate = rospy.Rate(50)

        self.ok = True
        self._stop = False
        self._running = False

        self._linear_speed = 0.2
        self._distence = 1.0
        self.distance = 0

        self.position_x = 0.0
        self.position_y = 0.0

        # Update Initial Position
        self.position_start_x = 0
        self.position_start_y = 0

        self.start_test = True

    def change_topic(self, cmd_vel_topic, odom_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.moved_distance_publisher = rospy.Publisher('/moved_distance_topic', Float64, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.position_start_x = self.position_x
        self.position_start_y = self.position_y
        self.start_test = True

    def init(self, linear_speed, distence, tolerance):
        self._linear_speed = linear_speed
        self._distence = distence
        self._tolerance = tolerance

    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        rospy.loginfo("Stopping the robot, Test finished...")
        self.cmd_vel_publisher.unregister()
        self.moved_distance_publisher.unregister()
        self.odom_subscriber.unregister()

    def stop(self):
        self._stop = True

    def command(self, twist):
        if self._stop or rospy.is_shutdown() or not self.start_test:
            return False
        self.cmd_vel_publisher.publish(twist)
        self.moved_distance_publisher.publish(self.distance)
        self.rate.sleep()
        return True

    def go(self):
        twist = Twist()
        while self.ok:
            if self.start_test:

                twist.linear.x = self._linear_speed

                # Compute the Euclidean distance from the target point
                self.distance = sqrt(pow((self.position_x - self.position_start_x), 2) +
                            pow((self.position_y - self.position_start_y), 2))
                #rospy.loginfo("current start_x: %f, start_y: %f" , self.position_start_x , self.position_start_y )
                #rospy.loginfo("current pos_x: %f, pos_y: %f",self.position_x,self.position_y)
                # How close are we?
                error = self.distance - self._distence

                # Are we close enough?
                if not self.start_test or abs(error) < self._tolerance:
                    self.start_test = False
                    # params = {'start_test': False}
                    #rospy.loginfo(params)
                    self.position_start_x = self.position_x
                    self.position_start_y = self.position_y
                else:
                    # If not, move in the appropriate direction
                    twist.linear.x = copysign(self._linear_speed, -1 * error)
            else:
                self.position_start_x = self.position_x
                self.position_start_y = self.position_y
            if not self.command(twist) :
                return  False
        return True


    def execute(self):
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

    ##########################################################################
    # Callbacks
    ##########################################################################

    def odom_callback(self, data):
        position = data.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y
