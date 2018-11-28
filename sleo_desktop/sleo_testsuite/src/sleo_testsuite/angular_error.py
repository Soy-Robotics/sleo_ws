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
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import copysign, sqrt, pow, atan, pi, floor
from sensor_msgs.msg import Imu

class AngularError(object):
    def __init__(self, cmd_vel_topic, odom_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        # pulish the rotated angle
        self.moved_angle_publisher = rospy.Publisher('/moved_angle_topic', Float64, queue_size=10)
        # self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)


        self.rate = rospy.Rate(50)

        self.ok = True
        self._stop = False
        self._running = False

        self._angular_speed = 0.5
        #***************************
        self._angular = 6

        self.angular = 0.0

        self.angular_start = self.angular

        self.start_test = True
        #*****************************
        self.count = 0
        self.flag = True
        self.count_init = 0

    def change_topic(self, cmd_vel_topic, odom_topic):
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        # self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.moved_angle_publisher = rospy.Publisher('/moved_angle_topic', Float64, queue_size=10)
        # Update Initial Position
        self.angular_start = self.angular
        if self.angular_start < 0:
            self.count_init = 1
        else:
            self.count_init = 0
        self.start_test = True
        self.count = 0
        self.flag = True

    def init(self, angular_speed, angular, tolerance):
        self._angular_speed = angular_speed
        self._angular = angular
        self._tolerance = tolerance
        self.count = 0
        self.flag = True
        if self.angular_start < 0:
            self.count_init = 1
        else:
            self.count_init = 0   

    def shutdown(self):
        self.stop()
        while self._running:
            self.rate.sleep()
        rospy.loginfo("Stopping the robot, Test finished...")
        self.cmd_vel_publisher.unregister()
        self.odom_subscriber.unregister()
        self.moved_angle_publisher.unregister()

    def stop(self):
        self._stop = True

    def command(self, twist):
        if self._stop or rospy.is_shutdown() or not self.start_test:
            return False
        self.cmd_vel_publisher.publish(twist)
        self.rate.sleep()
        if self.rotate_angular < 0:
            self.rotate_angular = 0
        self.moved_angle_publisher.publish(self.rotate_angular)
        return True

    def go(self):
        twist = Twist()
        while self.ok:
            if self.start_test:
                twist.angular.z = self._angular_speed
                print ("start")
                print(self.flag)
                print(self.angular_start)

                print(self.angular)
                print(self.count)
                print(self.count_init)
                self.rotate_angular =  self.angular - self.angular_start + pi * (self.count - self.count_init) 
                # print(self.angular)
                # print(self.count)
                # print(self.count_init)
                self.rotate_angular =  self.angular - self.angular_start + 2 * pi * (self.count - self.count_init) 

                #rospy.loginfo("current start_x: %f, start_y: %f" , self.position_start_x , self.position_start_y )
                #rospy.loginfo("current pos_x: %f, pos_y: %f",self.position_x,self.position_y)
                #How close are we?
                error = self.rotate_angular - self._angular

                # Are we close enough?
                if not self.start_test or abs(error) < self._tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    #rospy.loginfo(params)
                    self.angular_start = self.angular
                else:
                    # If not, move in the appropriate direction
                    twist.angular.z = copysign(self._angular_speed, -1 * error)
            else:
                self.angular_start = self.angular
            if not self.command(twist) :
                return  False
        return True


    def execute(self):
        if self._running:
            rospy.logerr("Sleo TestSuite: already executing anguar error test, ignoring the request")
            return
        self._stop = False
        self._running = True
        while True:
            if not self.go() :
                break

        self._running = False
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)

    ##########################################################################
    # Callbacks
    ##########################################################################

    # def odom_callback(self, data):
    #     angular = data.pose.pose.orientation
    #     (r, p, y) = tf.transformations.euler_from_quaternion([angular.x, angular.y, angular.z, angular.w])


    def imu_callback(self, data):
        (r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.angular = y
        #self.angular = 2*atan((2 * angular.w * angular.z) /(1 - 2 * angular.z * angular.z))
        # ensure input angle can be larger than 1.57
        if self.angular < 0 and self.flag == True:
            self.count = self.count + 1
            self.flag = False
        if self.angular > 0:
            self.flag = True
