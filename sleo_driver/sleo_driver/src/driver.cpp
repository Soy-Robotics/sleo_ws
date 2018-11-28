/**
Software License Agreement (BSD)

\file      driver.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike Irvine <mirvine@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "sleo_driver/controller.h"
#include "sleo_driver/sleo_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"
#include <boost/chrono.hpp>


#include "ros/ros.h"
typedef boost::chrono::steady_clock time_source;
/**
* Control loop for Sleo, not realtime safe
*/
void controlLoop(sleo::SleoHardware &sleo,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  sleo.controller_->spinOnce();        
  sleo.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  sleo.writeCommandsToHardware();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  double control_frequency,diagnostic_frequency;
  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  private_nh.param<std::string>("port", port, port);
  private_nh.param<int32_t>("baud", baud, baud);
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Interface to motor controller->
  sleo::SleoHardware sleo(nh, private_nh);
  controller_manager::ControllerManager cm(&sleo, nh);

  ros::CallbackQueue sleo_queue;
  ros::AsyncSpinner sleo_spinner(1, &sleo_queue);
  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
  ros::Duration(1 / control_frequency),
  boost::bind(controlLoop, boost::ref(sleo), boost::ref(cm), boost::ref(last_time)),
  &sleo_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);


  sleo_spinner.start();
  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();
  return 0;
}
