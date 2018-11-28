/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*  \copyright  Copyright (c) 2018-2025, Soy Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "sleo_driver/sleo_hardware.h"
#include <boost/assign/list_of.hpp>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace sleo
{

  /**
  * Initialize Sleo hardware
  */
  SleoHardware::SleoHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :
    nh_(nh),
    private_nh_(private_nh),
    prev_encoder_l(0),prev_encoder_r(0)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3302);
    private_nh_.param<double>("max_speed", max_speed_, 2.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/ttyUSB0");
    int32_t baud = 115200;
    private_nh_.param<int32_t>("baud", baud, baud);

    // Interface to motor controller.
    controller_ = new Controller(port.c_str(), baud);
    controller_->connect(); 
    if(controller_->connected()){
       ROS_INFO_STREAM("Open port "<<port.c_str()<<" with baud "<<baud<<" successfully and begin read...");
    }
    prev_time_ = ros::Time::now().toSec();
    resetTravelOffset();
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void SleoHardware::resetTravelOffset()
  {
    //controller_->getEncoder();
    ROS_ERROR_STREAM("Get original left encoder: "<<controller_->left_encoder_offset<<", and right: "<<controller_->right_encoder_offset);
    prev_encoder_l = controller_->left_encoder_offset;
    prev_encoder_r = controller_->right_encoder_offset;
    for (int i = 0; i < 4; i++)
    {
      if(i%2==0){
        joints_[i].position_offset = linearToAngular(controller_->encoder_to_meters(controller_->left_encoder_offset));
      }else{
        joints_[i].position_offset = linearToAngular(controller_->encoder_to_meters(controller_->right_encoder_offset));
      }
    }
  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void SleoHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void SleoHardware::updateJointsFromHardware()
  {
    long delta_r = 0,delta_l = 0;
    double current_time,delta_time;
    current_time = ros::Time::now().toSec();
    delta_l = controller_->left_encoder - prev_encoder_l;
    delta_r = controller_->right_encoder - prev_encoder_r;

    ROS_DEBUG_STREAM("Received travel information (L:" << controller_->encoder_to_meters(controller_->left_encoder) << " R:" << controller_->encoder_to_meters(controller_->right_encoder) << ")");

    for (int i = 0; i < 4; i++)
    {
      double delta = 0;
      if(i%2==0){
        delta = linearToAngular(controller_->encoder_to_meters(controller_->left_encoder)) - joints_[i].position - joints_[i].position_offset;
        ROS_DEBUG_STREAM("Received Encoder Travel: "<<controller_->encoder_to_meters(controller_->left_encoder)<<
                        ", Angular: "<<linearToAngular(controller_->encoder_to_meters(controller_->left_encoder)) <<
                        ", joint["<<i<<"].position:"<<joints_[i].position<<
                        ", joint["<<i<<"].position_offset:"<<joints_[i].position_offset<<
                        ", joint["<<i<<"].effort:"<<joints_[i].effort<<
                        ", delta:"<<delta<<"."); 
      }else{
        delta = linearToAngular(controller_->encoder_to_meters(controller_->right_encoder)) - joints_[i].position - joints_[i].position_offset;
        ROS_DEBUG_STREAM("Received Encoder Travel: "<<controller_->encoder_to_meters(controller_->right_encoder)<<
                        ", Angular: "<<linearToAngular(controller_->encoder_to_meters(controller_->right_encoder)) <<
                        ", joint["<<i<<"].position:"<<joints_[i].position<<
                        ", joint["<<i<<"].position_offset:"<<joints_[i].position_offset<<
                        ", joint["<<i<<"].effort:"<<joints_[i].effort<<
                        ", delta:"<<delta<<"."); 
      }

      ROS_DEBUG_STREAM("Received travel information (L:" << linearToAngular(controller_->encoder_to_meters(controller_->left_encoder))<<",joints.position: "<<joints_[0].position<<" offset: "<<joints_[0].position_offset << " R:" << controller_->encoder_to_meters(controller_->left_encoder) << "), "<<": delta: "<<linearToAngular(controller_->encoder_to_meters(controller_->left_encoder)) - joints_[i].position - joints_[i].position_offset);

      // detect suspiciously large readings, possibly from encoder rollover
      if (std::abs(delta) < 2.0){
        joints_[i].position += delta;
      }else{
        // suspicious! drop this measurement and update the offset for subsequent readings
        joints_[i].position_offset += delta;
        ROS_DEBUG("Dropping overflow measurement from encoder");
      }
    }
    
    delta_time = current_time - prev_time_;
    ROS_DEBUG_STREAM("Received linear speed information (L:" << controller_->left_encoder << " R:" << controller_->right_encoder << ", delta_l: "<<delta_l<<
                    " delta time: "<<delta_time);
//    ROS_DEBUG_STREAM("Current rpm : "<<controller_->left_rpm<<", speed: "<<(double)controller_->left_rpm/3000*M_PI*0.3302);
    for (int i = 0; i < 4; i++){
      if (i % 2 == LEFT){
        joints_[i].velocity = linearToAngular(controller_->encoder_to_speed(delta_l,delta_time));
	    ROS_DEBUG_STREAM("jiounts_["<<i<<"].velocity:"<<joints_[i].velocity<<".");
      }else{
        // assume RIGHT
        joints_[i].velocity = linearToAngular(controller_->encoder_to_speed(delta_r,delta_time));
	    ROS_DEBUG_STREAM("jiounts_["<<i<<"].velocity:"<<joints_[i].velocity<<".");
      }
    }
     prev_time_ = current_time;
     prev_encoder_l = controller_->left_encoder;
     prev_encoder_r = controller_->right_encoder;
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void SleoHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    controller_->controlSpeed(diff_speed_left, diff_speed_right);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void SleoHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Sleo reports travel in metres, need radians for ros_control RobotHW
  */
  double SleoHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Sleo needs m/s,
  */
  double SleoHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


} // namespace sleo_driver
