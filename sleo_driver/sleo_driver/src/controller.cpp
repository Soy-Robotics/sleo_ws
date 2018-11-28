/**
Software License Agreement (BSD)

\file      controller.cpp
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
#include "sleo_driver/transport.h"

#include "sleo_msgs/Status.h"
#include "sleo_msgs/Feedback.h"
#include "serial/serial.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>


// Link to generated source from Microbasic script file.
extern const int script_ver = 28;
#define LEFT 0
#define RIGHT 1

using namespace std;

namespace sleo {

const std::string eol("\r");
const size_t max_line_length(128);

Controller::Controller(const char *port, int baud)
  : nh_("~"), port_(port), baud_(baud), connected_(false), receiving_script_messages_(false),
    version_(""), start_script_attempts_(0), serial_(NULL),
    left_amps(0), right_amps(0),
    left_cmd(0), right_cmd(0),
    left_pwr(0), right_pwr(0),
    left_rpm(0), right_rpm(0),
    left_encoder(0), right_encoder(0),
    left_encoder_offset(0), right_encoder_offset(0),
    vdr(0), vmot(0), v5out(0),
    left_battery_amps(0), right_battery_amps(0),
    internal_temp(0), channel1_temp(0), channel2_temp(0),
    fault_message(0),
    status_flag(0),
    analog_input0(0),analog_input1(0),analog_input2(0),analog_input3(0),analog_input4(0),analog_input5(0),analog_input6(0),analog_input7(0),
    e_stop_(false),
    command("!", this), query("?", this), param("^", this)
{
  pub_status_ = nh_.advertise<sleo_msgs::Status>("status", 1);
  pub_feedback_left_ = nh_.advertise<sleo_msgs::Feedback>("left/feedback", 1);
  pub_feedback_right_ = nh_.advertise<sleo_msgs::Feedback>("right/feedback", 1);
}

Controller::~Controller() {
}

void Controller::connect() {
  if (!serial_) serial_ = new serial::Serial();
  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  serial_->setTimeout(to);
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);

  for (int tries = 0; tries < 5; tries++) {
    try {
      serial_->open();
      query << "FID" << send;
      setSerialEcho(false);
      flush();
    } catch (serial::IOException) {
    }

    if (serial_->isOpen()) {
      query << "C" << send;
      flush();
      while(1){
        std::string msg = serial_->readline(max_line_length, eol);
        if(msg[0] == 'C'){
          sscanf(msg.c_str(),"C=%ld:%ld",&left_encoder_offset,&right_encoder_offset);
          break;
        }
      }
      connected_ = true;
      return;
    } else {
      connected_ = false;
      ROS_INFO("Bad Connection with serial port Error %s",port_);
    }
  }

  ROS_INFO("Motor controller not responding.");
}

void Controller::read() {

  getMotorAmps();

  getMotorCmd();

  getMotorPwr();

  getMotorRPM();

  getEncoder();

  getVolts();

  getBatteryAmps();

#if 0
  getAnalogInput();
#endif

  getTemperature();

  getFaultMessage();

  getStatusFlag();

  std::vector<std::string> left_fields,right_fields,system_fields;

  left_fields.push_back(roboteq::toString(LEFT));
  left_fields.push_back(roboteq::toString(left_amps));
  left_fields.push_back(roboteq::toString(left_cmd));
  left_fields.push_back(roboteq::toString(left_pwr));
  left_fields.push_back(roboteq::toString(left_rpm));
  left_fields.push_back(roboteq::toString(left_encoder));
  left_fields.push_back(roboteq::toString(left_encoder_offset));
  left_fields.push_back(roboteq::toString(left_battery_amps));
  left_fields.push_back(roboteq::toString(channel1_temp));

  right_fields.push_back(roboteq::toString(RIGHT));
  right_fields.push_back(roboteq::toString(right_amps));
  right_fields.push_back(roboteq::toString(right_cmd));
  right_fields.push_back(roboteq::toString(right_pwr));
  right_fields.push_back(roboteq::toString(right_rpm));
  right_fields.push_back(roboteq::toString(right_encoder));
  right_fields.push_back(roboteq::toString(left_encoder_offset));
  right_fields.push_back(roboteq::toString(right_battery_amps));
  right_fields.push_back(roboteq::toString(channel2_temp));
  processFeedback(left_fields, right_fields);

  system_fields.push_back(roboteq::toString(fault_message));
  system_fields.push_back(roboteq::toString(status_flag));
  system_fields.push_back(roboteq::toString(internal_temp));
  system_fields.push_back(roboteq::toString(vdr));
  processStatus(system_fields);

}

void Controller::write(std::string msg) {
  tx_buffer_ << msg << eol;
}

void Controller::flush() {
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << boost::algorithm::replace_all_copy(tx_buffer_.str(), "\r", "\\r"));
  ssize_t bytes_written = serial_->write(tx_buffer_.str());
  if (bytes_written < tx_buffer_.tellp()) {
    ROS_WARN_STREAM("Serial write timeout, " << bytes_written << " bytes written of " << tx_buffer_.tellp() << ".");
  }
  tx_buffer_.str("");
}

void Controller::processStatus(std::vector<std::string> system_fields) {
  sleo_msgs::Status msg;
  msg.header.stamp = ros::Time::now();

  try {
    if (system_fields.size() != 4) {
      ROS_WARN("Wrong number of status fields. Dropping message.");
      return;
    }
    msg.fault = atoi(system_fields.at(0).c_str());
    msg.status = atoi(system_fields.at(1).c_str());
    msg.ic_temperature = atoi(system_fields.at(2).c_str());
    msg.internal_voltage = atof(system_fields.at(3).c_str());
  } catch (std::bad_cast& e) {
    ROS_WARN("Failure parsing status data. Dropping message.");
    return;
  }

  pub_status_.publish(msg);
}

void Controller::processFeedback(std::vector<std::string> left_fields,std::vector<std::string> right_fields) {
  if (left_fields.size() != 9 || right_fields.size() != 9) {
    ROS_WARN("Wrong number of feedback fields. Dropping message.");
    return;
  }
   
  sleo_msgs::Feedback left_msg, right_msg;
  left_msg.header.stamp = right_msg.header.stamp = ros::Time::now();

  // Scale factors as outlined in the relevant portions of the user manual, please
  // see mbs/script.mbs for URL and specific page references.
  try
  {
    left_msg.channel = atoi(left_fields.at(0).c_str());
    left_msg.motor_current = atof(left_fields.at(1).c_str()); 
    left_msg.commanded_velocity = from_rpm(atof(left_fields.at(2).c_str()));
    left_msg.motor_power = atof(left_fields.at(3).c_str());
    left_msg.measured_velocity = from_rpm(atof(left_fields.at(4).c_str()));
    left_msg.measured_position = from_encoder_ticks(atof(left_fields.at(5).c_str()));
    left_msg.measured_position_offset = from_encoder_ticks(atof(left_fields.at(6).c_str()));
    left_msg.supply_current = atof(left_fields.at(7).c_str());
    left_msg.channel_temperature = atoi(left_fields.at(8).c_str());

    right_msg.channel = atoi(right_fields.at(0).c_str());
    right_msg.motor_current = atof(right_fields.at(1).c_str()); 
    right_msg.commanded_velocity = from_rpm(atof(right_fields.at(2).c_str()));
    right_msg.motor_power = atof(right_fields.at(3).c_str());
    right_msg.measured_velocity = from_rpm(atof(right_fields.at(4).c_str()));
    right_msg.measured_position = from_encoder_ticks(atof(right_fields.at(5).c_str()));
    right_msg.measured_position_offset = from_encoder_ticks(atof(right_fields.at(6).c_str()));
    right_msg.supply_current = atof(right_fields.at(7).c_str());
    right_msg.channel_temperature = atoi(right_fields.at(8).c_str());
  }
  catch (std::bad_cast& e)
  {
    ROS_WARN("Failure parsing feedback data. Dropping message.");
    return;
  }
  pub_feedback_left_.publish(left_msg);
  pub_feedback_right_.publish(right_msg);
    
}


void Controller::getMotorAmps(){ 
  query << "A" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
    std::string msg = serial_->readline(max_line_length, eol);
    ROS_DEBUG_STREAM_NAMED("serial", "Amps: " << msg);

    if(msg[0] == 'A'){
      sscanf(msg.c_str(),"A=%f:%f",&left_amps,&right_amps);
      left_amps/=10;
      right_amps/=10;
      ROS_DEBUG_STREAM_NAMED("serial", "channel Amps 1: " << left_amps<<", 2: "<<right_amps);    
      break;
    }
  }
//  switchMessage(msg);
}

void Controller::getMotorCmd(){
  query << "M" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Motor Amps: " << msg);

  if(msg[0] == 'M'){
    sscanf(msg.c_str(),"M=%d:%d",&left_cmd,&right_cmd);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Command 1: " << left_cmd<<", channel 2: "<<right_cmd);    
    break;
  }
  }
//  switchMessage(msg);
}

void Controller::getMotorPwr(){
  query << "P" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Power: " << msg);

  if(msg[0] == 'P'){
    sscanf(msg.c_str(),"P=%d:%d",&left_pwr,&right_pwr);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Power 1: " << left_pwr<<", channel 2: "<<right_pwr);    
    break;
  }
  }
//  switchMessage(msg);

}

void Controller::getMotorRPM(){
  query << "BS" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "BS Speed: " << msg);

  if(msg[0] == 'B'){
    if(msg[1] == 'S'){
      sscanf(msg.c_str(),"BS=%d:%d",&left_rpm,&right_rpm);
      ROS_DEBUG_STREAM_NAMED("serial", "channel BS 1: " << left_rpm<<", channel 2: "<<right_rpm);    
      break;
    }
  }
  }
//  switchMessage(msg);
}

void Controller::getEncoder(){
  query << "C" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Encoder: " << msg);

  if(msg[0] == 'C'){
    sscanf(msg.c_str(),"C=%ld:%ld",&left_encoder,&right_encoder);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Encoder 1: " << left_encoder<<", Encoder 2: "<<right_encoder);    
    break;
  }
  }
//  switchMessage(msg);
}


void Controller::getVolts(){
  query << "V" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Volts: " << msg);

  if(msg[0] == 'V'){
    sscanf(msg.c_str(),"V=%f:%f:%f",&vdr,&vmot,&v5out);
    vdr /= 10;
    vmot /= 10;
    v5out /= 1000;
    ROS_DEBUG_STREAM_NAMED("serial", "Volts: vdr: " << vdr<<", vmot: "<<vmot<<", v5out: "<<v5out);    
    break;
  }
  }
//  switchMessage(msg);
}

void Controller::getBatteryAmps(){ 
  query << "BA" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Battery Amps: " << msg);

  if(msg[0] == 'B'){
    if(msg[1] == 'A'){
      sscanf(msg.c_str(),"BA=%f:%f",&left_battery_amps,&right_battery_amps);
      left_battery_amps/=10;
      right_battery_amps/=10;
      ROS_DEBUG_STREAM_NAMED("serial", "channel Battery Amps 1: " << left_battery_amps<<", 2: "<<right_battery_amps);    
      break;
    }
  }
  }
//  switchMessage(msg);
}

void Controller::getAnalogInput(){
  query << "AI" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Analog Input: " << msg);

  if(msg[0] == 'A'){
    if(msg[1] == 'I'){
      sscanf(msg.c_str(),"AI=%f:%f:%f:%f:%f:%f:%f:%f",&analog_input0,&analog_input1,&analog_input2,&analog_input3,&analog_input4,&analog_input5,&analog_input6,&analog_input7);
      ROS_DEBUG_STREAM_NAMED("serial", "Analog: Input0: "<< analog_input0/1000<<
                                      "\n\t\t\t\t        Input1: "<< analog_input1/1000<<
                                      "\n\t\t\t\t        Input2: "<< analog_input2/1000<<
                                      "\n\t\t\t\t        Input3: "<< analog_input3/1000<<
                                      "\n\t\t\t\t        Input4: "<< analog_input4/1000<<
                                      "\n\t\t\t\t        Input5: "<< analog_input5/1000<<
                                      "\n\t\t\t\t        Input6: "<< analog_input6/1000<<
                                      "\n\t\t\t\t        Input7: "<< analog_input7/1000);    
    break;
    }
  }
  }

//  switchMessage(msg);

}

void Controller::getTemperature(){
  query << "T" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Temperatures : " << msg);

  if(msg[0] == 'T'){
    sscanf(msg.c_str(),"T=%f:%f:%f",&internal_temp,&channel1_temp,&channel2_temp);
    ROS_DEBUG_STREAM_NAMED("serial", "Temperatures Internal: " << internal_temp<<", 1: "<<channel1_temp<<", 2:"<<channel2_temp);    
    break;
  } 
  }
//  switchMessage(msg);
}

void Controller::getFaultMessage(){
  query << "FF" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Fault : " << msg);

 if(msg[0] == 'F'){
    if(msg[1] == 'F'){
      sscanf(msg.c_str(),"FF=%d",&fault_message);
      ROS_DEBUG_STREAM_NAMED("serial", "Fault: " << fault_message);    
      break;
    }
  }
  }
//  switchMessage(msg);
}

void Controller::getStatusFlag(){
  query << "FS" << send;
  flush();
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  while(1){
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "Status : " << msg);

  if(msg[0] == 'F'){
    if(msg[1] == 'S'){
      sscanf(msg.c_str(),"FS=%d",&status_flag);
      ROS_DEBUG_STREAM_NAMED("serial", "Status: " << status_flag);    
      break;
    }
  }
  }
//  switchMessage(msg);
}

void Controller::switchMessage(std::string msg){

  if(msg[0] == 'A'){
    sscanf(msg.c_str(),"A=%f:%f",&left_amps,&right_amps);
    left_amps/=10;
    right_amps/=10;
    ROS_DEBUG_STREAM_NAMED("serial", "channel Amps 1: " << left_amps<<", 2: "<<right_amps);    
  }
  if(msg[0] == 'M'){
    sscanf(msg.c_str(),"M=%d:%d",&left_cmd,&right_cmd);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Command 1: " << left_cmd<<", channel 2: "<<right_cmd);    
  }
  if(msg[0] == 'P'){
    sscanf(msg.c_str(),"P=%d:%d",&left_pwr,&right_pwr);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Power 1: " << left_pwr<<", channel 2: "<<right_pwr);    
  }
  if(msg[0] == 'B'){
    if(msg[1] == 'S'){
      sscanf(msg.c_str(),"BS=%d:%d",&left_rpm,&right_rpm);
      ROS_DEBUG_STREAM_NAMED("serial", "channel BS 1: " << left_rpm<<", channel 2: "<<right_rpm);    
    }
  }
  if(msg[0] == 'V'){
    sscanf(msg.c_str(),"V=%f:%f:%f",&vdr,&vmot,&v5out);
    ROS_DEBUG_STREAM_NAMED("serial", "Volts: vdr: " << vdr/10<<", vmot: "<<vmot/10<<", v5out: "<<v5out/1000);    
  }
  if(msg[0] == 'C'){
    sscanf(msg.c_str(),"C=%ld:%ld",&left_encoder,&right_encoder);
    ROS_DEBUG_STREAM_NAMED("serial", "channel Encoder 1: " << left_encoder<<", Encoder 2: "<<right_encoder);    
  }
  if(msg[0] == 'B'){
    if(msg[1] == 'A'){
      sscanf(msg.c_str(),"BA=%f:%f",&left_battery_amps,&right_battery_amps);
      left_battery_amps/=10;
      right_battery_amps/=10;
      ROS_DEBUG_STREAM_NAMED("serial", "channel Battery Amps 1: " << left_battery_amps<<", 2: "<<right_battery_amps);    
    }
  }
  if(msg[0] == 'T'){
    sscanf(msg.c_str(),"T=%f:%f:%f",&internal_temp,&channel1_temp,&channel2_temp);
    ROS_DEBUG_STREAM_NAMED("serial", "Temperatures Internal: " << internal_temp<<", 1: "<<channel1_temp<<", 2:"<<channel2_temp);    
  } 
 if(msg[0] == 'F'){
    if(msg[1] == 'F'){
      sscanf(msg.c_str(),"FF=%d",&fault_message);
      ROS_DEBUG_STREAM_NAMED("serial", "Fault: " << fault_message);    
    }
  }
  if(msg[0] == 'F'){
    if(msg[1] == 'S'){
      sscanf(msg.c_str(),"FS=%d",&status_flag);
      ROS_DEBUG_STREAM_NAMED("serial", "Status: " << status_flag);    
    }
  }
  if(msg[0] == 'A'){
    if(msg[1] == 'I'){
      sscanf(msg.c_str(),"AI=%f:%f:%f:%f:%f:%f:%f:%f",&analog_input0,&analog_input1,&analog_input2,&analog_input3,&analog_input4,&analog_input5,&analog_input6,&analog_input7);
      ROS_DEBUG_STREAM_NAMED("serial", "Analog: Input0: "<< analog_input0/1000<<
                                      "\n\t\t\t\t        Input1: "<< analog_input1/1000<<
                                      "\n\t\t\t\t        Input2: "<< analog_input2/1000<<
                                      "\n\t\t\t\t        Input3: "<< analog_input3/1000<<
                                      "\n\t\t\t\t        Input4: "<< analog_input4/1000<<
                                      "\n\t\t\t\t        Input5: "<< analog_input5/1000<<
                                      "\n\t\t\t\t        Input6: "<< analog_input6/1000<<
                                      "\n\t\t\t\t        Input7: "<< analog_input7/1000);    
    }
  }
}

void Controller::controlSpeed(double diff_speed_left, double diff_speed_right){
  if(diff_speed_left ==0 && diff_speed_right==0){
    e_stop_ = true;
    command<<"EX"<<send;
    flush();
  }else{
    if(e_stop_){
      e_stop_ = false;
      command<<"MG"<<send;
      flush();      
    }
    ROS_DEBUG_STREAM("Commanding Left speed: " << speed_to_rpm(diff_speed_left) << " Right speed:"<<speed_to_rpm(diff_speed_right));
    command << "M" << speed_to_rpm(diff_speed_left) << speed_to_rpm(diff_speed_right) << send;
    flush();
  }
}

}  // namespace sleo
