/**
Software License Agreement (BSD)

\file      controller.h
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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
#ifndef ROBOTEQ_CONTROLLER
#define ROBOTEQ_CONTROLLER

#include "ros/ros.h"

#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
#include <stdint.h>
#include <string>

namespace serial {
  class Serial;
}

namespace sleo {
//class Channel;

class Controller {
//  friend class Channel;

public:

  float left_amps, right_amps;
  int left_cmd, right_cmd;
  int left_pwr, right_pwr;
  int left_rpm, right_rpm;
  long left_encoder, right_encoder;
  long left_encoder_offset, right_encoder_offset;
  float vdr, vmot, v5out;
  float left_battery_amps, right_battery_amps;
  float internal_temp, channel1_temp, channel2_temp;
  int fault_message;
  int status_flag;
  float analog_input0;
  float analog_input1;
  float analog_input2;
  float analog_input3;
  float analog_input4;
  float analog_input5; 
  float analog_input6; 
  float analog_input7;
  bool e_stop_;
private :
  const char *port_;
  int baud_;
  bool connected_;
  bool receiving_script_messages_;
  std::string version_;
  serial::Serial *serial_;
  std::stringstream tx_buffer_;

  std::vector<std::string> left_fields_;
  std::vector<std::string> right_fields_;
  ros::NodeHandle nh_;
  ros::Publisher pub_status_;
  ros::Publisher pub_feedback_left_;
  ros::Publisher pub_feedback_right_;
  double prev_time_;
  double delta_time_;

  void read();
  void write(std::string);

  void processStatus(std::vector<std::string> system_fields);
  void processFeedback(std::vector<std::string> left_fields,std::vector<std::string> right_fields);


protected:
  // These data members are the core of the synchronization strategy in this class.
  // In short, the sendWaitAck method blocks on receiving an ack, which is passed to
  // it from the read thread using the last_response_ string.
  std::string last_response_;
  boost::mutex last_response_mutex_;
  boost::condition_variable last_response_available_;
  bool haveLastResponse() { return !last_response_.empty(); }

  // These track our progress in attempting to initialize the controller.
  uint8_t start_script_attempts_;

  class EOMSend {};

  class MessageSender {
    public:
    MessageSender(std::string init, Controller* interface)
        : init_(init), interface_(interface) {}

    template<typename T>
    MessageSender& operator<<(const T val) {
      if (ss.tellp() == 0) {
        ss << init_ << val;
      } else {
        ss << ' ' << val;
      }
      return *this;
    }

    void operator<<(EOMSend)
    {
      //ROS_INFO_STREAM("Tx: "<<ss.str());
      interface_->write(ss.str());
      ss.str("");
    }

    private:
    std::string init_;
    Controller* interface_;
    std::stringstream ss;
  };

  MessageSender command;
  MessageSender query;
  MessageSender param;
  EOMSend send, sendVerify;

public :
  Controller (const char *port, int baud);
  ~Controller();

//  void addChannel(Channel* channel);
  void connect();
  bool connected() { return connected_; }
  void spinOnce() { read(); }
  void flush();

  // Send commands to motor driver.
  void setEstop() { command << "EX" << send; }
  void resetEstop() { command << "MG" << send; }
  void resetDIOx(int i) { command << "D0" << i << send; }
  void setDIOx(int i) { command << "D1" << i << send; }
  void startScript() { command << "R" << send; }
  void stopScript() { command << "R" << 0 << send; }
  void setUserVariable(int var, int val) { command << "VAR" << var << val << send; }
  void setUserBool(int var, bool val) { command << "B" << var << (val ? 1 : 0) << send; }

  void setSerialEcho(bool serial_echo) { param << "ECHOF" << (serial_echo ? 0 : 1) << sendVerify; }

  ///under functions is for sleo_msgs::Feedback 
  ///@TODO need to put in thread

  /*!
   * @brief Get current motor amps, map to motor left and right.
   * @param left_amps Left motor amps will be writed.
   * @param right_amps Right motor amps will be writed.
   */
  void getMotorAmps();

  /*!
   * @brief Get current motor command, map to motor left and right.
   * @param left_cmd Left motor command will be writed.
   * @param right_cmd Right motor command will be writed.
   */
  void getMotorCmd();

  /*!
   * @brief Get current motor power, map to motor left and right.
   * @param left_pwr Left motor power will be writed.
   * @param right_pwr Right motor power will be writed.
   */
  void getMotorPwr();

  /*!
   * @brief Get current motor rpm, map to motor left and right.
   * @param left_rpm Left motor rpm will be writed.
   * @param right_rpm Right motor rpm will be writed.
   */
  void getMotorRPM();

  /*!
   * @brief Get current motor encoder, map to motor left and right.
   * @param left_encoder Left motor encoder will be writed, cumulative value.
   * @param right_encoder Right motor encoder will be writed, cumulative value.
   */
  void getEncoder();//encoder

  /*!
   * @brief Get current voltages, map to internal voltage, mainly battery voltage and DSub connection 5V output.
   * @param vdr internal voltage will be writed.
   * @param vmot mainly battery voltage will be writed.
   * @param v5out DSub connection 5V output voltage will be writed.
   */
  void getVolts();

  /*!
   * @brief Get current Battery Amps, map to channel amps.
   * @param left_battery_amps channel 1 amps will be writed.
   * @param right_battery_amps channel 2 amps will be writed.
   */
  void getBatteryAmps();

  /*!
   * @brief Get current Analog Inpu, map to internal analog input.
   * @param analog_input0 channel 0 analog input will be writed.
   * @param analog_input1 channel 1 analog input will be writed.
   * @param analog_input2 channel 2 analog input will be writed.
   * @param analog_input3 channel 3 analog input will be writed.
   * @param analog_input4 channel 4 analog input will be writed.
   * @param analog_input5 channel 5 analog input will be writed.
   * @param analog_input6 channel 6 analog input will be writed.
   * @param analog_input7 channel 7 analog input will be writed.
   */
  void getAnalogInput();

  /*!
   * @brief Get current Temperature, map to interal temperature, and channel temperature.
   * @param internal_temp internal temperature will be writed.
   * @param channel1_temp channel 1 temperature will be writed.
   * @param channel2_temp channel 2 temperature will be writed.
   */
  void getTemperature();

  ///Under functions is for sleo_msgs::Status
  ///@TODO need to put in thread
  /*!
   * @brief Get current fault message, map to fault_message.
   * @param fault_message value with fault message.
   *  value|meaning
   * ------|-------------------
   *  0|normal
   *  1|OVERHEAT
   *  2|OVERVOLTAGE
   *  4|UNDERVOLTAGE
   *  8|SHORT_CIRCUIT
   *  16|EMERGENCY_STOP
   *  32|SEPEX_EXCITATION_FAULT
   *  64|MOSFET_FAILURE
   *  128|STARTUP_CONFIG_FAULT
   */
  void getFaultMessage();

  /*!
   * @brief Get current status flag, map to status_flag.
   * @param status_flag value with status message.
   *  value|meaning
   * ------|-------------------
   *  0|none
   *  1|SERIAL_MODE
   *  2|PULSE_MODE
   *  4|ANALOG_MODE
   *  8|POWER_STAGE_OFF
   *  16|STALL_DETECTED
   *  32|AT_LIMIT
   *  128|MICROBASIC_SCRIPT_RUNNING
   */
  void getStatusFlag();

  /*!
   * @brief Discarded this function because of will cause at least 1.2 seconds dely,
   * with instead of using while conditions.
   * @param msg read msg from serial will classify in here, and then map to correspond values.
   */
  void switchMessage(std::string msg);

 /**
  * Assumes 50:1 gear ratio.
  * wheel diameter:0.33 m
  * frequence: 10hz
  * x: delta encoder in 10hz
  */ 
 static double encoder_to_speed(double x, double delta_time)
  { 
    ROS_DEBUG_STREAM("Get encoder: "<<x<<", time is: "<<delta_time<<", speed: "<<x/8192/50*(M_PI*0.33)/delta_time);
    return x*M_PI*0.33/(8192.0*50.0*delta_time);
  }

 /**
  * Assumes 50:1 gear ratio.
  * wheel diameter:0.33 m
  */ 
 static double encoder_to_meters(double x)
  {
    ROS_DEBUG_STREAM("Get encoder: "<<x<<", meters is: "<<x/8192/50*(M_PI*0.33));
    return x*M_PI*0.33/(8192.0*50.0);
  }

  /*!
   * @brief Transform speed to motor rpm.
   * @param x speed in m/s.
   * @return transform speed in rpm.
   */
  static int speed_to_rpm(double x)
  {
    return (int)(x/M_PI/0.3302*50*60/6200*1000);
  }

  /*!
   * @brief Send diff command to roboteq driver.
   * @param diff_speed_left left speed in m/s.
   * @param diff_speed_right right speed in m/s.
   */
  void controlSpeed(double diff_speed_left, double diff_speed_right);

  bool read_e_status(){return e_stop_;}
protected:
  /**
   * @param x Angular velocity in radians/s.
   * @return Angular velocity in RPM.
   */
  static double to_rpm(double x)
  {
    return x * 60 / (2 * M_PI);
  }

  /**
   * @param x Angular velocity in RPM.
   * @return Angular velocity in rad/s.
   */
  static double from_rpm(double x)
  {                                                                                                                
    return x * (2 * M_PI) / 60;
  }

  /**
   * Conversion of radians to encoder ticks. Note that this assumes a
   * 2048-line quadrature encoder (hence 8192).
   *
   * @param x Angular position in radians.
   * @return Angular position in encoder ticks.
   */
  static double to_encoder_ticks(double x)
  {
    return x * 8192 / (2 * M_PI)/50;
  }

  /**
   * Conversion of encoder ticks to radians. Note that this assumes a
   * 2048-line quadrature encoder (hence 8192).
   *
   * @param x Angular position in encoder ticks.
   * @return Angular position in radians.
   */
  static double from_encoder_ticks(double x)
  {
    return x * (2 * M_PI) / 8192/50;
  }

};

}

#endif
