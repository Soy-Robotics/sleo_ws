/**
*
*  \author Carl Zhang <carlzhang@soyrobotics.com>
*  \copyright Copyright (c) 2018-2020 Soy Robotics, Inc.
*   All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are
*  met:
*      * Redistributions of source code must retain the above copyright
*        notice, this list of conditions and the following disclaimer.
*      * Redistributions in binary form must reproduce the above
*        copyright notice, this list of conditions and the following disclaimer
*        in the documentation and/or other materials provided with the
*        distribution.
*      * Neither the name of Soy Robotics Inc. nor the names of its
*        contributors may be used to endorse or promote products derived from
*        this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@soyrobotics.com
*
*/

#ifndef SLEO_DASHBOARD_H
#define SLEO_DASHBOARD_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_sleo_dashboard.h>

#include <ros/package.h>
#include <ros/macros.h>
#include <ros/ros.h>
#include <sleo_msgs/Feedback.h>
#include <sleo_msgs/Status.h>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>

#include <vector>

namespace sleo_dashboard {

class SleoDashboard
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  SleoDashboard();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  void switchStatus(int systemFault, int systemStatus);

protected slots:

  virtual void updateTopicList();

protected:

  // deprecated function for backward compatibility only, use getTopics() instead
  ROS_DEPRECATED virtual QList<QString> getTopicList(const QSet<QString>& message_types);

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types);

//  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged();

protected:

  virtual void callbackSystemStatus(const sleo_msgs::Status::ConstPtr& status);

  virtual void callbackMotorStatusLeft(const sleo_msgs::Feedback::ConstPtr& feedback);
  virtual void callbackMotorStatusRight(const sleo_msgs::Feedback::ConstPtr& feedback);

  Ui::SleoDashboardWidget ui_;

  ros::Subscriber system_status_sub_;

  ros::Subscriber motor_status_sub_left_;
  ros::Subscriber motor_status_sub_right_;

  QWidget* widget_;

  QString topic_feedback_left_;
  QString topic_feedback_right_;
  QString topic_status_;
private:
  int system_fault_;
  int system_status_;
  std::string system_fault;
  std::string system_status;

  QString arg_topic_name;
public:

  enum Fault {
    FAULT_OVERHEAT=1,
    FAULT_OVERVOLTAGE=2,
    FAULT_UNDERVOLTAGE=4,
    FAULT_SHORT_CIRCUIT=8,
    FAULT_EMERGENCY_STOP=16,
    FAULT_SEPEX_EXCITATION_FAULT=32,
    FAULT_MOSFET_FAILURE=64,
    FAULT_STARTUP_CONFIG_FAULT=128
  } fault; 

  enum Status{
    STATUS_SERIAL_MODE=1,
    STATUS_PULSE_MODE=2,
    STATUS_ANALOG_MODE=4,
    STATUS_POWER_STAGE_OFF=8,
    STATUS_STALL_DETECTED=16,
    STATUS_AT_LIMIT=32,
    STATUS_MICROBASIC_SCRIPT_RUNNING=128
  }status;
};

}

#endif // SLEO_DASHBOARD_H
