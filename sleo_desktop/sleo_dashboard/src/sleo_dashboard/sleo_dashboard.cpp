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

#include <sleo_dashboard/sleo_dashboard.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sleo_msgs/Feedback.h>
#include <sleo_msgs/Status.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QTextCodec>

namespace sleo_dashboard {

SleoDashboard::SleoDashboard()
  : rqt_gui_cpp::Plugin(),
    widget_(0),
    system_fault("正常"),
    system_status("正常"),
    system_fault_(0),
    system_status_(0),
    topic_feedback_left_(""),
    topic_feedback_right_(""),
    topic_status_("")
{
//  QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));//中文乱码
  setObjectName("SleoDashboard");
}

void SleoDashboard::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  updateTopicList();
  onTopicChanged();
  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  QObject::connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  // set topic name if passed in as argument
//  const QStringList& argv = context.argv();
//  if (!argv.empty()) {
//    arg_topic_name = argv[0];
//    selectTopic(arg_topic_name);
//  }

}

void SleoDashboard::shutdownPlugin()
{
  system_status_sub_.shutdown();
  motor_status_sub_left_.shutdown();
  motor_status_sub_right_.shutdown();
}

void SleoDashboard::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sleo_msgs/Feedback");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sleo_msgs/Status");

//  QString selected = ui_.sleoTopicComboBox->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types).values();
//  topics.append("");
//  qSort(topics);
//  ui_.sleoTopicComboBox->clear();
//  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
//  {
//    QString label(*it);
//    label.replace(" ", "/");
//    ui_.sleoTopicComboBox->addItem(label, QVariant(*it));
//  }

  // restore previous selection
//  selectTopic(selected);
  onTopicChanged();
}

QList<QString> SleoDashboard::getTopicList(const QSet<QString>& message_types)
{
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types).values();
}

QSet<QString> SleoDashboard::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
       //topic_feedback_ = topic;
       if(topic.contains("left")){
           topic_feedback_left_= topic;
       }if(topic.contains("right")){
           topic_feedback_right_= topic;
       }
      // add raw topic
      topics.insert(topic);
      //qDebug("SleoDashboard::getTopics() raw topic '%s'", topic.toStdString().c_str());

    }
    if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      topic_status_ = topic;      
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("SleoDashboard::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}
#if 0
void SleoDashboard::selectTopic(const QString& topic)
{
  int index = ui_.sleoTopicComboBox->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.sleoTopicComboBox->addItem(label, QVariant(topic));
    index = ui_.sleoTopicComboBox->findText(topic);
  }
  ui_.sleoTopicComboBox->setCurrentIndex(index);
}
#endif

void SleoDashboard::onTopicChanged()
{

  system_status_sub_.shutdown();
  motor_status_sub_left_.shutdown();
  motor_status_sub_right_.shutdown();

  ros::NodeHandle nh(getNodeHandle());
  if(!topic_status_.isEmpty()){
    system_status_sub_ = nh.subscribe(topic_status_.toStdString(), 1, &SleoDashboard::callbackSystemStatus, this);
  }
  if(!topic_feedback_left_.isEmpty() || !topic_feedback_right_.isEmpty()){
    motor_status_sub_left_ = nh.subscribe(topic_feedback_left_.toStdString(), 1, &SleoDashboard::callbackMotorStatusLeft, this);
    motor_status_sub_right_ = nh.subscribe(topic_feedback_right_.toStdString(), 1, &SleoDashboard::callbackMotorStatusRight, this);
  }
}

void SleoDashboard::callbackSystemStatus(const sleo_msgs::Status::ConstPtr& status)
{
  ui_.systemTemperaturerEditLabel->setText(QString::fromUtf8("%1").arg(status->ic_temperature)); 
  ui_.systemVoltageEditLabel->setText(QString::fromUtf8("%1").arg(status->internal_voltage));
  system_fault_ = status->fault;
  system_status_ = status->status;
  switchStatus(system_fault_, system_status_);
  ui_.systemErrorMessageEditLabel->setText(QString::fromUtf8("%1").arg(system_fault.c_str()));
  ui_.systemStatusMessageEditLabel->setText(QString::fromLocal8Bit("%1").arg(system_status.c_str()));
}

void SleoDashboard::callbackMotorStatusLeft(const sleo_msgs::Feedback::ConstPtr& feedback)
{ 
  ui_.motorCurrentLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->motor_current));

  ui_.motorVoltageLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->motor_power));

  ui_.motorCommandVelocityLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->commanded_velocity));

  ui_.motorMeasuredVelocityLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->measured_velocity));

  ui_.motorMeasuredPositionLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->measured_position));

  ui_.systemCurrentEditLabel->setText(QString::fromUtf8("%1").arg(feedback->supply_current));

  ui_.motorTemperatureLeftEditLabel->setText(QString::fromUtf8("%1").arg(feedback->channel_temperature));
}

void SleoDashboard::callbackMotorStatusRight(const sleo_msgs::Feedback::ConstPtr& feedback)
{
  ui_.motorCurrentRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->motor_current));

  ui_.motorVoltageRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->motor_power));

  ui_.motorCommandVelocityRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->commanded_velocity));

  ui_.motorMeasuredVelocityRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->measured_velocity));

  ui_.motorMeasuredPositionRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->measured_position));

  ui_.systemCurrentEditLabel->setText(QString::fromUtf8("%1").arg(feedback->supply_current));

  ui_.motorTemperatureRightEditLabel->setText(QString::fromUtf8("%1").arg(feedback->channel_temperature));
}

void SleoDashboard::switchStatus(int systemFault, int systemStatus)
{
  switch(systemFault){
    case FAULT_OVERHEAT:                   system_fault = "过热";break;
    case FAULT_OVERVOLTAGE:                system_fault = "过压";break;
    case FAULT_UNDERVOLTAGE:               system_fault = "欠压";break;
    case FAULT_SHORT_CIRCUIT:              system_fault = "短路";break;
    case FAULT_EMERGENCY_STOP:             system_fault = "急停";break;
    case FAULT_SEPEX_EXCITATION_FAULT:     system_fault = "励磁";break;
    case FAULT_MOSFET_FAILURE:             system_fault = "MOSFET";break;
    case FAULT_STARTUP_CONFIG_FAULT:       system_fault = "配置";break;
    default:                               system_fault = "正常";break;
  }
  switch(systemStatus){
    case STATUS_SERIAL_MODE:                system_status = "串口";break;
    case STATUS_PULSE_MODE:                 system_status = "脉冲";break;
    case STATUS_ANALOG_MODE:                system_status = "模拟";break;
    case STATUS_POWER_STAGE_OFF:            system_status = "断电";break;
    case STATUS_STALL_DETECTED:             system_status = "失速";break;
    case STATUS_AT_LIMIT:                   system_status = "极限";break;
    case STATUS_MICROBASIC_SCRIPT_RUNNING:  system_status = "脚本";break;
    default:                                system_status = "正常";break;
  }
}

}

PLUGINLIB_EXPORT_CLASS(sleo_dashboard::SleoDashboard, rqt_gui_cpp::Plugin)
