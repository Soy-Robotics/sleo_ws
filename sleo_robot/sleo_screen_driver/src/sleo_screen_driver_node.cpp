/**
 * @file /sleo_screen_driver/src/sleo_screen_driver_node.cpp
 *
 * @brief Implementation for dirver with read data from Sleo screen nodelet
 *
 * @author Carl
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <ecl/threads/thread.hpp>
#include <pluginlib/class_list_macros.h>
#include <sleo_screen_driver/sleo_screen_driver.hpp>
#include <sleo_msgs/Feedback.h>
#include <sleo_msgs/Status.h>

namespace sleo_screen_driver{

class SleoScreenDriverNode : public nodelet::Nodelet{

public:
  SleoScreenDriverNode() : shutdown_requested_(false),serialNumber_("/dev/ttyUSB0"),baudRate_(115200){}

  ~SleoScreenDriverNode(){
    NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }

  virtual void onInit(){

    ros::NodeHandle nh = this->getPrivateNodeHandle();
    std::string name = nh.getUnresolvedNamespace();
    nh.getParam("serialNumber", serialNumber_);
    nh.getParam("baudRate", baudRate_);

    fd = open(serialNumber_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
    if(fd < 0){
      ROS_ERROR_STREAM("Open Serial: "<<serialNumber_.c_str()<<" Error!");
      exit(0);
    }
    sleodriver_ = new SleoScreenDriver(serialNumber_, baudRate_);
    left_motor_feedback_sub_ = nh.subscribe("left/feedback", 1, &SleoScreenDriverNode::leftMotorFeedbackCb, this);
    right_motor_feedback_sub_ = nh.subscribe("right/feedback", 1, &SleoScreenDriverNode::rightMotorFeedbackCb, this);
    motor_status_sub_ = nh.subscribe("status", 1, &SleoScreenDriverNode::motorStatusCb, this);
    sleodriver_->Init();
    ROS_INFO_STREAM("Open serial: ["<< serialNumber_.c_str() <<" ] successful, with idex: "<<fd<<".");
    NODELET_INFO_STREAM("SleoScreenDriverNode initialised. Spinning up update thread ... [" << name << "]");
    update_thread_.start(&SleoScreenDriverNode::update, *this);
  }

private:
  void update(){
    ros::Rate spin_rate(10);
    while (! shutdown_requested_ && ros::ok())
    {
      sleodriver_->update();
      spin_rate.sleep();
    }// end while

    ROS_INFO_STREAM("Shutdown and close serial: "<<serialNumber_.c_str()<<".");
  }

  void leftMotorFeedbackCb(const sleo_msgs::FeedbackConstPtr feedback){
    sleodriver_->left_temperature_ = feedback->motor_temperature;
    sleodriver_->left_velocity_ = feedback->measured_velocity;
    sleodriver_->left_voltage_ = feedback->supply_voltage;
    sleodriver_->left_ampere_ = feedback->supply_current;
//    sleodriver_->left_temperature_ = feedback->channel_temperature;
  }

  void rightMotorFeedbackCb(const sleo_msgs::FeedbackConstPtr feedback){
    sleodriver_->right_temperature_ = feedback->motor_temperature;
    sleodriver_->right_velocity_ = feedback->measured_velocity;
    sleodriver_->right_voltage_ = feedback->supply_voltage;
    sleodriver_->right_ampere_ = feedback->supply_current;
//    sleodriver_->right_temperature_ = feedback->channel_temperature;
  }

  void motorStatusCb(const sleo_msgs::StatusConstPtr status){
    sleodriver_->system_temperature_ = status->ic_temperature;
    sleodriver_->system_voltage_ = status->internal_voltage;
    sleodriver_->system_fault_ = status->fault;
    sleodriver_->system_status_ = status->status;
    sleodriver_->switchStatus(sleodriver_->system_fault_, sleodriver_->system_status_);
  }

private:  
  int fd;
  SleoScreenDriver* sleodriver_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
  ros::Subscriber left_motor_feedback_sub_;
  ros::Subscriber right_motor_feedback_sub_;
  ros::Subscriber motor_status_sub_;
  // ROS Parameters
  std::string serialNumber_;
  int baudRate_;
};

} //namespace sleo_screen_driver_node

PLUGINLIB_EXPORT_CLASS(sleo_screen_driver::SleoScreenDriverNode, nodelet::Nodelet);
