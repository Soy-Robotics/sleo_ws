/**
 * @file /sleo_screen_driver/src/sleo_screen_driver.hpp
 *
 * @brief Implementation for dirver with read data from Sleo screen nodelet
 *
 * @author Carl
 *
 **/


/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SLEO_SCREEN_DRIVER_
#define SLEO_SCREEN_DRIVER_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <string>
#include <sleo_screen_driver/serial.hpp>

namespace sleo_screen_driver{

class SleoScreenDriver{
#define BUFSIZE 15
public:
  SleoScreenDriver(std::string serialNumber, int baudRate);

  ~SleoScreenDriver();

  void Init();

  void update();

  void switchStatus(int systemFault, int systemStatus);

public:

  ///Sleo controller fault message from sleo_msgs/Status
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

  ///Sleo controller Status message from sleo_msgs/Status
  enum Status{
    STATUS_SERIAL_MODE=1,
    STATUS_PULSE_MODE=2,
    STATUS_ANALOG_MODE=4,
    STATUS_POWER_STAGE_OFF=8,
    STATUS_STALL_DETECTED=16,
    STATUS_AT_LIMIT=32,
    STATUS_MICROBASIC_SCRIPT_RUNNING=128
  }status;
private:
  int fd, len;
  char temp_buf[BUFSIZE],result_buf[BUFSIZE];
  size_t screen_len;
  
  SleoSerial sleodriver_;
  std::string serialNumber_;
  int baudRate_;

public:
  // Screen Parameters
  float system_voltage_;
  float system_ampere_;
  float system_temperature_;
  int system_fault_;
  int system_status_;
  std::string system_fault;
  std::string system_status;

  float left_temperature_;
  float left_velocity_;
  float left_voltage_;
  float left_ampere_;

  float right_temperature_;
  float right_velocity_;
  float right_voltage_;
  float right_ampere_;
};

} //namespace sleo_screen_driver
#endif
