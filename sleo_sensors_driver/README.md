sleo_robot
===========

Robot ROS packages for the Soy Sleo, for hardware communication.

 - sleo_bringup : Bringup launch files and scripts.
 - sleo_control : Control configuration.
 - sleo_description : Robot description (URDF).
 - sleo_screen_driver : Screen driver.
 - sleo_sensors_driver : Accessories driver.
 - sleo_diagnostics : Sleo diagnostics system node.
 - sleo_driver : Communication with MCU with sleo.
 - sleo_msgs : Sleo communication messages in all system.

Note: First begin your sleo, make sure you have run:
```
rosrun  sleo_bringup  create_udev_rules
```
