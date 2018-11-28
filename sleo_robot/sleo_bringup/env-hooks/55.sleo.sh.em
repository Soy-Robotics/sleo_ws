# Set some sane defaults for the sleo launch environment

##Documentation: 
#  The colon command simply has its arguments evaluated and then succeeds. 
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character. 
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside. 
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${SLEO_BASE:=sleo}                             # sleo
: ${SLEO_3D_SENSOR:=d415}				 		 # d415
: ${SLEO_3D_SENSOR_ENABLE:=false}             	 # enable 3d sensor
: ${SLEO_LASER:=sicklmsxx}				 		 # sicklmsxx
: ${SLEO_LASER_IP:=}
: ${SLEO_LASER_ENABLE:=true}				   	 # enable laser
: ${SLEO_VELODYNE:=32}                           # 16, 32, 64
: ${SLEO_VELODYNE_ENABLE:=false}                 # enable velodyne
: ${SLEO_UR5_IP:=}
: ${SLEO_UR5_ENABLE:=false}                      # enable ur5
: ${SLEO_SIMULATION:=false}
: ${SLEO_SERIAL_PORT:=/dev/sleo}                 # /dev/ttyUSB0, /dev/ttyS0
: ${SLEO_SCREEN_SERIAL_PORT:=/dev/sleo_screen}

# Exports
export SLEO_BASE
export SLEO_3D_SENSOR
export SLEO_3D_SENSOR_ENABLE
export SLEO_LASER
export SLEO_LASER_IP
export SLEO_LASER_ENABLE
export SLEO_VELODYNE
export SLEO_VELODYNE_ENABLE
export SLEO_UR5_IP
export SLEO_UR5_ENABLE
export SLEO_SIMULATION
export SLEO_SERIAL_PORT
export SLEO_SCREEN_SERIAL_PORT
