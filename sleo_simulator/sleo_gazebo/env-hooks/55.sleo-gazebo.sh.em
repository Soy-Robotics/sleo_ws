# Set some sane defaults for the sleo stage launch environment
##Documentation:
# The colon command simply has its arguments evaluated and then succeeds.
# It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character.
# The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
# Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside.
# But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler
: ${SLEO_GAZEBO_MAP_FILE:=`rospack find sleo_gazebo`/maps/soy_playgen.yaml}
: ${SLEO_GAZEBO_WORLD_FILE:=`rospack find sleo_gazebo`/worlds/soy_playgen.world}

# Exports
export SLEO_GAZEBO_MAP_FILE
export SLEO_GAZEBO_WORLD_FILE
