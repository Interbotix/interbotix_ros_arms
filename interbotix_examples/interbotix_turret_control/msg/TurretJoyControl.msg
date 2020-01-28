# Send commands from the joy_node to the turret_control node

# enum values that define the joystick controls for the robot

#########################################################################################################

# Control the pan-and-tilt mechanism
int8 PAN_CCW = 1
int8 PAN_CW = 2
int8 TILT_UP = 3
int8 TILT_DOWN = 4
int8 PAN_TILT_HOME = 5

#########################################################################################################

# Customize configurations for the Interbotix Turret
# Inc/Dec Joint speed
int8 SPEED_INC = 6
int8 SPEED_DEC = 7

# Quickly toggle between a fast and slow speed setting
int8 SPEED_COURSE = 8
int8 SPEED_FINE = 9

#########################################################################################################

# Control the motion of the pan-and-tilt mechanism
int8 pan_cmd
int8 tilt_cmd

# Speed Configs
int8 speed_cmd
int8 toggle_speed_cmd
