# Send a vector of position [rad], velocity [rad/s], current [mA], or pwm commands to the robot arm joints
# as defined in the 'order' sequence in the motor config files
#
# The order of the joints is the same as the order of the joint names as published in the joint states topic

float64[] cmd
