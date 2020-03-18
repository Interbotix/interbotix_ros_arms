# Send a vector of position [rad], velocity [rad/s], current [mA], or pwm commands to a group of joints synchronously
# as defined in the 'order' sequence in the motor config files (excludes the 'gripper' joint if present)
#
# The order of the joints is the same as the order of the joint names as published in the joint states topic

float64[] cmd
