# Send a command to the specified joint
#
# 'joint_name' is the name of the joint to control (any of the motor names listed
# in the 'order' or 'singles' lists located in a motor config file - except for shadow motors)
# 'cmd' can be a position [rad], velocity [rad/s], current [mA], or pwm depending on what operating
# mode the joint is currently set at.
#
# Note that the gripper can be controlled by publishing this type of message to the
# /<robot_name>/single_joint/command topic as well. There are two main differences between using
# this topic and the /<robot_name>/gripper/command topic for controlling the gripper:
#   1) By publishing to the /<robot_name>/gripper/command topic, you do not have to specify that you
#      are commanding the gripper - it's automatically understood.
#   2) When the gripper is in "position" control, using the /<robot_name>/gripper/command topic specifies a
#      linear distance in meters between the gripper fingers. However, using the /<robot_name>/single_joint/command
#      topic specifies an angular distance in radians.

string joint_name
float64 cmd
