# Send commands from the joy_node to the turret_control node

# enum values that define the joystick controls for the robot

# Control the motion of the pan joint
int8 PAN_CCW = 1
int8 PAN_CW = 2

# Control the motion of the tilt joint
int8 TILT_CCW = 3
int8 TILT_CW = 4

# Control the speed that the robot arm moves
int8 ARM_LOW_SPEED = 5
int8 ARM_HIGH_SPEED = 6

# Quickly toggle between a fast and slow speed setting
int8 ARM_COURSE_SPEED = 7
int8 ARM_FINE_SPEED = 8

# Set robot to specific pose
int8 HOME_POSE = 9

int8 pan_cmd
int8 tilt_cmd
int8 arm_speed_cmd
int8 arm_toggle_speed_cmd
int8 robot_pose
