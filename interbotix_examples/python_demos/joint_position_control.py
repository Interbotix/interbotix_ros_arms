from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk arm_run.launch robot_name:=wx250s use_time_based_profile:=true gripper_operating_mode:=pwm'
# Then change to this directory and type 'python joint_position_control.py'

def main():
    joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57]
    arm = InterbotixRobot(robot_name="wx250s", mrd=mrd)
    arm.go_to_home_pose()
    arm.set_joint_positions(joint_positions)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
