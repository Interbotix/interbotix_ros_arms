from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

# This script closes and opens the gripper twice, changing the gripper pressure half way through
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk arm_run.launch robot_name:=wx200 use_time_based_profile:=true gripper_operating_mode:=pwm'
# Then change to this directory and type 'python gripper_control.py'

def main():
    arm = InterbotixRobot(robot_name="wx200", mrd=mrd)
    arm.close_gripper(2.0)
    arm.open_gripper(2.0)
    arm.set_gripper_pressure(1.0)
    arm.close_gripper(2.0)
    arm.open_gripper(2.0)

if __name__=='__main__':
    main()
