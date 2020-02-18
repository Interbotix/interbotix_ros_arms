from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

# This script makes the end-effector draw a square in 3D space
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk arm_run.launch robot_name:=wx250 use_time_based_profile:=true gripper_operating_mode:=pwm'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'

def main():
    arm = InterbotixRobot(robot_name="wx250", mrd=mrd)
    arm.go_to_home_pose()
    arm.set_ee_cartesian_trajectory(z=-0.2)
    arm.set_ee_cartesian_trajectory(x=-0.2)
    arm.set_ee_cartesian_trajectory(z=0.2)
    arm.set_ee_cartesian_trajectory(x=0.2)
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
