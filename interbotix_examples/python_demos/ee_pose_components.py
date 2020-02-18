from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

# This script makes the end-effector go to a specific pose by defining the pose components
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk arm_run.launch robot_name:=wx250 use_time_based_profile:=true gripper_operating_mode:=pwm'
# Then change to this directory and type 'python ee_pose_components.py'

def main():
    arm = InterbotixRobot(robot_name="wx250", mrd=mrd)
    arm.go_to_home_pose()
    arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
