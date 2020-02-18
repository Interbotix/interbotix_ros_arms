from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd

# This script commands currents [mA] to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_sdk arm_run.launch robot_name:=vx250 use_time_based_profile:=true gripper_operating_mode:=pwm'
# Then change to this directory and type 'python joint_current_control.py'

def main():
    joint_currents = [0, 200 , 200, 50, 0]
    arm = InterbotixRobot(robot_name="vx250", mrd=mrd)
    arm.set_joint_operating_mode("current")
    arm.set_joint_commands(joint_currents)

if __name__=='__main__':
    main()
