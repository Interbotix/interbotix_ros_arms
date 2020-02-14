from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script commands some arbitrary positions to the arm joints
def main():
    joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57]
    arm = InterbotixRobot(robot_name="wx250s")
    arm.go_to_home_pose()
    arm.set_joint_positions(joint_positions)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
