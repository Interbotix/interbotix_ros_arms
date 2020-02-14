from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script closes and opens the gripper twice, changing the gripper pressure half way through
def main():
    arm = InterbotixRobot(robot_name="wx200")
    arm.close_gripper(2.0)
    arm.open_gripper(2.0)
    arm.set_gripper_pressure(1.0)
    arm.close_gripper(2.0)
    arm.open_gripper(2.0)

if __name__=='__main__':
    main()
