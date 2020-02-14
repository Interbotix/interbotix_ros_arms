from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script makes the end-effector draw a square in 3D space
def main():
    arm = InterbotixRobot(robot_name="wx250")
    arm.go_to_home_pose()
    arm.set_ee_cartesian_trajectory(z=-0.2)
    arm.set_ee_cartesian_trajectory(x=-0.2)
    arm.set_ee_cartesian_trajectory(z=0.2)
    arm.set_ee_cartesian_trajectory(x=0.2)
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
