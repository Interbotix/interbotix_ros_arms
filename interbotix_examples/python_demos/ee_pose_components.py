from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script makes the end-effector go to a specific pose by defining the pose components
def main():
    arm = InterbotixRobot(robot_name="wx250")
    arm.go_to_home_pose()
    arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
