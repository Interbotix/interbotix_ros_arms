from interbotix_sdk.robot_manipulation import InterbotixRobot
import numpy as np

# This script makes the end-effector go to a specific pose only possible with a 6dof arm using a transformation matrix
def main():
    T_sd = np.identity(4)
    T_sd[0,3] = 0.3
    T_sd[1,3] = 0.1
    T_sd[2,3] = 0.2

    arm = InterbotixRobot(robot_name="wx250s")
    arm.go_to_home_pose()
    arm.set_ee_pose_matrix(T_sd)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
