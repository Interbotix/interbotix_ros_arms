from interbotix_sdk.robot_manipulation import InterbotixRobot
import numpy as np

# This script makes the end-effector perform pick, pour, and place tasks
def main():
    arm = InterbotixRobot(robot_name="wx250")
    arm.set_ee_pose_components(x=0.3, z=0.2)
    arm.set_single_joint_position("waist", np.pi/2.0)
    arm.open_gripper()
    arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    arm.close_gripper()
    arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    arm.set_single_joint_position("waist", -np.pi/2.0)
    arm.set_ee_cartesian_trajectory(pitch=1.5)
    arm.set_ee_cartesian_trajectory(pitch=-1.5)
    arm.set_single_joint_position("waist", np.pi/2.0)
    arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    arm.open_gripper()
    arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
