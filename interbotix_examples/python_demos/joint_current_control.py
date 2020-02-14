from interbotix_sdk.robot_manipulation import InterbotixRobot

# This script commands currents to the arm joints
def main():
    joint_currents = [0, 200 , 200, 50, 0]
    arm = InterbotixRobot(robot_name="vx250")
    arm.set_joint_operating_mode("current")
    arm.set_joint_commands(joint_currents)

if __name__=='__main__':
    main()
