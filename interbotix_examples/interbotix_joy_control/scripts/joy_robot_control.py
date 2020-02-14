#!/usr/bin/env python

import rospy
import threading
import numpy as np
import modern_robotics as mr
from interbotix_sdk import angle_manipulation as ang
from interbotix_descriptions import interbotix_mr_descriptions as mrd

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from interbotix_sdk.msg import JointCommands
from interbotix_sdk.srv import RobotInfo
from interbotix_joy_control.msg import ArmJoyControl

class ArmRobotControl(object):
    def __init__(self):

        # Initialization parameters to control the Interbotix Arm
        rospy.wait_for_service("get_robot_info")
        srv_robot_info = rospy.ServiceProxy("get_robot_info", RobotInfo)                                                # ROS Service to get joint limit information among other things
        self.resp = srv_robot_info()                                                                                    # Store the robot information in this variable
        self.joint_indx_dict = dict(zip(self.resp.joint_names, range(self.resp.num_single_joints)))                     # Map each joint-name to their respective index in the joint_names list (which conveniently matches their index in the joint-limit lists)
        self.joy_msg = ArmJoyControl()                                                                                  # Incoming message coming from the 'joy_control' node
        self.arm_model = rospy.get_param("~robot_name")                                                                 # Arm-model type
        self.num_joints = self.resp.num_joints                                                                          # Number of joints in the arm
        self.speed_max = 3.0                                                                                            # Max scaling factor when bumping up joint speed
        self.gripper_pwm = 200                                                                                          # Initial gripper PWM value
        self.gripper_moving = False                                                                                     # Boolean that is set to 'True' if the gripper is moving - 'False' otherwise
        self.gripper_command = Float64()                                                                                # Float64 message to be sent to the 'gripper' joint
        self.gripper_index = self.num_joints + 1                                                                        # Index of the 'left_finger' joint in the JointState message
        self.follow_pose = True                                                                                         # True if going to 'Home' or 'Sleep' pose
        self.joint_states = None                                                                                        # Holds the most up-to-date JointState message
        self.js_mutex = threading.Lock()                                                                                # Mutex to make sure that 'self.joint_states' variable isn't being modified and read at the same time
        self.joint_commands = JointCommands()                                                                           # JointCommands message to command the arm joints as a group
        self.target_positions = list(self.resp.sleep_pos)                                                               # Holds the 'Sleep' or 'Home' joint values
        self.robot_des = getattr(mrd, self.arm_model)                                                                   # Modern Robotics robot description
        self.joy_speeds = {"course" : 2.0, "fine" : 2.0, "current" : 2.0}                                               # Dictionary containing the desired joint speed scaling factors
        self.pub_joint_commands = rospy.Publisher("joint/commands", JointCommands, queue_size=100, latch=True)          # ROS Publisher to command joint positions [rad]
        self.pub_gripper_command = rospy.Publisher("gripper/command", Float64, queue_size=100)                          # ROS Publisher to command gripper PWM values
        self.sub_joy_commands = rospy.Subscriber("joy/commands", ArmJoyControl, self.joy_control_cb)                    # ROS Subscriber to get the joystick commands
        self.sub_joint_states = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)                       # ROS Subscriber to get the current joint states
        while (self.joint_states == None and not rospy.is_shutdown()): pass                                             # Wait until we know the current joint values of the robot
        self.joint_positions = list(self.joint_states.position[:self.num_joints])                                       # Holds the desired joint positions for the robot arm at any point in time
        self.yaw = 0.0                                                                                                  # Holds the desired 'yaw' of the end-effector w.r.t. the 'base_link' frame
        self.T_sy = np.identity(4)                                                                                      # Transformation matrix of a virtual frame with the same x, y, z, roll, and pitch values as 'base_link' but containing the 'yaw' of the end-effector
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.resp.sleep_pos)                           # Transformation matrix of the end-effector w.r.t. the 'base_link' frame
        self.T_yb = np.dot(mr.TransInv(self.T_sy), self.T_sb)                                                           # Transformation matrix of the end-effector w.r.t. T_sy
        tmr_controller = rospy.Timer(rospy.Duration(0.02), self.controller)                                             # ROS Timer to control the Interbotix Arm

    ### @brief Used to convert 'self.yaw' to a rotation matrix
    ### @param yaw - yaw angle to convert to a rotation matrix
    def yaw_to_rotation_matrix(self, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])

    ### @brief ROS Subscriber Callback function to update the latest arm joint states
    ### @param msg - latest JointState message
    ### @details - the JointState message is mainly used to determine current gripper position
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    ### @brief ROS Subscriber Callback function to receive the latest ArmJoyControl message
    ### @param msg - latest ArmJoyControl message
    def joy_control_cb(self, msg):
        self.joy_msg = msg

        # check gripper_cmd
        if (self.joy_msg.gripper_cmd != 0):
            with self.js_mutex:
                gripper_pos = self.joint_states.position[self.gripper_index]
            if (self.joy_msg.gripper_cmd == ArmJoyControl.GRIPPER_OPEN and gripper_pos < self.resp.upper_gripper_limit):
                self.gripper_command.data = self.gripper_pwm
            elif (self.joy_msg.gripper_cmd == ArmJoyControl.GRIPPER_CLOSE and gripper_pos > self.resp.lower_gripper_limit):
                self.gripper_command.data = -self.gripper_pwm
            self.pub_gripper_command.publish(self.gripper_command)
            self.gripper_moving = True

        # check robot_pose
        if (self.joy_msg.robot_pose != 0):
            if (self.joy_msg.robot_pose == ArmJoyControl.HOME_POSE):
                self.target_positions = list(self.resp.home_pos)
            elif (self.joy_msg.robot_pose == ArmJoyControl.SLEEP_POSE):
                self.target_positions = list(self.resp.sleep_pos)
            self.follow_pose = True
            # reset all transforms
            self.yaw = 0.0
            self.T_sy[:2,:2] = self.yaw_to_rotation_matrix(self.yaw)
            self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.target_positions)
            self.T_yb = np.dot(mr.TransInv(self.T_sy), self.T_sb)

        # check speed_cmd
        if (self.joy_msg.speed_cmd != 0):
            if (self.joy_msg.speed_cmd == ArmJoyControl.SPEED_INC and self.joy_speeds["current"] < self.speed_max):
                self.joy_speeds["current"] += 0.25
            elif (self.joy_msg.speed_cmd == ArmJoyControl.SPEED_DEC and self.joy_speeds["current"] > 1):
                self.joy_speeds["current"] -= 0.25
            rospy.loginfo("Current scaling factor is %.2f." % self.joy_speeds["current"])

        # check toggle_speed_cmd
        if (self.joy_msg.toggle_speed_cmd != 0):
            if (self.joy_msg.toggle_speed_cmd == ArmJoyControl.SPEED_COURSE):
                self.joy_speeds["fine"] = self.joy_speeds["current"]
                self.joy_speeds["current"] = self.joy_speeds["course"]
                rospy.loginfo("Switched to Course Control")
            elif (self.joy_msg.toggle_speed_cmd == ArmJoyControl.SPEED_FINE):
                self.joy_speeds["course"] = self.joy_speeds["current"]
                self.joy_speeds["current"] = self.joy_speeds["fine"]
                rospy.loginfo("Switched to Fine Control")

        # check gripper_pwm_cmd
        if (self.joy_msg.gripper_pwm_cmd != 0):
            if (self.joy_msg.gripper_pwm_cmd == ArmJoyControl.GRIPPER_PWM_INC and self.gripper_pwm < 350):
                self.gripper_pwm += 25
            elif (self.joy_msg.gripper_pwm_cmd == ArmJoyControl.GRIPPER_PWM_DEC and self.gripper_pwm > 150):
                self.gripper_pwm -= 25
            rospy.loginfo("Current PWM command is %d." % self.gripper_pwm)

    ### @brief ROS Timer Callback function running at 50 Hz that updates joint positions constantly
    ### @param event - ROS timer message (unused)
    def controller(self, event):

        # check end-effector related commands
        if (sum([self.joy_msg.ee_x_cmd, self.joy_msg.ee_y_cmd, self.joy_msg.ee_z_cmd, self.joy_msg.ee_roll_cmd, self.joy_msg.ee_pitch_cmd]) > 0 and self.follow_pose == False):

            # Copy the most recent T_yb transform into a temporary variable
            T_yb = np.array(self.T_yb)

            # check ee_x_cmd
            if (self.joy_msg.ee_x_cmd == ArmJoyControl.EE_X_INC):
                T_yb[0, 3] += 0.0025 * self.joy_speeds["current"]
            elif (self.joy_msg.ee_x_cmd == ArmJoyControl.EE_X_DEC):
                T_yb[0, 3] -= 0.0025 * self.joy_speeds["current"]

            # check ee_y_cmd
            if (self.joy_msg.ee_y_cmd == ArmJoyControl.EE_Y_INC and self.num_joints >= 6):
                T_yb[1, 3] += 0.0025 * self.joy_speeds["current"]
            elif (self.joy_msg.ee_y_cmd == ArmJoyControl.EE_Y_DEC and self.num_joints >= 6):
                T_yb[1, 3] -= 0.0025 * self.joy_speeds["current"]

            # check ee_z_cmd
            if (self.joy_msg.ee_z_cmd == ArmJoyControl.EE_Z_INC):
                T_yb[2, 3] += 0.0025 * self.joy_speeds["current"]
            elif (self.joy_msg.ee_z_cmd == ArmJoyControl.EE_Z_DEC):
                T_yb[2, 3] -= 0.0025 * self.joy_speeds["current"]

            # check end-effector orientation related commands
            if (self.joy_msg.ee_roll_cmd != 0 or self.joy_msg.ee_pitch_cmd != 0):
                rpy = ang.rotationMatrixToEulerAngles(T_yb[:3, :3])

                # check ee_roll_cmd
                if (self.joy_msg.ee_roll_cmd == ArmJoyControl.EE_ROLL_CCW):
                    rpy[0] += 0.01 * self.joy_speeds["current"]
                elif (self.joy_msg.ee_roll_cmd == ArmJoyControl.EE_ROLL_CW):
                    rpy[0] -= 0.01 * self.joy_speeds["current"]

                # check ee_pitch_cmd
                if (self.joy_msg.ee_pitch_cmd == ArmJoyControl.EE_PITCH_DOWN):
                    rpy[1] += 0.01 * self.joy_speeds["current"]
                elif (self.joy_msg.ee_pitch_cmd == ArmJoyControl.EE_PITCH_UP):
                    rpy[1] -= 0.01 * self.joy_speeds["current"]

                T_yb[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)

            # Get desired transformation matrix of the end-effector w.r.t. the base frame
            T_sd = np.dot(self.T_sy, T_yb)

            theta_list, success = mr.IKinSpace(self.robot_des.Slist, self.robot_des.M, T_sd, self.joint_positions, 0.001, 0.001)
            joint_limits_violated = False

            # Check to make sure no joint limits were violated
            for x in range(self.num_joints):
                if not (self.resp.lower_joint_limits[x] <= theta_list[x] <= self.resp.upper_joint_limits[x]):
                    joint_limits_violated = True
                    break

            # Also check to make sure that the 'IKinSpace' function found a valid solution.
            # Otherwise, don't update self.joint_positions or the other transforms
            if (success and not joint_limits_violated):
                self.T_sb = T_sd
                self.T_yb = T_yb
                self.joint_positions = theta_list
                self.joint_commands.cmd = self.joint_positions
                self.pub_joint_commands.publish(self.joint_commands)
            else:
                rospy.loginfo("No valid pose could be found")

        # check waist_cmd
        if (self.joy_msg.waist_cmd != 0 and self.follow_pose == False):
            if (self.joy_msg.waist_cmd == ArmJoyControl.WAIST_CCW and self.joint_positions[self.joint_indx_dict["waist"]] < self.resp.upper_joint_limits[self.joint_indx_dict["waist"]]):
                self.joint_positions[self.joint_indx_dict["waist"]] += 0.01 * self.joy_speeds["current"]
                self.yaw += 0.01 * self.joy_speeds["current"]
                if (self.joint_positions[self.joint_indx_dict["waist"]] > self.resp.upper_joint_limits[self.joint_indx_dict["waist"]]):
                    self.joint_positions[self.joint_indx_dict["waist"]] = self.resp.upper_joint_limits[self.joint_indx_dict["waist"]]
                    self.yaw = self.resp.upper_joint_limits[self.joint_indx_dict["waist"]]
            elif (self.joy_msg.waist_cmd == ArmJoyControl.WAIST_CW and self.joint_positions[self.joint_indx_dict["waist"]] > self.resp.lower_joint_limits[self.joint_indx_dict["waist"]]):
                self.joint_positions[self.joint_indx_dict["waist"]] -= 0.01 * self.joy_speeds["current"]
                self.yaw -= 0.01 * self.joy_speeds["current"]
                if (self.joint_positions[self.joint_indx_dict["waist"]] < self.resp.lower_joint_limits[self.joint_indx_dict["waist"]]):
                    self.joint_positions[self.joint_indx_dict["waist"]] = self.resp.lower_joint_limits[self.joint_indx_dict["waist"]]
                    self.yaw = self.resp.lower_joint_limits[self.joint_indx_dict["waist"]]
            self.T_sy[:2,:2] = self.yaw_to_rotation_matrix(self.yaw)
            self.T_sb = np.dot(self.T_sy, self.T_yb)
            self.joint_commands.cmd = self.joint_positions
            self.pub_joint_commands.publish(self.joint_commands)

        # get updated joint positions
        with self.js_mutex:
            js_msg = list(self.joint_states.position)

        # check gripper position
        if (self.gripper_moving):
            if ((self.gripper_command.data > 0 and js_msg[self.gripper_index] >= self.resp.upper_gripper_limit) or
               (self.gripper_command.data < 0 and js_msg[self.gripper_index] <= self.resp.lower_gripper_limit)):
                self.gripper_command.data = 0
                self.pub_gripper_command.publish(self.gripper_command)
                self.gripper_moving = False

        # check if the arm should slowly move to the 'Home' or 'Sleep' pose
        if (self.follow_pose):
            poses_are_equal = True
            for x in range(self.num_joints):
                if (self.target_positions[x] > self.joint_positions[x]):
                    self.joint_positions[x] += 0.01 * self.joy_speeds["current"]
                    if (self.joint_positions[x] > self.target_positions[x]):
                        self.joint_positions[x] = self.target_positions[x]
                    poses_are_equal = False
                elif (self.target_positions[x] < self.joint_positions[x]):
                    self.joint_positions[x] -= 0.01 * self.joy_speeds["current"]
                    if (self.joint_positions[x] < self.target_positions[x]):
                        self.joint_positions[x] = self.target_positions[x]
                    poses_are_equal = False
            self.joint_commands.cmd = self.joint_positions
            self.pub_joint_commands.publish(self.joint_commands)
            if (poses_are_equal):
                self.follow_pose = False
                rospy.loginfo("Done following pose")

def main():
    rospy.init_node('arm_robot_control')
    arm_robot_control = ArmRobotControl()
    rospy.spin()

if __name__=='__main__':
    main()
