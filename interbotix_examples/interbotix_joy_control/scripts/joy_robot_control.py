#!/usr/bin/env python

import interbotix_mr_descriptions as mrd
import modern_robotics as mr
from enum import Enum
import numpy as np
import threading
import tf2_ros
import rospy
import pid

from interbotix_joy_control.msg import JoyControl
from interbotix_sdk.msg import JointCommands
from interbotix_sdk.srv import RobotInfo
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Note that the end-effector frame is defined as 'ee_arm_link' and that the 'Space' frame is equivalent to the 'base_link' frame

# 'ArmState' defines the current state of the joint controller (excludes gripper)
class ArmState(Enum):
    ROBOT_POSE_CONTROL = 1          # Robot is currently moving to the 'Home' or 'Sleep' pose
    SINGLE_JOINT = 2                # Individual control of either the 'waist', 'wrist_angle', or 'wrist_rotate' joints or a combination thereof is currently taking place
    VELOCITY_IK = 3                 # The end-effector of the robot is currently tracking a horizontal, vertical, or diagonal line in space
    STOPPED = 4                     # Robot is commanded to stop moving all its joints
    IDLE = 5                        # Controller is waiting for a command

# 'GripperState' defines the current state of the gripper controller
class GripperState(Enum):
    OPEN = 1                        # Gripper is currently opening
    CLOSE = 2                       # Gripper is currently closing
    IDLE = 3                        # Gripper is waiting for a command

class JoyRobotControl(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name")                # Name of the robot
        self.robot_des = getattr(mrd, self.robot_name)                  # Get the Modern Robotics description of the specified robot
        self.joint_states = None                                        # Current joint states
        self.ee_reference = None                                        # Snapshot of the end-effector pose w.r.t the 'Space' frame - used when tracking a horizontal line in space
        self.velocity_x_ik_only = False                                 # True if the end-effector is tracking a horizontal line in space only (as opposed to a vertical or diagonal line)
        self.gripper_state = GripperState.IDLE                          # Current gripper controller state
        self.joint_limit_padding = 0.2                                  # All joint limits are buffered by this amount [rad] as it takes time for joints to stop moving
        self.stop_single_joint = False                                  # True if a joint limit was breached while in 'SINGLE_JOINT' control mode
        self.ee_vel = 0.15                                              # Desired end-effector velocity [m/s] w.r.t the 'shoulder_link' frame
        self.controllers = []                                           # List of PID controller objects used when in the 'ROBOT_POSE_CONTROL' mode
        self.mutex = threading.Lock()                                   # Mutex to essentially ensure that the 'joy_control_cb' and 'controller' timer occur sequentially
        self.js_mutex = threading.Lock()                                # Mutex to get the most up-to-date joint states
        self.angles = None                                              # Desired angular positions of the joints (used in the 'ROBOT_POSE_CONTROL' mode)
        self.gripper_pwm = 300                                          # Desired gripper pwm when opening or closing
        self.arm_vel = 2.0                                              # Desired joint velocity [rad/s] when in 'SINGLE_JOINT' mode
        self.arm_vel_max = 2.5                                          # Max allowed single joint velocity [rad/s] when in 'SINGLE_JOINT' mode
        self.T_ssh = None                                               # Transform from the 'shoulder_link' frame w.r.t the 'Space' frame - used in the 'VELOCITY_IK' mode
        self.arm_state = ArmState.IDLE                                  # Current joint controller state
        self.last_active_arm_state = ArmState.IDLE                      # Last joint controller state (excluding 'STOPPED' and 'IDLE')
        self.pub_joint_commands = rospy.Publisher("joint/commands", JointCommands, queue_size=100)          # ROS Publisher to command joint velocities [rad/s]
        self.pub_gripper_command = rospy.Publisher("gripper/command", Float64, queue_size=100)              # ROS Publisher to command gripper position [rad]
        self.sub_joint_states = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)           # ROS Subscriber to get the current joint states
        self.sub_joy_commands = rospy.Subscriber("joy/commands", JoyControl, self.joy_control_cb)           # ROS Subscriber to get the joystick commands
        rospy.wait_for_service("get_robot_info")
        srv_robot_info = rospy.ServiceProxy("get_robot_info", RobotInfo)
        self.resp = srv_robot_info()                                                                        # ROS Service to get joint limits
        self.upper_joint_limits = [x - self.joint_limit_padding for x in self.resp.upper_joint_limits]      # Upper joint limits with padding included
        self.upper_joint_limits[2] = 1.0                                                                    # Prevent the 'elbow' joint from going past this position [rad] to avoid singularities
        self.lower_joint_limits = [x + self.joint_limit_padding for x in self.resp.lower_joint_limits]      # Lower joint limits with padding included
        self.num_joints = self.resp.num_joints                                                              # Number of joints in the arm
        self.joint_commands = JointCommands()                                                               # Class-wide Joint Commands message
        self.joint_commands.cmd = [0] * self.num_joints
        self.twist = [0, 0, 0, 0, 0, 0]                                                                     # Modern_Robotics Twist [Wx, Wy, Wz, Vx, Vy, Vz] of end-effector w.r.t the 'shoulder_link' frame
        for x in range(self.num_joints):
            pid_controller = pid.PID()
            self.controllers.append(pid_controller)
        while (self.joint_states == None and not rospy.is_shutdown()):
            pass
        self.get_initial_T_ssh()
        self.tmr_controller = rospy.Timer(rospy.Duration(0.01), self.controller)                             # ROS Timer to poll the arm and gripper control states

    ### @brief Get the transform from the 'shoulder_link' frame w.r.t. the 'Space' frame
    ### @details - This is used mainly to get the 'z' offset of the 'shoulder_link' from the 'base_link' frame.
    ###            As this offset never changes, the transform only has to be looked up once. The orientation of
    ###            the frame can be determined from the 'waist' joint position.
    def get_initial_T_ssh(self):
        self.T_ssh = np.array([[1.0 , 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(self.robot_name + "/base_link", self.robot_name + "/shoulder_link", rospy.Time(), rospy.Duration(1.0))
        self.T_ssh[2,3] = trans.transform.translation.z

    ### @brief Modify 'self.Tssh' to get updated with the latest 'waist' joint position
    ### @param yaw - position of the 'waist' joint in radians
    def yaw_to_rotation_matrix(self, yaw):
        self.T_ssh[:2, :2] = np.array([[np.cos(yaw), -np.sin(yaw)],
                                      [np.sin(yaw), np.cos(yaw)]])

    ### @brief Set the desired robot_pose goal (list of joint positions)
    ### @param robot_pose - An enum value specifying whether the arm should go to its 'Home' position or 'Sleep' position
    def set_robot_pose(self, robot_pose):
        # Clear the controller error variables
        for i in range(self.num_joints):
            self.controllers[i].clear()
        # Check robot_pose_cmd
        if (robot_pose == JoyControl.HOME_POSE):
            self.angles = self.resp.home_pos
        elif (robot_pose == JoyControl.SLEEP_POSE):
            self.angles = self.resp.sleep_pos

    ### @brief Set the desired end-effector twist w.r.t the 'shoulder_link' frame
    ### @param ee_z_cmd - An enum value specifying if the end-effector should go up or down
    ### @param ee_x_cmd - An enum value specifying if the end-effector should go forward or backwards
    def set_velocity_ik(self, ee_z_cmd, ee_x_cmd):
        # Check ee_z_cmd
        if (ee_z_cmd == JoyControl.EE_UP):
            self.twist[5] = self.ee_vel
        elif (ee_z_cmd == JoyControl.EE_DOWN):
            self.twist[5] = -self.ee_vel
        elif(ee_z_cmd == 0):
            self.twist[5] = 0

        # Check ee_x_cmd
        if (ee_x_cmd == JoyControl.EE_FORWARD):
            self.twist[3] = self.ee_vel
        elif (ee_x_cmd == JoyControl.EE_BACKWARD):
            self.twist[3] = -self.ee_vel
        elif (ee_x_cmd == 0):
            self.twist[3] = 0

    ### @brief Set the desired angular velocities [rad/s] of the 'waist', 'wrist_angle', and/or 'wrist_rotate' joints
    ### @param waist_cmd - An enum value specifying if the 'waist' joint should rotate CW or CCW
    ### @param wrist_angle_cmd - An enum value specifying if the 'wrist_angle' joint should rotate CW or CCW
    ### @param wrist_rotate_cmd - An enum value specifying if the 'wrist_rotate' joint should rotate CW or CCW
    def set_single_joint(self, waist_cmd, wrist_angle_cmd, wrist_rotate_cmd):
        self.joint_commands.cmd = [0] * self.num_joints

        # Check waist_cmd
        if (waist_cmd == JoyControl.WAIST_CCW):
            self.joint_commands.cmd[0] = self.arm_vel
        elif (waist_cmd == JoyControl.WAIST_CW):
            self.joint_commands.cmd[0] = -self.arm_vel

        # Check wrist_angle_cmd
        if (wrist_angle_cmd == JoyControl.WRIST_ANGLE_CCW):
            self.joint_commands.cmd[3] = self.arm_vel
        elif (wrist_angle_cmd == JoyControl.WRIST_ANGLE_CW):
            self.joint_commands.cmd[3] = -self.arm_vel

        # The px100 does not have a 'wrist_rotate' joint
        if (self.robot_name != "px100"):
            # Check wrist_rotate_cmd
            if (wrist_rotate_cmd == JoyControl.WRIST_ROTATE_CCW):
                self.joint_commands.cmd[4] = self.arm_vel
            elif (wrist_rotate_cmd == JoyControl.WRIST_ROTATE_CW):
                self.joint_commands.cmd[4] = -self.arm_vel

    ### @brief Set the desired speed of the arm joints
    ### @param arm_speed_cmd - An enum value specifying if the joints rotated in 'SINGLE_JOINT' mode should increment or decrement their speeds
    ### @param arm_toggle_speed_cmd - An enum value specifying if the joints rotated in 'SINGLE_JOINT' mode should go to preset speeds
    def set_arm_speeds(self, arm_speed_cmd, arm_toggle_speed_cmd):
        # Check arm_speed_cmd
        if (arm_speed_cmd == JoyControl.ARM_HIGH_SPEED):
            if (self.arm_vel < self.arm_vel_max):
                self.arm_vel += 0.25
            else:
                rospy.loginfo("Velocity upper threshold limit reached.")
        elif (arm_speed_cmd == JoyControl.ARM_LOW_SPEED):
            if (self.arm_vel > 0.25):
                self.arm_vel -= 0.25
            else:
                rospy.loginfo("Velocity lower threshold limit reached.")

        # Check arm_toggle_speed_cmd
        if (arm_toggle_speed_cmd == JoyControl.ARM_COURSE_SPEED):
            self.arm_vel = 2.25
        elif (arm_toggle_speed_cmd == JoyControl.ARM_FINE_SPEED):
            self.arm_vel = 0.75
        # adjust the end-effector speed proportionately to the 'single' joint speed
        self.ee_vel = 0.05 + 0.15*((self.arm_vel - 0.25)/2.25)
        rospy.loginfo("Velocity command is now %.2f rad/s." % self.arm_vel)
        rospy.loginfo("End Effector velocity is now %.3f m/s.\n" % self.ee_vel)

    ### @brief Set the desired pwm of the gripper (range from 150-350)
    ### @param gripper_pwm_cmd - An enum value specifying if the gripper pwm should be increased or decreased
    def set_gripper_pwm(self, gripper_pwm_cmd):
        # Check gripper_pwm_cmd
        if (gripper_pwm_cmd == JoyControl.GRIPPER_HIGH_PWM):
            if (self.gripper_pwm < 350):
                self.gripper_pwm += 25
            else:
                rospy.loginfo("Gripper PWM upper threshold limit reached.")
            rospy.loginfo("Gripper PWM command is now %d." % self.gripper_pwm)
        elif (gripper_pwm_cmd == JoyControl.GRIPPER_LOW_PWM):
            if (self.gripper_pwm > 150):
                self.gripper_pwm -= 25
            else:
                rospy.loginfo("Gripper PWM lower threshold limit reached.")
            rospy.loginfo("Gripper PWM command is now %d." % self.gripper_pwm)

    ### @brief Command the gripper to open or close
    ### @param gripper_cmd - An enum value specifying if the gripper should open or close
    ### @param gripper_pos - Linear distance [m] from the 'fingers_link' to the 'left_finger_link' - used to make sure that the gripper is not already open
    def set_gripper_cmd(self, gripper_cmd, gripper_pos):
        # Check gripper_cmd
        gripper_command = Float64()
        if (gripper_cmd == JoyControl.GRIPPER_OPEN and
            gripper_pos < self.resp.upper_gripper_limit):
            gripper_command.data = self.gripper_pwm
            self.gripper_state = GripperState.OPEN
        elif (gripper_cmd == JoyControl.GRIPPER_CLOSE):
            gripper_command.data = -self.gripper_pwm
            self.gripper_state = GripperState.CLOSE
        self.pub_gripper_command.publish(gripper_command)

    ### @brief ROS Subscriber callback function to get updated joint states
    ### @param msg - Updated joint state message
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    ### @brief ROS Subscriber callback function to get the latest joystick command
    ### @param msg - Latest joystick command containing custom Enum values
    ### @details - Note that this function is only called if the joystick state
    ###            has changed. It is NOT called at a certain frequency.
    def joy_control_cb(self, msg):
        with self.js_mutex:
            js_msg = self.joint_states.position

        with self.mutex:
            # Check if the gripper pwm or state should be changed
            if (self.resp.use_gripper == True):
                if (msg.gripper_pwm_cmd != 0):
                    self.set_gripper_pwm(msg.gripper_pwm_cmd)
                if (msg.gripper_cmd != 0):
                    self.set_gripper_cmd(msg.gripper_cmd, js_msg[-2])
            if (msg.arm_speed_cmd != 0 or msg.arm_toggle_speed_cmd != 0):
                self.set_arm_speeds(msg.arm_speed_cmd, msg.arm_toggle_speed_cmd)

            # Check if the arm should go to a 'Home' or 'Sleep' pose
            if (msg.robot_pose != 0):
                self.set_robot_pose(msg.robot_pose)
                self.arm_state = ArmState.ROBOT_POSE_CONTROL
                self.last_active_arm_state = ArmState.ROBOT_POSE_CONTROL

            # Check if the end-effector should move horizontally and/or vertically
            elif (msg.ee_z_cmd != 0 or msg.ee_x_cmd != 0):
                self.set_velocity_ik(msg.ee_z_cmd, msg.ee_x_cmd)
                if (msg.ee_z_cmd == 0):
                    if not (self.last_active_arm_state == ArmState.VELOCITY_IK and self.velocity_x_ik_only == True):
                        self.ee_reference = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, js_msg[:self.num_joints])
                    self.velocity_x_ik_only = True
                else:
                    self.velocity_x_ik_only = False
                self.arm_state = ArmState.VELOCITY_IK
                self.last_active_arm_state = ArmState.VELOCITY_IK

            # Check if indiviual joints should be rotated
            elif (msg.waist_cmd != 0 or msg.wrist_angle_cmd != 0 or msg.wrist_rotate_cmd != 0):
                self.set_single_joint(msg.waist_cmd, msg.wrist_angle_cmd, msg.wrist_rotate_cmd)
                self.arm_state = ArmState.SINGLE_JOINT
                self.last_active_arm_state = ArmState.SINGLE_JOINT
                self.check_joint_limits()
                self.pub_joint_commands.publish(self.joint_commands)

            # If the user is not pressing or toggling a button, then stop the robot
            else:
                if (self.arm_state != ArmState.ROBOT_POSE_CONTROL):
                    self.arm_state = ArmState.STOPPED

    ### @brief ROS timer for control loop
    ### @param event - ROS Timer event message
    ### @details - There are 4 main purposes of this timer:
    ###                 1) Check if the gripper is moving and if it is, stop the gripper once it is fully open or closed
    ###                 2) Check if the 'ROBOT_POSE_CONTROL' mode is active, and if it is, command joint velocities using PID controllers until the goal pose has been attained
    ###                 3) Check if the 'VELOCITY_IK' mode is active, and if it is, use the Space Jacobian to command the appropiate joint velocities
    ###                 4) Check to make sure no joint limits were violated
    def controller(self, event):
        with self.js_mutex:
            js_msg = self.joint_states.position

        with self.mutex:
            if ((self.gripper_state == GripperState.OPEN and js_msg[-2] >= self.resp.upper_gripper_limit) or
                (self.gripper_state == GripperState.CLOSE and js_msg[-2] <= self.resp.lower_gripper_limit)):
                gripper_cmd = Float64()
                gripper_cmd.data = 0
                self.pub_gripper_command.publish(gripper_cmd)
                self.gripper_state = GripperState.IDLE

            if (self.arm_state == ArmState.ROBOT_POSE_CONTROL):
                for i in range(self.num_joints):
                    self.joint_commands.cmd[i] = self.controllers[i].compute_control(self.angles[i], js_msg[i])
                if (sum(map(abs, self.joint_commands.cmd)) < 0.1):
                    self.arm_state = ArmState.STOPPED

            if (self.arm_state == ArmState.VELOCITY_IK):
                # calculate the latest T_ssh (transform of the 'shoulder_link' w.r.t. the 'Space' frame)
                self.yaw_to_rotation_matrix(js_msg[0])
                # calculate the latest T_sb (transform of the end-effector w.r.t. the 'Space' frame)
                T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, js_msg[:self.num_joints])
                # Transform T_sb to get the end-effector w.r.t. the 'shoulder_link' frame.
                T_shb = np.dot(mr.TransInv(self.T_ssh), T_sb)
                # Vsh holds the desired twist w.r.t the 'shoulder_link' frame. The 'shoulder_link' frame
                # is used since its 'X-axis' is always aligned with the 'X-axis' of the end-effector. Additionally,
                # its 'Z-axis' always points straight up. Thus, it's easy to think of twists in this frame.
                Vsh = self.twist
                # If the end-effector is only doing horizontal movement...
                if (self.velocity_x_ik_only == True):
                    # Calculate the error in the current end-effector pose w.r.t the initial end-effector pose.
                    err = np.dot(mr.TransInv(T_sb), self.ee_reference)
                    # convert this pose error into a twist w.r.t the end-effector frame
                    Vb_err = mr.se3ToVec(mr.MatrixLog6(err))
                    # transform this twist into the 'shoulder_link' frame as this is the frame that the desired twist is in
                    Vsh_err = np.dot(mr.Adjoint(T_shb), Vb_err)
                    # Set Vx to 0 and use the Vx value in self.twist instead
                    Vsh_err[3] = 0
                    Vsh = np.add(Vsh_err, self.twist)
                    # The whole process above is done to account for gravity. If the desired twist alone was converted to
                    # joint velocities, over time the end-effector would sag. Thus, by adding in the 'error' twist, the
                    # end-effector will track the initial height much more accurately.
                # Convert the twist from the 'shoulder_link' frame to the 'Space' frame using the Adjoint
                Vs = np.dot(mr.Adjoint(self.T_ssh), Vsh)
                # Calculte the Space Jacobian
                js = mr.JacobianSpace(self.robot_des.Slist, js_msg[:self.num_joints])
                # Calculate the joint velocities needed to achieve Vsh
                qdot = np.dot(np.linalg.pinv(js), Vs)
                # If any of the joint velocities violate their velocity limits, scale the twist by that amount
                scaling_factor = 1.0
                for x in range(self.num_joints):
                    if (abs(qdot[x]) > self.resp.velocity_limits[x]):
                        sample_factor = abs(qdot[x])/self.resp.velocity_limits[x]
                        if (sample_factor > scaling_factor):
                            scaling_factor = sample_factor
                if (scaling_factor != 1.0):
                    qdot[:] = [x / scaling_factor for x in qdot]
                self.joint_commands.cmd = qdot

            if (self.arm_state != ArmState.IDLE and self.arm_state != ArmState.STOPPED):
                self.check_joint_limits()

            if (self.arm_state == ArmState.STOPPED):
                # Send a speed of 0 rad/s to each joint - effectively, stopping all joints
                self.joint_commands.cmd = [0] * self.num_joints

            # Publish the joint_commands message
            if (self.arm_state != ArmState.IDLE and self.arm_state != ArmState.SINGLE_JOINT or self.stop_single_joint == True):
                self.pub_joint_commands.publish(self.joint_commands)
                if (self.stop_single_joint == True):
                    self.stop_single_joint = False
                if (self.arm_state == ArmState.STOPPED):
                    self.arm_state = ArmState.IDLE

    ### @brief Check to make sure none of the joint limits were breached, and if they were - stop the arm!!
    def check_joint_limits(self):
        with self.js_mutex:
            js_msg = self.joint_states.position

        for x in range(self.num_joints):
            # If an upper joint limit was breached...
            if (js_msg[x] >= self.upper_joint_limits[x]):
                # If the joint velocity would make the breach worse...
                if (self.joint_commands.cmd[x] > 0 and self.arm_state == ArmState.SINGLE_JOINT):
                    self.joint_commands.cmd[x] = 0
                    self.stop_single_joint = True
                elif (self.joint_commands.cmd[x] > 0 and self.arm_state == ArmState.VELOCITY_IK):
                    self.arm_state = ArmState.STOPPED
                    break
            # If a lower joint limit was breached...
            elif (js_msg[x] <= self.lower_joint_limits[x]):
                # If the joint velocity would make the breack wors...
                if (self.joint_commands.cmd[x] < 0 and self.arm_state == ArmState.SINGLE_JOINT):
                    self.joint_commands.cmd[x] = 0
                    self.stop_single_joint = True
                elif (self.joint_commands.cmd[x] < 0 and self.arm_state == ArmState.VELOCITY_IK):
                    self.arm_state = ArmState.STOPPED
                    break

def main():
    rospy.init_node('joy_robot_control')
    joy_robot_control = JoyRobotControl()
    rospy.spin()

if __name__=='__main__':
    main()
