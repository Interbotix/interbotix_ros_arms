import math
import rospy
import threading
import numpy as np
import modern_robotics as mr
from interbotix_sdk import angle_manipulation as ang
from interbotix_descriptions import interbotix_mr_descriptions as mrd

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_sdk.msg import JointCommands
from interbotix_sdk.msg import SingleCommand
from interbotix_sdk.srv import RobotInfo
from interbotix_sdk.srv import OperatingModes
from interbotix_sdk.srv import OperatingModesRequest
from interbotix_sdk.srv import RegisterValues
from interbotix_sdk.srv import RegisterValuesRequest

# Python Module that abstracts away the ROS layer and allows easy control of an Interbotix Arm's joints or end-effector
# Notes:
#       The end-effector frame (a.k.a Body frame) is located at /<robot_name>/ee_gripper_link
#       The World frame (a.k.a Space frame) is located at /<robot_name>/base_link
class InterbotixRobot(object):

    ### @brief Constructor for the InterbotixRobot object
    ### @param robot_name - namespace of the interbotix_sdk node
    ### @param arm_model - one of the robot models listed in the 'interbotix_mr_descriptions.py' module located in the interbotix_descriptions ROS package
    ### @param moving_time - time in seconds that it should take for the robot motion to complete
    ### @param accel_time - time in seconds that it should take for the motors to accelerate/decelerate - should never be more than half the moving_time
    ### @param gripper_pressure - value from 0 to 1 representing the pressure the gripper should exert when grasping an object
    def __init__(self, robot_name, arm_model=None, moving_time=2.0, accel_time=0.3, gripper_pressure=0.5):
        rospy.init_node(robot_name + "_robot_manipulation")                                                                         # Initialize ROS Node
        rospy.wait_for_service(robot_name + "/get_robot_info")                                                                      # Wait for ROS Services to become available
        rospy.wait_for_service(robot_name + "/set_operating_modes")
        rospy.wait_for_service(robot_name + "/set_motor_register_values")
        srv_robot_info = rospy.ServiceProxy(robot_name + "/get_robot_info", RobotInfo)                                              # Create Service Proxies
        self.srv_set_op = rospy.ServiceProxy(robot_name + "/set_operating_modes", OperatingModes)
        self.srv_set_register = rospy.ServiceProxy(robot_name + "/set_motor_register_values", RegisterValues)
        self.resp = srv_robot_info()                                                                                                # Get Robot Info like joint limits
        self.set_trajectory_time(moving_time, accel_time)                                                                           # Change the Profile Velocity/Acceleration Registers in the Arm motors
        self.joint_indx_dict = dict(zip(self.resp.joint_names, range(self.resp.num_single_joints)))                                 # Map joint names to their respective indices in the joint limit lists
        self.gripper_moving = False                                                                                                 # When in PWM mode, False means the gripper PWM is 0; True means the gripper PWM in nonzero
        self.gripper_command = Float64()                                                                                            # ROS Message to hold the gripper PWM command
        self.set_gripper_pressure(gripper_pressure)                                                                                 # Maps gripper pressure to a PWM range
        self.gripper_index = self.joint_indx_dict["gripper"] + 1                                                                    # Index of the 'left_finger' joint in the JointState message
        self.initial_guesses = [[0.0] * self.resp.num_joints for i in range(3)]                                                     # Guesses made up of joint values to seed the IK function
        self.initial_guesses[1][0] = np.deg2rad(-120)
        self.initial_guesses[2][0] = np.deg2rad(120)
        self.joint_states = None                                                                                                    # Holds the current joint states of the arm
        self.js_mutex = threading.Lock()                                                                                            # Mutex to prevent writing/accessing the joint states variable at the same time
        if arm_model is None:
            arm_model = robot_name
        self.robot_des = getattr(mrd, arm_model)                                                                                    # Contains the Modern Robotics description of the desired arm model
        self.pub_gripper_command = rospy.Publisher(robot_name + "/gripper/command", Float64, queue_size=100)                        # ROS Publisher to command the gripper
        self.pub_joint_commands = rospy.Publisher(robot_name + "/joint/commands", JointCommands, queue_size=100)                    # ROS Publisher to command the arm
        self.pub_single_command = rospy.Publisher(robot_name + "/single_joint/command", SingleCommand, queue_size=100)              # ROS Publisher to command a specified joint
        self.pub_joint_traj = rospy.Publisher(robot_name + "/arm_controller/joint_trajectory", JointTrajectory, queue_size=100)     # ROS Pubilsher to command Cartesian trajectories
        self.sub_joint_states = rospy.Subscriber(robot_name + "/joint_states", JointState, self.joint_state_cb)                     # ROS Subscriber to get the current joint states
        while (self.joint_states == None and not rospy.is_shutdown()): pass                                                         # Wait until the first JointState message is received
        self.joint_positions = list(self.joint_states.position[:self.resp.num_joints])                                              # Holds the desired joint positions
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_positions)                                      # Transformation matrix describing the pose of the end-effector w.r.t. the Space frame
        tmr_controller = rospy.Timer(rospy.Duration(0.02), self.controller)                                                         # ROS Timer to check gripper position
        rospy.loginfo("\nRobot Name: %s\nArm Model: %s\nMoving Time: %.2f seconds\nAcceleration Time: %.2f seconds\nGripper Pressure: %d%%" % (robot_name, arm_model, moving_time, accel_time, gripper_pressure * 100))
        rospy.sleep(1)                                                                                                              # Give time for the ROS Publishers to get set up

    ### @brief ROS Subscriber Callback function to update the latest arm joint states
    ### @param msg - latest JointState message
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    ### @brief ROS Timer Callback function to stop the gripper moving past its limits when in PWM mode
    ### @param event [unused] - Timer event message
    def controller(self, event):
        if (self.gripper_moving):
            with self.js_mutex:
                gripper_pos = self.joint_states.position[self.gripper_index]
            if ((self.gripper_command.data > 0 and gripper_pos >= self.resp.upper_gripper_limit) or
                (self.gripper_command.data < 0 and gripper_pos <= self.resp.lower_gripper_limit)):
                self.gripper_command.data = 0
                self.pub_gripper_command.publish(self.gripper_command)
                self.gripper_moving = False

    ### @brief Command positions to the arm joints (when in 'position' control mode)
    ### @param joint_positions - desired joint positions [rad] excluding gripper
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def set_joint_positions(self, joint_positions, moving_time=None, accel_time=None, blocking=True):
        self.publish_positions(joint_positions, moving_time, accel_time, blocking)

    ### @brief Command the arm to go to its Home pose (when in 'position' control mode)
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def go_to_home_pose(self, moving_time=None, accel_time=None, blocking=True):
        self.publish_positions(self.resp.home_pos, moving_time, accel_time, blocking)

    ### @brief Command the arm to go to its Sleep pose (when in 'position' control mode)
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def go_to_sleep_pose(self, moving_time=None, accel_time=None, blocking=True):
        self.publish_positions(self.resp.sleep_pos, moving_time, accel_time, blocking)

    ### @brief Command a single joint to a desired position (when in 'position' control mode)
    ### @param joint_name - name of the joint to control
    ### @param position - desired position [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @details - Note that if a moving_time or accel_time is specified, the changes affect ALL the arm joints, not just the specified one
    def set_single_joint_position(self, joint_name, position, moving_time=None, accel_time=None, blocking=True):
        self.set_trajectory_time(moving_time, accel_time)
        self.joint_positions[self.joint_indx_dict[joint_name]] = position
        single_command = SingleCommand(joint_name, position)
        self.pub_single_command.publish(single_command)
        if blocking:
            rospy.sleep(self.moving_time)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_positions)

    ### @brief Command a desired end-effector pose (when in 'position' control mode)
    ### @param T_sd - 4x4 Transformation Matrix representing the transform from /<robot_name>/ee_gripper_link to /<robot_name>/base_link
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @param theta_list [out] - joint values needed to get the end-effector to the desired pose
    ### @param <bool> [out] - True if a valid solution was found; False otherwise
    def set_ee_pose_matrix(self, T_sd, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        if (custom_guess is None):
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(self.robot_des.Slist, self.robot_des.M, T_sd, guess, 0.001, 0.001)
            solution_found = True

            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                theta_list = [int(elem * 1000)/1000.0 for elem in theta_list]
                for x in range(self.resp.num_joints):
                    if not (self.resp.lower_joint_limits[x] <= theta_list[x] <= self.resp.upper_joint_limits[x]):
                        solution_found = False
                        break
            else:
                solution_found = False

            if solution_found:
                if execute:
                    self.publish_positions(theta_list, moving_time, accel_time, blocking)
                    self.T_sb = T_sd
                return theta_list, True
            else:
                rospy.loginfo("Guess failed to converge...")

        rospy.loginfo("No valid pose could be found")
        return theta_list, False

    ### @brief Command a desired end-effector pose w.r.t. the Space frame (when in 'position' control mode)
    ### @param x - linear position along the X-axis of the Space frame [m]
    ### @param y - linear position along the Y-axis of the Space frame [m]
    ### @param z - linear position along the Z-axis of the Space frame [m]
    ### @param roll - angular position around the X-axis of the Space frame [rad]
    ### @param pitch - angular position around the Y-axis of the Space frame [rad]
    ### @param yaw - angular position around the Z-axis of the Space frame [rad]
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    ### @param theta_list [out] - joint values needed to get the end-effector to the desired pose
    ### @param <bool> [out] - True if a valid solution was found; False otherwise
    ### @details - Do not set 'yaw' if using an arm with fewer than 6dof
    def set_ee_pose_components(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=None, custom_guess=None, execute=True, moving_time=None, accel_time=None, blocking=True):
        if (self.resp.num_joints < 6 or (self.resp.num_joints >= 6 and yaw is None)):
            yaw = math.atan2(y,x)
        T_sd = np.identity(4)
        T_sd[:3,:3] = ang.eulerAnglesToRotationMatrix([roll, pitch, yaw])
        T_sd[:3, 3] = [x, y, z]
        return self.set_ee_pose_matrix(T_sd, custom_guess, execute, moving_time, accel_time, blocking)

    ### @brief Command a desired end-effector displacement that will follow a straight line path (when in 'position' control mode)
    ### @param x - linear displacement along the X-axis w.r.t. T_sy [m]
    ### @param y - linear displacement along the Y-axis w.r.t. T_sy [m]
    ### @param z - linear displacement along the Z-axis w.r.t. T_sy [m]
    ### @param roll - angular displacement around the X-axis w.r.t. T_sy [rad]
    ### @param pitch - angular displacement around the Y-axis w.r.t. T_sy [rad]
    ### @param yaw - angular displacement around the Z-axis w.r.t. T_sy [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param wp_moving_time - duration in seconds that each waypoint in the trajectory should move
    ### @param wp_accel_time - duration in seconds that each waypoint in the trajectory should be accelerating/decelerating (must be equal to or less than half of wp_moving_time)
    ### @param wp_period - duration in seconds between each waypoint
    ### @param <bool> [out] - True if a trajectory was succesfully planned and executed; otherwise False
    ### @details - T_sy is a 4x4 transformation matrix representing the pose of a virtual frame w.r.t. /<robot_name>/base_link.
    ###            This virtual frame has the exact same x, y, z, roll, and pitch of /<robot_name>/base_link but contains the yaw
    ###            of the end-effector frame (/<robot_name>/ee_gripper_link).
    ###            Note that 'y' and 'yaw' must equal 0 if using arms with less than 6dof.
    def set_ee_cartesian_trajectory(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=None, wp_moving_time=0.5, wp_accel_time=0.25, wp_period=0.05):
        if self.resp.num_joints < 6 and (y is not 0 or yaw is not 0):
            rospy.loginfo("Please leave the 'y' and 'yaw' fields at '0' when working with arms that have less than 6dof.")
            return False
        rpy = ang.rotationMatrixToEulerAngles(self.T_sb[:3,:3])
        T_sy = np.identity(4)
        T_sy[:3,:3] = ang.eulerAnglesToRotationMatrix([0.0, 0.0, rpy[2]])
        T_yb = np.dot(mr.TransInv(T_sy), self.T_sb)
        rpy[2] = 0.0
        if (moving_time == None):
            moving_time = self.moving_time
        accel_time = self.accel_time
        N = int(moving_time / wp_period)
        inc = 1.0 / float(N)
        joint_traj = JointTrajectory()
        joint_positions = list(self.joint_positions)
        for i in range(N+1):
            joint_traj_point = JointTrajectoryPoint()
            joint_traj_point.positions = joint_positions
            joint_traj_point.time_from_start = rospy.Duration.from_sec(i * wp_period)
            joint_traj.points.append(joint_traj_point)
            if (i == N):
                break
            T_yb[:3,3] += [inc * x, inc * y, inc * z]
            rpy[0] += inc * roll
            rpy[1] += inc * pitch
            rpy[2] += inc * yaw
            T_yb[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
            T_sd = np.dot(T_sy, T_yb)
            theta_list, success = self.set_ee_pose_matrix(T_sd, joint_positions, False, blocking=False)
            if success:
                joint_positions = theta_list
            else:
                rospy.loginfo("%.1f%% of trajectory successfully planned. Trajectory will not be executed." % (i/float(N) * 100))
                break

        if success:
            self.set_trajectory_time(wp_moving_time, wp_accel_time)
            joint_traj.joint_names = self.resp.joint_names[:self.resp.num_joints]
            with self.js_mutex:
                current_positions = list(self.joint_states.position[:self.resp.num_joints])
            joint_traj.points[0].positions = current_positions
            joint_traj.header.stamp = rospy.Time.now()
            self.pub_joint_traj.publish(joint_traj)
            rospy.sleep(moving_time + wp_moving_time)
            self.T_sb = T_sd
            self.joint_positions = joint_positions
            self.set_trajectory_time(moving_time, accel_time)

        return success

    ### @brief Set the joints to a different operating mode - Note: by default, joints start in 'position' control mode
    ### @param mode - either "position", "velocity", "pwm", or "current" ("current" only for the ViperX robots)
    def set_joint_operating_mode(self, mode):
        self.srv_set_op(cmd=OperatingModesRequest.ARM_JOINTS, mode=mode)

    ### @brief Command the joints (when not in 'position' control mode)
    ### @param commands - the type of commands corresponds to the joint operating mode
    ### @param delay - number of seconds to delay while the robot moves
    ### @details - if "pwm", values range from -885 to +885
    ###            if "velocity", values typically range from -3.14 to +3.14 [rad/s]
    ###            if "current", values range from -3000 to +3000 [mA]
    ###            if "position", values can range from -3.14 to +3.14 [rad] - however,
    ###            the 'set_joint_positions' function should be used in this case
    def set_joint_commands(self, commands, delay=0):
        joint_commands = JointCommands(commands)
        self.pub_joint_commands.publish(joint_commands)
        if (delay > 0):
            rospy.sleep(delay)

    ### @brief Set the gripper to a different operating mode - Note: by default, the gripper starts in 'pwm' control mode
    ### @param mode - either "position", "ext_position", "velocity", "pwm", or "current" ("current" only for the ViperX robots)
    def set_gripper_operating_mode(self, mode):
        if (self.gripper_moving):
            rospy.logwarn("Please put down object before changing the gripper operating mode.")
            return
        self.srv_set_op(cmd=OperatingModesRequest.GRIPPER, mode=mode)

    ### @brief Command the gripper (when not in 'pwm' control mode)
    ### @param command - the type of command corresponds to the gripper operating mode
    ### @param delay - number of seconds to delay while the gripper moves
    ### @details - if "pwm", values range from -885 to +885 - however, the 'open_gripper',
    ###            'close_gripper', and 'set_gripper_pressure' functions should be used in this case
    ###            if "velocity", values typically range from -3.14 to +3.14 [rad/s]
    ###            if "current", values range from -3000 to +3000 [mA]
    ###            if "position", values range from 0.015 (closed) to 0.037 (open) [m]
    ###            if "ext_position", values can be +/- multiples of PI [rad] - however,
    ###            this should only be used with a custom gripper capable of multiple rotations
    def set_gripper_command(self, command, delay=0):
        gripper_command = Float64(command)
        self.pub_gripper_command.publish(gripper_command)
        if (delay > 0):
            rospy.sleep(delay)

    ### @brief Set the amount of pressure that the gripper should use when grasping an object (when in 'pwm' control mode)
    ### @param pressure - a scaling factor from 0 to 1 where the pressure increases as the factor increases
    ### @details - the PWM range goes from 150 - 350. Anything higher and the gripper runs the risk of overloading
    def set_gripper_pressure(self, pressure):
        self.gripper_pwm = 150 + int(pressure * 200)

    ### @brief Opens the gripper (when in 'pwm' control mode)
    ### @param delay - number of seconds to delay before returning control to the user
    def open_gripper(self, delay=1.0):
        self.gripper_controller(self.gripper_pwm, delay)

    ### @brief Closes the gripper (when in 'pwm' control mode)
    ### @param delay - number of seconds to delay before returning control to the user
    def close_gripper(self, delay=1.0):
        self.gripper_controller(-self.gripper_pwm, delay)

    ### @brief Returns a list containing the positions of the arm joints (excluding the gripper)
    ### @param positions [out] - current joint positions [rad]
    def get_joint_positions(self):
        with self.js_mutex:
            positions = self.joint_states.position[:self.num_joints]
        return positions

    ### @brief Returns a floating point number containing the linear distance between the two gripper fingers
    ### @param <float> [out] - linear distance between the gripper fingers [m]
    def get_gripper_position(self):
        with self.js_mutex:
            gripper_pos = self.joint_states.position[self.gripper_index]
        return gripper_pos * 2.0

    ### @brief Helper function used to publish PWM commands to the gripper (when in 'pwm' mode)
    ### @param pwm - PWM command to send to the gripper motor
    ### @param delay - number of seconds to wait before returning control to the user
    def gripper_controller(self, pwm, delay):
        self.gripper_command.data = pwm
        with self.js_mutex:
            gripper_pos = self.joint_states.position[self.gripper_index]
        if ((self.gripper_command.data > 0 and gripper_pos < self.resp.upper_gripper_limit) or
            (self.gripper_command.data < 0 and gripper_pos > self.resp.lower_gripper_limit)):
            self.pub_gripper_command.publish(self.gripper_command)
            self.gripper_moving = True
            rospy.sleep(delay)

    ### @brief Helper function to publish joint positions and block if necessary (when in 'position' control mode)
    ### @param positions - desired joint positions
    ### @param moving_time - duration in seconds that the robot should move
    ### @param accel_time - duration in seconds that that robot should spend accelerating/decelerating (must be less than or equal to half the moving_time)
    ### @param blocking - whether the function should wait to return control to the user until the robot finishes moving
    def publish_positions(self, positions, moving_time=None, accel_time=None, blocking=True):
        self.set_trajectory_time(moving_time, accel_time)
        self.joint_positions = list(positions)
        joint_commands = JointCommands(positions)
        self.pub_joint_commands.publish(joint_commands)
        if blocking:
            rospy.sleep(self.moving_time)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, positions)

    ### @brief Helper function to set the amount of time it takes for the robot to execute a movement (when in 'position' control)
    ### @param moving_time - total time it should take for the robot to move [seconds]
    ### @param accel_time - time it should take for the joints to accelerate/decelerate [seconds]
    ### @details - accel_time must be less than or equal to half the moving_time
    def set_trajectory_time(self, moving_time, accel_time=0.3):
        if (moving_time != None):
            self.moving_time = moving_time
            self.srv_set_register(cmd=RegisterValuesRequest.ARM_JOINTS, addr_name="Profile_Velocity", value=int(moving_time * 1000))
        if (accel_time != None):
            self.accel_time = accel_time
            self.srv_set_register(cmd=RegisterValuesRequest.ARM_JOINTS, addr_name="Profile_Acceleration", value=int(accel_time * 1000))
