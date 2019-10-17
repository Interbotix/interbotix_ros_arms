#ifndef ROBOT_ARM_OBJ_H_
#define ROBOT_ARM_OBJ_H_

#include <ros/ros.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "interbotix_sdk/OperatingModes.h"
#include "interbotix_sdk/RegisterValues.h"
#include "interbotix_sdk/JointCommands.h"
#include "interbotix_sdk/FirmwareGains.h"
#include "interbotix_sdk/RobotInfo.h"
#include "interbotix_sdk/arm_poses.h"

#define BAUDRATE 1000000                                                // All motors are preset to 1M baud
#define PORT "/dev/ttyDXL"                                              // Udev rule creates a symlink with this name
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0                          // Write goal positions [rad] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1                          // Write goal velocities [rad/s] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 2                           // Write goal currents [mA] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_PWM 3                               // Write goal pwms to multiple motors at the same time
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0       // Read current joint states for multiple motors at the same time

static const uint8_t PWM_CONTROL_MODE = 16;                             // PWM control mode option for the 'Operating_Mode' register
static const int32_t MAX_PWM = 885;                                     // Max allowable Gripper PWM value for the 'Goal_PWM' register
static const int32_t JOINT_PROFILE_VELOCITY = 0;                        // Allow joint velocity to be infinite when in position control mode - makes robot very reactive to joint commands
static const int32_t JOINT_PROFILE_ACCELERATION = 0;                    // Allow joint acceleration to be infinite when in position control mode - makes robot very reactive to joint commands
static const int32_t WHEEL_PROFILE_ACCELERATION = 0;                    // Allow joint acceleration to be infinite when in velocity control mode - makes robot very reactive to joint commands
static const int GRIPPER_MAX_EFFORT = 600;                              // Max allowable gripper effort [mA] when operating the gripper through MoveIt
static const float HORN_RADIUS = 0.014;                                 // Distance from the center of the gripper servo horn to its edge [m] (not including the VX300 model)
static const float ARM_RADIUS = 0.024;                                  // Length of the 'swing arm' that connects the edge of the servo horn to a gripper finger (not including the VX300 model)
static const float HORN_RADIUS_VX300 = 0.022;                           // Distance from the center of the gripper servo horn to its edge [m] for the VX300 model
static const float ARM_RADIUS_VX300 = 0.036;                            // Length of the 'swing arm' that connects the edge of the servo horn to a gripper finger for the VX300 model
static const float PI = 3.14159265358979f;                              // Defines Pi

struct Motor
{
  std::string name;                                                     // Name of a motor (as defined in a robot-specific config file located in the interbotix_sdk/config directory)
  uint8_t motor_id;                                                     // Dynamixel ID corresponding the above motor name
};

struct Info
{
  uint8_t motor_id;                                                     // Dynamixel ID of a motor
  std::string item_name;                                                // Register name
  int32_t value;                                                        // Value to write to the above register for the specified motor
};

enum class State                                                        // Operating_Mode state to keep track of the arm joints and gripper operating modes
{
  NONE,                                                                 // When in this state, all motor commands will be ignored
  POSITION,                                                             // When in this state, all commands will be treated as goal positions (cannot go past 1 revolution)
  EXT_POSITION,                                                         // When in this state, all commands will be treated as goal positions (multiple revolutions allowed) - gripper support only
  VELOCITY,                                                             // When in this state, all commands will be treated as goal velocities [rad/s]
  CURRENT,                                                              // When in this state, all commands will be treated as goal currents [mA]
  PWM                                                                   // When in this state, all commands will be treated as goal pwms
};

class RobotArm
{
public:
    /// @brief Constructor for the RobotArm
    explicit RobotArm(ros::NodeHandle *node_handle, const std::string robot_name, const double timer_hz = 100);

    /// @brief Destructor for the RobotArm
    ~RobotArm();

    /// @brief Set operating mode for the arm joints to position [rad] control
    /// @param joint_profile_vel - max angular velocity that a motor is allowed to reach - uses register values
    /// @param joint_profile_accel - max acceleration that a motor is allowed to reach - uses register values
    void arm_set_joints_to_position_control(const int32_t joint_profile_vel = JOINT_PROFILE_VELOCITY, const int32_t joint_profile_accel = JOINT_PROFILE_ACCELERATION);

    /// @brief Set operating mode for the arm joints to velocity [rad/s] control
    /// @param wheel_profile_accel - max acceleration that a motor is allowed to reach - uses register values
    void arm_set_joints_to_velocity_control(const int32_t wheel_profile_accel = WHEEL_PROFILE_ACCELERATION);

    /// @brief Set operating mode for the arm joints to current [mA] control (Viper robots only)
    void arm_set_joints_to_current_control(void);

    /// @brief Set operating mode for the arm joints to pwm control
    void arm_set_joints_to_pwm_control(void);

    /// @brief Command joint positions
    /// @param joint_positions - array of joint positions [rad] to write to the motors; sequence of positions match the joint name order in the published joint_state messages
    void arm_set_joint_positions(const double joint_positions[]);

    /// @brief Command joint velocities
    /// @param joint_velocities - array of joint velocities [rad/s] to write to the motors; sequence of velocities match the joint name order in the published joint_state messages
    void arm_set_joint_velocities(const double joint_velocities[]);

    /// @brief Command joint currents (Viper robots only)
    /// @param joint_currents - array of joint currents [mA] to write to the motors; sequence of currents match the joint name order in the published joint_state messages
    void arm_set_joint_currents(const double joint_currents[]);

    /// @brief Command joint pwms
    /// @param joint_pwms - array of joint pwms to write to the motors; sequence of pwms match the joint name order in the published joint_state messages
    void arm_set_joint_pwms(int32_t joint_pwms[]);

    /// @brief Set operating mode for the gripper to position [rad] control
    /// @param joint_profile_vel - max angular velocity that the gripper motor is allowed to reach - uses register values
    /// @param joint_profile_accel - max acceleration that the gripper motor is allowed to reach - uses register values
    void arm_set_gripper_to_position_control(const int32_t joint_profile_vel = JOINT_PROFILE_VELOCITY, const int32_t joint_profile_accel = JOINT_PROFILE_ACCELERATION);

    /// @brief Set operating mode for the gripper to ext_position [rad] control (servo can perform multiple turns instead of only one revolution)
    /// @param joint_profile_vel - max angular velocity that the gripper motor is allowed to reach - uses register values
    /// @param joint_profile_accel - max acceleration that the gripper motor is allowed to reach - uses register values
    /// @details - this mode should only be used with a custom gripper that has the ability to freely rotate
    void arm_set_gripper_to_ext_position_control(const int32_t joint_profile_vel = JOINT_PROFILE_VELOCITY, const int32_t joint_profile_accel = JOINT_PROFILE_ACCELERATION);

    /// @brief Set operating mode for the gripper to velocity [rad/s] control
    /// @param wheel_profile_accel - max acceleration that the gripper motor is allowed to reach - uses register values
    void arm_set_gripper_to_velocity_control(const int32_t wheel_profile_accel = WHEEL_PROFILE_ACCELERATION);

    /// @brief Set operating mode for the gripper to current [mA] control (Viper robots only)
    void arm_set_gripper_to_current_control(void);

    /// @brief Set operating mode for the gripper to pwm control
    void arm_set_gripper_to_pwm_control(void);

    /// @brief Set gripper finger position
    /// @param dist - desired distance [m] between the gripper fingers
    void arm_set_gripper_linear_position(const float dist);

    /// @brief Set gripper angle
    /// @param horn_angle - desired gripper angular position [rad]
    void arm_set_gripper_angular_position(const float horn_angle);

    /// @brief Set gripper velocity
    /// @param vel - desired gripper angular velocity [rad/s]
    void arm_set_gripper_velocity(const float vel);

    /// @brief Set gripper current
    /// @param current - desired gripper current [mA]
    void arm_set_gripper_current(const float current);

    /// @brief Set gripper pwm
    /// @param pwm - desired pwm - values between 150-350 (or the negative equiavlent) seem to work best
    void arm_set_gripper_pwm(int32_t pwm);

    /// @brief Set max gripper effort
    /// @param max_effort - the max effort [mA] that the gripper Action server will allow before preempting
    void arm_set_gripper_max_effort(const double max_effort = GRIPPER_MAX_EFFORT);

    /// @brief Torque on all motors (including the gripper)
    void arm_torque_on(void);

    /// @brief Torque off all motors (including the gripper)
    void arm_torque_off(void);

private:
    ros::NodeHandle node;                                                                             // ROS node handler
    ros::Publisher pub_joint_states;                                                                  // Publishes joint states
    ros::Subscriber sub_joint_traj_msg;                                                               // Subscribes to the joint trajectory topic (excludes gripper)
    ros::Subscriber sub_gripper_traj_msg;                                                             // Subscribes to the gripper trajectory topic
    ros::Subscriber sub_joint_commands;                                                               // Subscribes to the joint commands topic (excludes gripper)
    ros::Subscriber sub_gripper_command;                                                              // Subscribes to the gripper command topic
    ros::ServiceServer srv_firmware_gains;                                                            // Service to set PID firmware gains for the motors
    ros::ServiceServer srv_operating_mode;                                                            // Service to set the operating modes for the motors
    ros::ServiceServer srv_set_register;                                                              // Service to set multiple motor registers
    ros::ServiceServer srv_get_register;                                                              // Service to get raw values from multiple motor registers
    ros::ServiceServer srv_get_robot_info;                                                            // Service to get information about the robot arm
    ros::ServiceServer srv_torque_on;                                                                 // Service to torque on the robot arm's motors
    ros::ServiceServer srv_torque_off;                                                                // Service to torque off the robot arm's motors
    ros::Timer tmr_joint_states;                                                                      // Timer to continuously publish the joint states
    ros::Timer tmr_joint_traj;                                                                        // Timer that writes joint commands to the motors (excludes gripper) based on a given trajectory
    ros::Timer tmr_gripper_traj;                                                                      // Timer that writes a command to the gripper motor based on a given trajectory
    trajectory_msgs::JointTrajectory jnt_tra_msg;                                                     // Joint trajectory message for the robot arm to follow (excludes gripper)
    trajectory_msgs::JointTrajectory gripper_tra_msg;                                                 // Joint trajectory message for the robot gripper to follow
    sensor_msgs::JointState joint_states;                                                             // Current joint states of the  robot arm
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *joint_action_server;    // Action Server that responds to MoveIt's Action Client to control the joints (excludes gripper)
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *gripper_action_server;  // Action Server that responds to MoveIt's Action client to control the gripper

    DynamixelWorkbench dxl_wb;                                  // Instance of DynamixelWorkbench to interface with the U2D2 and motor firmware
    std::vector<Info> dynamixel_info;                           // Vector of motor registers and values to write to them
    std::map<std::string, const ControlItem*> control_items;    // Dictionary mapping register names to places in memory

    const double timer_hz;                                      // Frequency at which all ROS timers should run - defaults to 100 Hz
    const std::string robot_name;                               // Name of the robot - the last 5 characters must be one of the robot models (ex. wx200)

    State arm_operating_mode;                                   // Arm operating mode (either position [rad], velocity [rad/s], current [mA], pwm, or none)
    std::vector<Motor> joints;                                  // Vector of all joint names (including  gripper) and their corresponding Dynamixel IDs; shadow motors are not included
    std::vector<Motor> all_motors;                              // Vector of all motor names (as specified by the 'order' sequence in the config file) and their corresponding IDs (includes shadow motors)
    uint8_t *joint_ids_read;                                    // Pointer to first element in a dynamic array of joint IDs (as specified in the 'joints' vector above) to read joint states
    uint8_t *joint_ids_write;                                   // Pointer to first element in a dynamic array of joint IDs (as specified in the 'joints' vector above - excluding gripper) to write joint commands
    size_t joint_num_write;                                     // Number of elements in 'joint_ids_write'
    double joint_start_time;                                    // ROS start time for a joint trajectory (excludes gripper)
    bool execute_joint_traj;                                    // True if the joint trajectory should be executed - False otherwise

    State gripper_operating_mode;                               // Gripper operating mode (either position [rad], ext_position [rad], velocity [rad/s], current [mA], pwm, or none)
    Motor gripper;                                              // Motor instance with the name of the gripper and its corresponding Dynamixel ID
    double horn_radius;                                         // Distance [m] from the gripper servo center to the end of the servo horn (1st link in slider-crank mechanism to drive the gripper fingers)
    double arm_radius;                                          // Distance [m] from the end of the 1st link to the gripper finger attachment point (2nd link in slider-crank mechanism to drive the gripper fingers)
    double gripper_effort;                                      // Current gripper effort in mA
    double gripper_max_effort;                                  // Max allowable gripper effort in mA before the gripper MoveIt trajectory is preempted
    double gripper_start_time;                                  // ROS start time for the gripper trajectory
    bool execute_gripper_traj;                                  // True if the gripper trajectory should be executed - False otherwise
    bool default_gripper_bar;                                   // True if the gripper_bar_link in the URDF was loaded - False otherwise
    bool default_gripper_fingers;                               // True if the gripper fingers in the URDF were loaded - False otherwise


    /// @brief Initializes the port to talk to the Dynamixel motors
    void arm_init_port();

    /// @brief Loads a robot-specific yaml file from the config directory into class variables
    /// @details - The yaml file contains default values needed to set the motor registers properly
    /// The function is also able to dynamically figure out:
    ///   - the number of joints that need to be commanded and their Dynamixel IDs (it disregards 'shadow' motors and the gripper)
    ///   - the number of joints that need to be read from and their Dynamixel IDs (it disregards 'shadow' motors, and includes the gripper if present)
    ///   - which motor corresponds to the gripper and its Dynamixel ID (if present)
    /// As a result, any Dynamixel based robot arm can be configured to work with this node as long as the motor config file has the correct structure. It
    /// also eliminates the need to keep track of these parameters in the config file.
    void arm_get_motor_configs(void);

    /// @brief Pings all motors to make sure each servo can be found
    void arm_ping_motors(void);

    /// @brief Writes motor configs to the Dynamixel registers
    void arm_load_motor_configs(void);

    /// @brief Creates a class dictionary containing info on specific registers
    /// @details - Info includes a register's name, address, and data length
    bool arm_init_controlItems(void);

    /// @brief Creates SyncWrite and SyncRead Handlers to write/read data
    /// @details - This allows an array of values to be written to (or read from) many motors at the same time
    bool arm_init_SDK_handlers(void);

    /// @brief Sets the initial operating modes for the arm and gripper
    /// @details - Operating modes include "position", "velocity", "current", "pwm", etc...
    void arm_init_operating_modes(void);

    /// @brief Initialize gripper-related parameters
    /// @details - Used to set correct dimensions for the slider-crank mechanism that operates the gripper
    void arm_init_gripper(void);

    /// @brief Initialize ROS Publishers
    void arm_init_publishers(void);

    /// @brief Initialize ROS Subscribers
    void arm_init_subscribers(void);

    /// @brief Initialize ROS Services
    void arm_init_services(void);

    /// @brief Initialize ROS Timers
    void arm_init_timers(void);

    /// @brief Initialize ROS Action Servers if MoveIt is being used
    void arm_init_action_servers(void);

    /// @brief ROS Subscriber callback function to write any type of joint commands
    /// @param msg - custom message that accepts a vector of position [rad], velocity [rad/s], current [mA], or pwm commands
    void arm_write_joint_commands(const interbotix_sdk::JointCommands &msg);

    /// @brief ROS Subscriber callback function to write any type of gripper command
    /// @param msg - accepts either an angular position [rad], linear position [m], velocity [rad/s], current [mA], or pwm command
    void arm_write_gripper_command(const std_msgs::Float64 &msg);

    /// @brief ROS Subscriber callback function to a user-provided joint trajectory for the arm (excludes gripper)
    /// @param msg - user-provided joint trajectory using the trajectory_msgs::JointTrajectory message type
    void arm_joint_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);

    /// @brief ROS Subscriber callback function to a user-provided joint trajectory for the gripper only
    /// @param msg - user-provided joint trajectory using the trajectory_msgs::JointTrajectory message type
    /// @details - Commands should only be for the 'left_finger' joint and must specify half the desired distance between the fingers
    void arm_gripper_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);

    /// @brief ROS Service to torque on the arm motors
    /// @param req - Empty message
    /// @param res [out] - Empty message
    bool arm_torque_arm_on(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// @brief ROS Service to torque off the arm motors
    /// @param req - Empty message
    /// @param res [out] - Empty message
    bool arm_torque_arm_off(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// @brief ROS Service that allows the user to get information about the robot
    /// @param req - custom message of type 'RobotInfo'. Look at the service message for details
    /// @param res [out] - all types of robot info!!!!!
    bool arm_get_robot_info(interbotix_sdk::RobotInfo::Request &req, interbotix_sdk::RobotInfo::Response &res);

    /// @brief ROS Service that allows the user to change operating modes (position, velocity, current, pwm) and set profiles
    /// @param req - custom message of type 'OperatingModes'. Look at the service message for details
    /// @param res [out] - no message is returned
    bool arm_set_operating_modes(interbotix_sdk::OperatingModes::Request &req, interbotix_sdk::OperatingModes::Response &res);

    /// @brief ROS Service that allows the user to set the Position, Velocity, and Feedforward gains used in the motor firmware
    /// @param req - custom message of type 'FirmwareGains'. Look at the service message for details
    /// @param res [out] - no message is returned
    bool arm_set_firmware_pid_gains(interbotix_sdk::FirmwareGains::Request &req, interbotix_sdk::FirmwareGains::Response &res);

    /// @brief ROS Service that allows the user to change a specific register to a specific value for multiple motors
    /// @param req - custom message of type 'RegisterValues'. Look at the service message for details
    /// @param res [out] - empty vector of type int32[]
    bool arm_set_firmware_register_values(interbotix_sdk::RegisterValues::Request &req, interbotix_sdk::RegisterValues::Response &res);

    /// @brief ROS Service that allows the user to read a specific register on multiple motors
    /// @param req - custom message of type 'RegisterValues'. Look at the service message for details
    /// @param res [out] - vector of raw register values
    bool arm_get_firmware_register_values(interbotix_sdk::RegisterValues::Request &req, interbotix_sdk::RegisterValues::Response &res);

    /// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
    void arm_update_joint_states(const ros::TimerEvent &e);

    /// @brief ROS Timer that executes a joint trajectory for the arm (excludes gripper)
    /// @details - Can follow either position or velocity trajectories and uses the 'time_from_start' field
    void arm_execute_joint_trajectory(const ros::TimerEvent&);

    /// @brief ROS Timer that executes a joint trajectory for the gripper only
    /// @details - Can follow position trajectories only and uses the 'time_from_start' field
    void arm_execute_gripper_trajectory(const ros::TimerEvent&);

    /// @brief ROS Action server used to receive joint trajectories from MoveIt
    /// @param goal - MoveIt specific trajectory message
    void arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    /// @brief ROS Action server used to receive the gripper trajectory from MoveIt
    /// @param goal - MoveIt specific trajectory message
    void arm_gripper_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    /// @brief Sets the operating mode for the arm specifc motors (excluding gripper)
    /// @param arm_mode - specified mode like "position", "velocity", "current", and "pwm"
    /// @param profile_vel - register value that sets the max velocity for the motors if using the "position" operating mode
    /// @param profile_accel - register value that sets the max acceleration for the motors if using the "position" operating mode
    bool arm_set_joint_operating_mode(std::string arm_mode, const int32_t profile_vel = JOINT_PROFILE_VELOCITY, const int32_t profile_accel = JOINT_PROFILE_ACCELERATION);

    /// @brief Sets the operating mode for the gripper motor only
    /// @param gripper_mode - specified mode like "position", "ext_position", "velocity", "current", and "pwm"
    /// @param profile_vel - register value that sets the max velocity for the motor if using the "position" or "ext_position" operating modes
    /// @param profile_accel - register value that sets the max acceleration for the motor if using the "position" or "ext_position" operating modes
    bool arm_set_gripper_operating_mode(std::string gripper_mode, const int32_t profile_vel = JOINT_PROFILE_VELOCITY, const int32_t profile_accel = JOINT_PROFILE_ACCELERATION);

    /// @brief Calculates the linear distance in meters between the gripper fingers given a servo angle
    /// @param horn_angle - gripper angle in radians
    double arm_calculate_gripper_linear_position(const double horn_angle);

    /// @brief Calculates the angular position in radians of the gripper servo given a linear distance between the gripper fingers
    /// @param dist - distance in meters between the gripper fingers
    float arm_calculate_gripper_angular_position(const float dist);

    /// @brief Helper function to check if a function executed successfully
    /// @param result - boolean returned from a function that is True if it executed correctly
    /// @param log - error message to be output if the function did not run correctly
    void arm_check_error(bool result, const char** log);
};

#endif
