#include "interbotix_sdk/arm_obj.h"

/// @brief Constructor for the RobotArm
RobotArm::RobotArm(ros::NodeHandle *node_handle, const std::string robot_name, const std::string robot_model, const double timer_hz)
    : node(*node_handle), robot_name(robot_name), robot_model(robot_model), timer_hz(timer_hz)
{
  arm_init_port();
  arm_get_motor_configs();
  arm_ping_motors();
  arm_load_motor_configs();
  arm_init_controlItems();
  arm_init_SDK_handlers();
  arm_init_operating_modes();
  arm_init_gripper();
  arm_init_pid_controllers();
  arm_init_publishers();
  arm_init_subscribers();
  arm_init_services();
  arm_init_timers();
  arm_init_action_servers();
  ROS_INFO("All motors ready to go!");
}

/// @brief Destructor for the RobotArm
RobotArm::~RobotArm()
{
  // Release dynamically allocated memory
  delete [] joint_ids_read;
  delete [] joint_ids_write;
  bool use_moveit;
  ros::param::param<bool>("~use_moveit", use_moveit, false);
  if (use_moveit)
  {
    delete joint_action_server;
    delete gripper_action_server;
  }
}

/// @brief Set operating mode for the arm joints to position [rad] control
/// @param joint_profile_vel - max angular velocity that a motor is allowed to reach - uses register values
/// @param joint_profile_accel - max acceleration that a motor is allowed to reach - uses register values
void RobotArm::arm_set_joints_to_position_control(const int32_t joint_profile_vel, const int32_t joint_profile_accel)
{
  bool result = false;
  const char* log = NULL;

  for (auto const& motor:arm_motors)
  {
    if (motor.name != "gripper")
    {
      result = dxl_wb.jointMode(motor.motor_id, joint_profile_vel, joint_profile_accel, &log);
      arm_check_error(result, &log);
    }
  }
  arm_operating_mode = State::POSITION;
  ROS_INFO("Arm joints set to position control with a profile velocity of %d and a profile acceleration of %d.", joint_profile_vel, joint_profile_accel);
}

/// @brief Set operating mode for the arm joints to velocity [rad/s] control
/// @param wheel_profile_accel - max acceleration that a motor is allowed to reach - uses register values
void RobotArm::arm_set_joints_to_velocity_control(const int32_t wheel_profile_accel)
{
  bool result = false;
  const char* log = NULL;

  for (auto const& motor:arm_motors)
  {
    if (motor.name != "gripper")
    {
      result = dxl_wb.wheelMode(motor.motor_id, WHEEL_PROFILE_ACCELERATION, &log);
      arm_check_error(result, &log);
    }
  }
  arm_operating_mode = State::VELOCITY;
  ROS_INFO("Arm joints set to velocity control with a profile acceleration of %d.", wheel_profile_accel);
}

/// @brief Set operating mode for the arm joints to current [mA] control (Viper robots only)
void RobotArm::arm_set_joints_to_current_control(void)
{
  bool result = false;
  const char* log = NULL;

  for (auto const& motor:arm_motors)
  {
    if (motor.name != "gripper")
    {
      dxl_wb.torqueOff(motor.motor_id);
      result = dxl_wb.setCurrentControlMode(motor.motor_id, &log);
      dxl_wb.torqueOn(motor.motor_id);
      arm_check_error(result, &log);
    }
  }
  arm_operating_mode = State::CURRENT;
  ROS_INFO("Arm joints set to current control.");
}

/// @brief Set operating mode for the arm joints to pwm control
void RobotArm::arm_set_joints_to_pwm_control(void)
{
  bool result = false;
  const char* log = NULL;

  for (auto const& motor:arm_motors)
  {
    if (motor.name != "gripper")
    {
      dxl_wb.torqueOff(motor.motor_id);
      const char* model_name = dxl_wb.getModelName(motor.motor_id, &log);
      // For some reason, set 'setPWMControlMode' function does not work properly on XL430 servos.
      // To get around this, just write to the register directly using the 'itemWrite' function.
      if (strncmp(model_name, "XL430", strlen("XL430")) == 0)
        result = dxl_wb.itemWrite(motor.motor_id, "Operating_Mode", PWM_CONTROL_MODE, &log);
      else
        result = dxl_wb.setPWMControlMode(motor.motor_id, &log);
      arm_check_error(result, &log);
      dxl_wb.torqueOn(motor.motor_id);
    }
  }
  arm_operating_mode = State::PWM;
  ROS_INFO("Arm joints set to pwm control.");
}

/// @brief Command joint positions
/// @param joint_positions - array of joint positions [rad] to write to the motors; sequence of positions match the joint name order in the published joint_state messages
void RobotArm::arm_set_joint_positions(const double joint_positions[])
{
  if(arm_operating_mode != State::POSITION)
  {
    ROS_ERROR("Incorrect control mode. Please set to position control mode.");
    return;
  }

  int32_t dynamixel_position[joint_num_write];

  for (size_t i{0}; i < joint_num_write; i++)
    dynamixel_position[i] = dxl_wb.convertRadian2Value(joint_ids_write[i], joint_positions[i]);

  const char* log = NULL;
  bool result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, joint_ids_write, joint_num_write, dynamixel_position, 1, &log);
  arm_check_error(result, &log);
}

/// @brief Command joint velocities
/// @param joint_velocities - array of joint velocities [rad/s] to write to the motors; sequence of velocities match the joint name order in the published joint_state messages
void RobotArm::arm_set_joint_velocities(const double joint_velocities[])
{
  if(arm_operating_mode != State::VELOCITY)
  {
    ROS_ERROR("Incorrect control mode. Please set to velocity control mode.");
    return;
  }

  int32_t dynamixel_velocity[joint_num_write];

  for (size_t i{0}; i < joint_num_write; i++)
    dynamixel_velocity[i] = dxl_wb.convertVelocity2Value(joint_ids_write[i], joint_velocities[i]);

  const char* log = NULL;
  bool result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, joint_ids_write, joint_num_write, dynamixel_velocity, 1, &log);
  arm_check_error(result, &log);
}

/// @brief Command joint currents (Viper robots only)
/// @param joint_currents - array of joint currents to write to the motors; sequence of currents match the joint name order in the published joint_state messages
void RobotArm::arm_set_joint_currents(const double joint_currents[])
{
  if(arm_operating_mode != State::CURRENT)
  {
    ROS_ERROR("Incorrect control mode. Please set to current control mode.");
    return;
  }

  int32_t dynamixel_current[joint_num_write];

  for (size_t i{0}; i < joint_num_write; i++)
    dynamixel_current[i] = dxl_wb.convertCurrent2Value(joint_currents[i]);

  const char* log = NULL;
  bool result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, joint_ids_write, joint_num_write, dynamixel_current, 1, &log);
  arm_check_error(result, &log);
}

/// @brief Command joint pwms
/// @param joint_pwms - array of joint pwms to write to the motors; sequence of pwms match the joint name order in the published joint_state messages
void RobotArm::arm_set_joint_pwms(int32_t joint_pwms[])
{
  if(arm_operating_mode != State::PWM)
  {
    ROS_ERROR("Incorrect control mode. Please set to pwm control mode.");
    return;
  }

  const char* log = NULL;
  bool result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_PWM, joint_ids_write, joint_num_write, joint_pwms, 1, &log);
  arm_check_error(result, &log);
}

/// @brief Set operating mode for the specified joint to position [rad] control
/// @param joint_name - name of the joint for which the operating mode will be changed
/// @param joint_profile_vel - max angular velocity that the specified joint is allowed to reach - uses register values
/// @param joint_profile_accel - max acceleration that the specified joint is allowed to reach - uses register values
void RobotArm::arm_set_single_joint_to_position_control(const std::string joint_name, const int32_t joint_profile_vel, const int32_t joint_profile_accel)
{
  const char* log = NULL;
  for (auto const& id : shadow_map[joint_map[joint_name].motor_id])
  {
    bool result = dxl_wb.jointMode(id, joint_profile_vel, joint_profile_accel, &log);
    arm_check_error(result, &log);
  }
  joint_map[joint_name].mode = State::POSITION;
  ROS_INFO("%s set to position control with a profile velocity of %d and a profile acceleration of %d.", joint_name.c_str(), joint_profile_vel, joint_profile_accel);
}

/// @brief Set operating mode for the specified joint to ext_position [rad] control (servo can perform multiple turns instead of only one revolution)
/// @param joint_name - name of the joint for which the operating mode will be changed
/// @param joint_profile_vel - max angular velocity that the specified joint is allowed to reach - uses register values
/// @param joint_profile_accel - max acceleration that the specified joint is allowed to reach - uses register values
/// @details - this mode should only be used with a joint that has the ability to freely rotate
void RobotArm::arm_set_single_joint_to_ext_position_control(const std::string joint_name, const int32_t joint_profile_vel, const int32_t joint_profile_accel)
{
  const char* log = NULL;
  for (auto const& id : shadow_map[joint_map[joint_name].motor_id])
  {
    dxl_wb.torqueOff(id);
    bool result = dxl_wb.setExtendedPositionControlMode(id, &log);
    arm_check_error(result, &log);
    dxl_wb.itemWrite(id, "Profile_Velocity", joint_profile_vel, &log);
    dxl_wb.itemWrite(id, "Profile_Acceleration", joint_profile_accel, &log);
    dxl_wb.torqueOn(id);
  }
  joint_map[joint_name].mode = State::EXT_POSITION;
  ROS_INFO("%s set to extended position control with a profile velocity of %d and a profile acceleration of %d.", joint_name.c_str(), joint_profile_vel, joint_profile_accel);
}

/// @brief Set operating mode for the specified joint to velocity [rad/s] control
/// @param joint_name - name of the joint for which the operating mode will be changed
/// @param wheel_profile_accel - max acceleration that the specified joint is allowed to reach - uses register values
void RobotArm::arm_set_single_joint_to_velocity_control(const std::string joint_name, const int32_t wheel_profile_accel)
{
  const char* log = NULL;
  for (auto const& id : shadow_map[joint_map[joint_name].motor_id])
  {
    bool result = dxl_wb.wheelMode(id, wheel_profile_accel, &log);
    arm_check_error(result, &log);
  }
  joint_map[joint_name].mode = State::VELOCITY;
  ROS_INFO("%s set to velocity control with a profile acceleration of %d.", joint_name.c_str(), wheel_profile_accel);
}

/// @brief Set operating mode for the specified joint to current [mA] control
/// @param joint_name - name of the joint for which the operating mode will be changed
void RobotArm::arm_set_single_joint_to_current_control(const std::string joint_name)
{
  const char* log = NULL;
  for (auto const& id : shadow_map[joint_map[joint_name].motor_id])
  {
    dxl_wb.torqueOff(joint_map[joint_name].motor_id);
    bool result = dxl_wb.setCurrentControlMode(id, &log);
    arm_check_error(result, &log);
    dxl_wb.torqueOn(id);
  }
  joint_map[joint_name].mode = State::CURRENT;
  ROS_INFO("%s set to current control.", joint_name.c_str());
}

/// @brief Set operating mode for the specified joint to pwm control
/// @param joint_name - name of the joint for which the operating mode will be changed
void RobotArm::arm_set_single_joint_to_pwm_control(const std::string joint_name)
{
  const char* log = NULL;
  bool result = false;
  for (auto const& id : shadow_map[joint_map[joint_name].motor_id])
  {
    dxl_wb.torqueOff(joint_map[joint_name].motor_id);
    const char* model_name = dxl_wb.getModelName(id, &log);
    // For some reason, set 'setPWMControlMode' function does not work properly on XL430 servos.
    // To get around this, just write to the register directly using the 'itemWrite' function.
    if (strncmp(model_name, "XL430", strlen("XL430")) == 0)
      result = dxl_wb.itemWrite(id, "Operating_Mode", PWM_CONTROL_MODE, &log);
    else
      result = dxl_wb.setPWMControlMode(id, &log);
    arm_check_error(result, &log);
    dxl_wb.torqueOn(id);
  }
  joint_map[joint_name].mode = State::PWM;
  ROS_INFO("%s set to pwm control.", joint_name.c_str());
}

/// @brief Set gripper finger position
/// @param dist - desired distance [m] between the gripper fingers
void RobotArm::arm_set_gripper_linear_position(const float dist)
{
  float horn_angle = arm_calculate_gripper_angular_position(dist);
  arm_set_single_joint_angular_position("gripper", horn_angle);
}

/// @brief Set the angular position for the specified joint
/// @param joint_name - name of the joint to command
/// @param horn_angle - desired angular position [rad]
void RobotArm::arm_set_single_joint_angular_position(const std::string joint_name, const float horn_angle)
{
  const char* log = NULL;
  bool result = dxl_wb.goalPosition(joint_map[joint_name].motor_id, horn_angle, &log);
  arm_check_error(result, &log);
}

/// @brief Set the velocity for the specified joint
/// @param joint_name - name of the joint to command
/// @param vel - desired angular velocity [rad/s]
void RobotArm::arm_set_single_joint_velocity(const std::string joint_name, const float vel)
{
  const char* log = NULL;
  bool result = dxl_wb.goalVelocity(joint_map[joint_name].motor_id, vel, &log);
  arm_check_error(result, &log);
}

/// @brief Set the current for the specified joint
/// @param joint_name - name of the joint to command
/// @param current - desired current [mA]
void RobotArm::arm_set_single_joint_current(const std::string joint_name, const float current)
{
  const char* log = NULL;
  int16_t current_val = dxl_wb.convertCurrent2Value(current);
  bool result = dxl_wb.itemWrite(joint_map[joint_name].motor_id, "Goal_Current", current_val, &log);
  arm_check_error(result, &log);
}

/// @brief Set the pwm for the specified joint
/// @param joint_name - name of the joint to command
/// @param pwm - desired pwm - values between 150-350 (or the negative equiavlent) seem to work best
void RobotArm::arm_set_single_joint_pwm(const std::string joint_name, int32_t pwm)
{
  const char* log = NULL;
  bool result = dxl_wb.itemWrite(joint_map[joint_name].motor_id, "Goal_PWM", pwm, &log);
  arm_check_error(result, &log);
}

/// @brief Set max gripper effort
/// @param max_effort - the max effort [mA] that the gripper Action server will allow before preempting
void RobotArm::arm_set_gripper_max_effort(const double max_effort)
{
  gripper_max_effort = max_effort;
}

/// @brief Torque on all motors (including the gripper)
void RobotArm::arm_torque_on(void)
{
  for (auto const& joint:all_joints)
    dxl_wb.torqueOn(joint.motor_id);
  ROS_INFO("Robot torqued on!");
}

/// @brief Torque off all motors (including the gripper)
void RobotArm::arm_torque_off(void)
{
  for (auto const& joint:all_joints)
    dxl_wb.torqueOff(joint.motor_id);
  ROS_INFO("Robot torqued off!");
}

/// @brief Initializes the port to talk to the Dynamixel motors
void RobotArm::arm_init_port(void)
{
  std::string port;
  ros::param::param<std::string>("~port", port, PORT);

  const char* log;
  bool result = dxl_wb.init(port.c_str(), BAUDRATE, &log);
  if (!result)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to open port at %s.", port.c_str());
  }
}

/// @brief Loads a robot-specific yaml file from the config directory into class variables
/// @details - The yaml file contains default values needed to set the motor registers properly
/// The function is also able to dynamically figure out:
///   - the number of joints that need to be commanded and their Dynamixel IDs (it disregards 'shadow' motors and the gripper)
///   - the number of joints that need to be read from and their Dynamixel IDs (it disregards 'shadow' motors, and includes the gripper if present)
///   - which motor corresponds to the gripper and its Dynamixel ID (if present)
/// As a result, any Dynamixel based robot arm can be configured to work with this node as long as the motor config file has the correct structure. It
/// also eliminates the need to keep track of these parameters in the config file.
void RobotArm::arm_get_motor_configs(void)
{
  std::string yaml_file;
  ros::param::get("~motor_configs", yaml_file);
  yaml_file += robot_model + ".yaml";
  YAML::Node dxl_config = YAML::LoadFile(yaml_file.c_str());
  if (dxl_config.IsNull())
  {
    ROS_ERROR("Config file not found.");
    return;
  }

  // Define the home and sleep positions for the arm joints
  YAML::Node sleep_node = dxl_config["sleep"];
  for (auto const& value: sleep_node)
  {
    sleep_positions.push_back(value.as<double>());
    home_positions.push_back(0);
  }

  // Get the actual motor configs
  YAML::Node order_node = dxl_config["order"];
  YAML::Node singles_node = dxl_config["singles"];
  std::vector <YAML::Node> nodes {order_node, singles_node};
  joint_num_write = 0;
  use_arm = false;
  use_gripper = false;

  if (order_node.size() > 0)
    use_arm = true;

  // Read in each node in the yaml file according to the 'order' sequence. This will make it easier
  // to keep track of where each motor is located in an array. This first loop is mainly used to get
  // a vector of all the motors and their corresponding Dynamixel IDs, as well as determine the number
  // of joints that need to be commanded.
  for (auto const& node: nodes)
  {
    for(size_t i{0}; i < node.size(); i++)
    {
      std::string name = node[i].as<std::string>();
      YAML::Node item = dxl_config[name];
      int32_t id = item["ID"].as<int32_t>();
      int32_t secondary_id = item["Secondary_ID"].as<int32_t>();
      Motor motor = {name, (uint8_t) id};
      std::vector<uint8_t> shadow_list {(uint8_t) id};
      all_motors.push_back(motor);
      motor_map.insert(std::pair<std::string, uint8_t>(name, id));
      shadow_map.insert(std::pair<uint8_t, std::vector<uint8_t>>(id, shadow_list));
      // Determine if a motor is a shadow of another by checking its secondary 'shadow' ID.
      // Only increment 'joint_num_write' if the motor's secondary_id register is disabled (set to 255)
      if (node == dxl_config["order"] && name != "gripper" && secondary_id == 255)
        joint_num_write++;
      if (node == dxl_config["order"] && name != "gripper")
        arm_motors.push_back(motor);
      if (node == dxl_config["order"] && name == "gripper")
        use_gripper = true;
    }
  }

  // Now that the number of joints that need to be commanded has been figured out, create an array of
  // that size to hold the motor ids. Unfortunately, vectors cannot be used since the Robotis provided function
  // that actually commands the motors ('syncWrite') does not take them as input.
  joint_ids_write = new uint8_t[joint_num_write];
  size_t cntr = 0;

  // Check to see if the Drive_Mode register should be set to 'Time Based Profile' instead of 'Velocity Based Profile'
  bool use_time_based_profile;
  ros::param::get("~use_time_based_profile", use_time_based_profile);

  // For each node in the yaml file, read in the register names and the values that should be written to them.
  for (auto const& node: nodes)
  {
    for (size_t i{0}; i < node.size(); i++)
    {
      std::string name = node[i].as<std::string>();
      YAML::Node item = dxl_config[name];
      int32_t id = item["ID"].as<int32_t>();
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        int32_t value = it_item->second.as<int32_t>();

        if (item_name == "ID")
          continue;
        else if (item_name == "Secondary_ID" && value == 255)
        {
          Motor joint = {name, (uint8_t) id};
          JointMode jm = {(uint8_t) id, State::NONE};
          all_joints.push_back(joint);
          joint_map.insert(std::pair<std::string, JointMode>(name, jm));
          if (node == dxl_config["order"])
          {
            arm_joints.push_back(joint);
            if (name != "gripper")
            {
              joint_ids_write[cntr] = (uint8_t) id;
              cntr++;
            }
          }
        }
        else if (item_name == "Secondary_ID" && value != 255)
          shadow_map[value].push_back(id);
        else if (item_name == "Drive_Mode" && use_time_based_profile == true)
          value += 4;
        Info info = {(uint8_t) id, item_name, value};
        dynamixel_info.push_back(info);
      }
    }
  }
  // Dynamically create an array to hold the IDs of the motors that will be used when reading joint states
  joint_ids_read = new uint8_t[all_joints.size()];
  for (size_t i{0}; i < all_joints.size(); i++)
    joint_ids_read[i] = all_joints.at(i).motor_id;
}

/// @brief Pings all motors to make sure each servo can be found
void RobotArm::arm_ping_motors(void)
{
  const char* log;

  for (auto const& motor:all_motors)
  {
    uint16_t model_number = 0;
    bool result = dxl_wb.ping(motor.motor_id, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", motor.motor_id);
    }
    else
    {
      ROS_INFO("ID : %d, Model Number : %d", motor.motor_id, model_number);
    }
  }
}

/// @brief Writes motor configs to the Dynamixel registers
void RobotArm::arm_load_motor_configs(void)
{
  const char* log;
  arm_torque_off();

  for (auto const& info:dynamixel_info)
  {
    if (info.item_name != "ID" && info.item_name != "Baud_Rate")
    {
      bool result = dxl_wb.itemWrite(info.motor_id, info.item_name.c_str(), info.value, &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write value[%d] on items[%s] to [ID : %d]", info.value, info.item_name.c_str(), info.motor_id);
        return;
      }
    }
  }
  arm_torque_on();
}

/// @brief Creates a class dictionary containing info on specific registers
/// @details - Info includes a register's name, address, and data length
bool RobotArm::arm_init_controlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = all_joints.begin();

  const ControlItem *goal_position = dxl_wb.getItemInfo(it->motor_id, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb.getItemInfo(it->motor_id, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb.getItemInfo(it->motor_id, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *goal_current = dxl_wb.getItemInfo(it->motor_id, "Goal_Current");
  // only return an error if the motor is not an XL430; otherwise, as XL430s don't have this register, just skip it
  const char* model_name = dxl_wb.getModelName(it->motor_id, &log);
  if (goal_current == NULL && strncmp(model_name, "XM", 2) == 0) return false;

  const ControlItem *goal_pwm = dxl_wb.getItemInfo(it->motor_id, "Goal_PWM");
  if (goal_pwm == NULL) return false;

  const ControlItem *present_position = dxl_wb.getItemInfo(it->motor_id, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb.getItemInfo(it->motor_id, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb.getItemInfo(it->motor_id, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb.getItemInfo(it->motor_id, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb.getItemInfo(it->motor_id, "Present_Load");
  if (present_current == NULL) return false;

  control_items["Goal_Position"] = goal_position;
  control_items["Goal_Velocity"] = goal_velocity;
  control_items["Goal_Current"] = goal_current;
  control_items["Goal_PWM"] = goal_pwm;

  control_items["Present_Position"] = present_position;
  control_items["Present_Velocity"] = present_velocity;
  control_items["Present_Current"] = present_current;

  return true;
}

/// @brief Creates SyncWrite and SyncRead Handlers to write/read data
/// @details - This allows an array of values to be written to (or read from) many motors at the same time
bool RobotArm::arm_init_SDK_handlers(void)
{
  bool result = false;
  const char* log = NULL;

  result = dxl_wb.addSyncWriteHandler(control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb.addSyncWriteHandler(control_items["Goal_Velocity"]->address, control_items["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  // only add a SyncWriteHandler for 'Goal_Current' if the register actually exists!
  if (control_items["Goal_Current"] != NULL)
  {
    result = dxl_wb.addSyncWriteHandler(control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
    else
    {
      ROS_INFO("%s", log);
    }
  }

  result = dxl_wb.addSyncWriteHandler(control_items["Goal_PWM"]->address, control_items["Goal_PWM"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items["Present_Position"]->address, control_items["Present_Current"]->address);
    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
    // uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length;
    uint16_t read_length = control_items["Present_Position"]->data_length + control_items["Present_Velocity"]->data_length + control_items["Present_Current"]->data_length+2;

    result = dxl_wb.addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }
  return result;
}

/// @brief Sets the initial operating modes for the arm and gripper
/// @details - Operating modes include "position", "velocity", "current", "pwm", etc...
void RobotArm::arm_init_operating_modes(void)
{
  arm_operating_mode = State::NONE;
  joint_map["gripper"].mode = State::NONE;
  std::string arm_mode, gripper_mode;
  int arm_profile_vel, arm_profile_accel;
  int gripper_profile_vel, gripper_profile_accel;
  ros::param::param<std::string>("~arm_operating_mode", arm_mode, "none");
  ros::param::param<int>("~arm_profile_velocity", arm_profile_vel, 0);
  ros::param::param<int>("~arm_profile_acceleration", arm_profile_accel, 0);
  ros::param::param<std::string>("~gripper_operating_mode", gripper_mode, "none");
  ros::param::param<int>("~gripper_profile_velocity", gripper_profile_vel, 0);
  ros::param::param<int>("~gripper_profile_acceleration", gripper_profile_accel, 0);
  if (use_arm)
    arm_set_joint_operating_mode(arm_mode, arm_profile_vel, arm_profile_accel);
  else
    arm_set_joint_operating_mode("none");

  if (use_gripper)
    arm_set_single_joint_operating_mode("gripper", gripper_mode, gripper_profile_vel, gripper_profile_accel);
  else
    arm_set_single_joint_operating_mode("gripper", "none");
}

/// @brief Initialize gripper-related parameters
/// @details - Used to set correct dimensions for the slider-crank mechanism that operates the gripper
void RobotArm::arm_init_gripper(void)
{
  ros::param::param<bool>("~use_default_gripper_bar", use_default_gripper_bar, false);
  ros::param::param<bool>("~use_default_gripper_fingers", use_default_gripper_fingers, false);

  if (robot_model.find("300") != std::string::npos)
  {
    horn_radius = HORN_RADIUS_300;
    arm_radius = ARM_RADIUS_300;
  }
  else
  {
    horn_radius = HORN_RADIUS;
    arm_radius = ARM_RADIUS;
  }
  arm_set_gripper_max_effort();
}

/// @brief Initialize pid controllers
/// @details - Used to better track goal positions when doing trajectory control
void RobotArm::arm_init_pid_controllers(void)
{
  ros::param::param<bool>("~use_pid_cntlrs", use_pid_cntlrs, false);
  if (!use_arm || !use_pid_cntlrs)
    return;

  std::vector<double> Kp_vec, Ki_vec, Kd_vec, umin_vec, umax_vec;
  ros::param::get("~Kp_vec", Kp_vec);
  ros::param::get("~Ki_vec", Ki_vec);
  ros::param::get("~Kd_vec", Kd_vec);
  ros::param::get("~umin_vec", umin_vec);
  ros::param::get("~umax_vec", umax_vec);

  pid_cntlrs.multi_pid_init(joint_num_write, Kp_vec, Ki_vec, Kd_vec, umin_vec, umax_vec);
  ROS_INFO("Successfully initialized the PID Controllers.");
}

/// @brief Initialize ROS Publishers
void RobotArm::arm_init_publishers(void)
{
  pub_joint_states = node.advertise<sensor_msgs::JointState>("joint_states", 100);
}

/// @brief Initialize ROS Subscribers
void RobotArm::arm_init_subscribers(void)
{
  sub_single_joint_command = node.subscribe("single_joint/command", 100, &RobotArm::arm_write_single_joint_command, this);
  if (use_arm)
  {
    sub_joint_commands = node.subscribe("joint/commands", 100, &RobotArm::arm_write_joint_commands, this);
    sub_joint_traj_msg = node.subscribe("arm_controller/joint_trajectory", 100, &RobotArm::arm_joint_trajectory_msg_callback, this);
  }
  if (use_gripper)
  {
    sub_gripper_command = node.subscribe("gripper/command", 100, &RobotArm::arm_write_gripper_command, this);
    sub_gripper_traj_msg = node.subscribe("gripper_controller/gripper_trajectory", 100, &RobotArm::arm_gripper_trajectory_msg_callback, this);
  }
}

/// @brief Initialize ROS Services
void RobotArm::arm_init_services(void)
{
  srv_torque_on = node.advertiseService("torque_joints_on", &RobotArm::arm_torque_joints_on, this);
  srv_torque_off = node.advertiseService("torque_joints_off", &RobotArm::arm_torque_joints_off, this);
  srv_get_robot_info = node.advertiseService("get_robot_info", &RobotArm::arm_get_robot_info, this);
  srv_operating_mode = node.advertiseService("set_operating_modes", &RobotArm::arm_set_operating_modes, this);
  srv_firmware_gains = node.advertiseService("set_firmware_pid_gains", &RobotArm::arm_set_firmware_pid_gains, this);
  srv_set_register = node.advertiseService("set_motor_register_values", &RobotArm::arm_set_firmware_register_values, this);
  srv_get_register = node.advertiseService("get_motor_register_values", &RobotArm::arm_get_firmware_register_values, this);
}

/// @brief Initialize ROS Timers
void RobotArm::arm_init_timers(void)
{
  execute_joint_traj = false;
  execute_gripper_traj = false;
  tmr_joint_states = node.createTimer(ros::Duration(1/timer_hz), &RobotArm::arm_update_joint_states, this);
  if (use_arm)
    tmr_joint_traj = node.createTimer(ros::Duration(1/timer_hz), &RobotArm::arm_execute_joint_trajectory, this);
  if (use_gripper)
    tmr_gripper_traj = node.createTimer(ros::Duration(1/timer_hz), &RobotArm::arm_execute_gripper_trajectory, this);
}

/// @brief Initialize ROS Action Servers if MoveIt is being used
void RobotArm::arm_init_action_servers(void)
{
  bool use_moveit;
  ros::param::param<bool>("~use_moveit", use_moveit, false);
  if (use_moveit)
  {
    joint_action_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                              (node, "arm_controller/follow_joint_trajectory", boost::bind(&RobotArm::arm_joint_trajectory_action_callback, this, _1),false);
    gripper_action_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                              (node, "gripper_controller/follow_joint_trajectory", boost::bind(&RobotArm::arm_gripper_trajectory_action_callback, this, _1),false);
    joint_action_server->start();
    gripper_action_server->start();
  }
}

/// @brief ROS Subscriber callback function to write any type of joint commands
/// @param msg - custom message that accepts a vector of position [rad], velocity [rad/s], or pwm commands
void RobotArm::arm_write_joint_commands(const interbotix_sdk::JointCommands &msg)
{
  switch(arm_operating_mode)
  {
    case State::POSITION:
    {
      double joint_positions[msg.cmd.size()];
      for (size_t i {0}; i < msg.cmd.size(); i++)
        joint_positions[i] = msg.cmd.at(i);
      arm_set_joint_positions(joint_positions);
      break;
    }
    case State::VELOCITY:
    {
      double joint_velocities[msg.cmd.size()];
      for (size_t i {0}; i < msg.cmd.size(); i++)
        joint_velocities[i] = msg.cmd.at(i);
      arm_set_joint_velocities(joint_velocities);
      break;
    }
    case State::CURRENT:
    {
      double joint_currents[msg.cmd.size()];
      for (size_t i {0}; i < msg.cmd.size(); i++)
        joint_currents[i] = msg.cmd.at(i);
      arm_set_joint_currents(joint_currents);
      break;
    }
    case State::PWM:
    {
      int32_t joint_pwms[msg.cmd.size()];
      for (size_t i {0}; i < msg.cmd.size(); i++)
        joint_pwms[i] = (int32_t) msg.cmd.at(i);
      arm_set_joint_pwms(joint_pwms);
      break;
    }
    case State::NONE:
    {
      ROS_WARN("Arm joint control mode not set.");
      break;
    }
    default:
    {
      ROS_ERROR("Invalid joint control mode.");
    }
  }
}

/// @brief ROS Subscriber callback function to write any type of gripper command
/// @param msg - accepts either an angular position [rad], linear position [m], velocity [rad/s], or pwm command
void RobotArm::arm_write_gripper_command(const std_msgs::Float64 &msg)
{
  switch(joint_map["gripper"].mode)
  {
    case State::POSITION:
    {
      arm_set_gripper_linear_position(msg.data);
      break;
    }
    case State::EXT_POSITION:
    {
      arm_set_single_joint_angular_position("gripper", msg.data);
      break;
    }
    case State::VELOCITY:
    {
      arm_set_single_joint_velocity("gripper", msg.data);
      break;
    }
    case State::CURRENT:
    {
      arm_set_single_joint_current("gripper", msg.data);
      break;
    }
    case State::PWM:
    {
      arm_set_single_joint_pwm("gripper", (int32_t) msg.data);
      break;
    }
    case State::NONE:
    {
      ROS_WARN("Gripper control mode not set.");
      break;
    }
    default:
    {
      ROS_ERROR("Invalid gripper control mode.");
    }
  }
}

/// @brief ROS Subscriber callback function to write any type of command to a specified joint
/// @param msg - accepts either an angular position [rad], velocity [rad/s], current [mA], or pwm command
void RobotArm::arm_write_single_joint_command(const interbotix_sdk::SingleCommand &msg)
{
  switch(joint_map[msg.joint_name].mode)
  {
    case State::POSITION:
    case State::EXT_POSITION:
    {
      arm_set_single_joint_angular_position(msg.joint_name, msg.cmd);
      break;
    }
    case State::VELOCITY:
    {
      arm_set_single_joint_velocity(msg.joint_name, msg.cmd);
      break;
    }
    case State::CURRENT:
    {
      arm_set_single_joint_current(msg.joint_name, msg.cmd);
      break;
    }
    case State::PWM:
    {
      arm_set_single_joint_pwm(msg.joint_name, (int32_t) msg.cmd);
      break;
    }
    case State::NONE:
    {
      ROS_WARN("%s control mode not set.", msg.joint_name.c_str());
      break;
    }
    default:
    {
      ROS_ERROR("Invalid %s control mode.", msg.joint_name.c_str());
    }
  }
}

/// @brief ROS Subscriber callback function to a user-provided joint trajectory for the arm (excludes gripper)
/// @param msg - user-provided joint trajectory using the trajectory_msgs::JointTrajectory message type
void RobotArm::arm_joint_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg)
{
  if (execute_joint_traj == false)
  {
    // For some reason, Moveit organizes the joint data in the trajectory message in alphabetical order. Thus,
    // to make it easier to parse through the trajectory message, reorganize the data points in the vectors so that the order
    // matches the order of the joints as they are published in the joint_states topic.
    std::map<std::string, uint8_t> joint_order;
    uint8_t cntr = 0;

    for (auto const& name:msg.joint_names)
    {
      joint_order[name] = cntr;
      cntr++;
    }
    jnt_tra_msg.joint_names.clear();
    jnt_tra_msg.points.clear();
    jnt_tra_msg.header = msg.header;
    for (auto const& joint:arm_joints)
      if (joint.name != "gripper")
        jnt_tra_msg.joint_names.push_back(joint.name);

    cntr = 0;
    size_t pos_size = msg.points.at(0).positions.size();
    size_t vel_size = msg.points.at(0).velocities.size();
    size_t accel_size = msg.points.at(0).accelerations.size();

    while(cntr < msg.points.size())
    {
      trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;
      jnt_tra_point_msg.time_from_start = msg.points.at(cntr).time_from_start;
      for (auto const& joint:arm_joints)
      {
        if (joint.name != "gripper")
        {
          if (pos_size != 0)
            jnt_tra_point_msg.positions.push_back(msg.points.at(cntr).positions.at(joint_order[joint.name]));
          if (vel_size != 0)
            jnt_tra_point_msg.velocities.push_back(msg.points.at(cntr).velocities.at(joint_order[joint.name]));
          if (accel_size != 0)
            jnt_tra_point_msg.accelerations.push_back(msg.points.at(cntr).accelerations.at(joint_order[joint.name]));
        }
      }
      jnt_tra_msg.points.push_back(jnt_tra_point_msg);
      cntr++;
    }
    // Make sure that the initial joint positions in the trajectory match the current
    // joint states (with an arbitrary error less than 0.1 rad)
    size_t itr = 0;
    for (auto const& joint:arm_joints)
    {
      if (joint.name != "gripper")
      {
        if (!(fabs(jnt_tra_msg.points[0].positions.at(itr) - joint_states.position.at(itr)) < 0.1))
        {
          ROS_WARN("%s motor is not at the correct initial state.", joint_states.name.at(itr).c_str());
          ROS_WARN("Expected state: %f, Actual State: %f.", jnt_tra_msg.points.at(0).positions.at(itr), joint_states.position.at(itr));
        }
        itr++;
      }
    }
    ROS_INFO("Succeeded to get joint trajectory!");
    joint_start_time = ros::Time::now().toSec();
    execute_joint_traj = true;
  }
  else
  {
    ROS_WARN("Arm joints are still moving");
  }
}

/// @brief ROS Subscriber callback function to a user-provided joint trajectory for the gripper only
/// @param msg - user-provided joint trajectory using the trajectory_msgs::JointTrajectory message type
/// @details - Commands should only be for the 'left_finger' joint and must specify half the desired distance between the fingers
void RobotArm::arm_gripper_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg)
{
  if (execute_gripper_traj == false)
  {
    gripper_tra_msg.joint_names.clear();
    gripper_tra_msg.points.clear();
    gripper_tra_msg = msg;

    // Make sure that the initial gripper position in the trajectory matches the current
    // gripper position (with an arbitrary error less than 0.1 rad)
    // Note that MoveIt sends the desired position of the 'left_finger_link'. In the joint_states message,
    // this is located right after the gripper position in radians (conveniently, this index matches arm_joints.size())
    if (!(fabs(gripper_tra_msg.points.at(0).positions.at(0) - joint_states.position.at(arm_joints.size())) < 0.1))
    {
      ROS_WARN("Gripper motor is not at the correct initial state.");
      ROS_WARN("Expected state: %f, Actual State: %f.", gripper_tra_msg.points.at(0).positions.at(0), joint_states.position.at(arm_joints.size()));
    }

    ROS_INFO("Succeeded to get gripper trajectory!");
    gripper_start_time = ros::Time::now().toSec();
    execute_gripper_traj = true;
  }
  else
  {
    ROS_WARN("Gripper is still moving");
  }
}

/// @brief ROS Service to torque on all joints
/// @param req - Empty message
/// @param res [out] - Empty message
bool RobotArm::arm_torque_joints_on(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  arm_torque_on();
  return true;
}

/// @brief ROS Service to torque off all joints
/// @param req - Empty message
/// @param res [out] - Empty message
bool RobotArm::arm_torque_joints_off(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  arm_torque_off();
  return true;
}

/// @brief ROS Service that allows the user to get information about the robot
/// @param req - custom message of type 'RobotInfo'. Look at the service message for details
/// @param res [out] - all types of robot info!!!!!
bool RobotArm::arm_get_robot_info(interbotix_sdk::RobotInfo::Request &req, interbotix_sdk::RobotInfo::Response &res)
{
  // Parse the urdf model to get joint position and velocity limits
  if (!ros::param::has("robot_description"))
  {
    ROS_WARN("The %s/robot_description parameter was not found!", robot_name.c_str());
    return false;
  }

  urdf::Model model;
  model.initParam(robot_name + "/robot_description");
  for (auto const& joint:all_joints)
  {
    urdf::JointConstSharedPtr ptr;
    if (joint.name == "gripper" && use_default_gripper_bar && use_default_gripper_fingers)
    {
      ptr = model.getJoint("left_finger");
      res.lower_gripper_limit = ptr->limits->lower;
      res.upper_gripper_limit = ptr->limits->upper;
      res.use_gripper = true;
    }
    ptr = model.getJoint(joint.name);
    res.lower_joint_limits.push_back(ptr->limits->lower);
    res.upper_joint_limits.push_back(ptr->limits->upper);
    res.velocity_limits.push_back(ptr->limits->velocity);
    res.joint_names.push_back(joint.name);
    res.joint_ids.push_back(joint.motor_id);
  }

  res.home_pos = home_positions;
  res.sleep_pos = sleep_positions;
  res.num_joints = joint_num_write;
  res.num_single_joints = all_joints.size();
  return true;
}

/// @brief ROS Service that allows the user to change operating modes (position, velocity, pwm) and set profiles
/// @param req - custom message of type 'OperatingModes'. Look at the service message for details
/// @param res [out] - no message is returned
bool RobotArm::arm_set_operating_modes(interbotix_sdk::OperatingModes::Request &req, interbotix_sdk::OperatingModes::Response &res)
{
  bool result = true;
  if (req.cmd == interbotix_sdk::OperatingModes::Request::GRIPPER || req.cmd == interbotix_sdk::OperatingModes::Request::ARM_JOINTS_AND_GRIPPER)
  {
    if (req.use_custom_profiles)
      result = arm_set_single_joint_operating_mode("gripper", req.mode, req.profile_velocity, req.profile_acceleration);
    else
      result = arm_set_single_joint_operating_mode("gripper", req.mode);
  }

  if (!result)
    return false;

  if (req.cmd == interbotix_sdk::OperatingModes::Request::ARM_JOINTS_AND_GRIPPER || req.cmd == interbotix_sdk::OperatingModes::Request::ARM_JOINTS)
  {
    if (req.use_custom_profiles)
      result = arm_set_joint_operating_mode(req.mode, req.profile_velocity, req.profile_acceleration);
    else
      result = arm_set_joint_operating_mode(req.mode);
  }

  if (!result)
    return false;

  if (req.cmd == interbotix_sdk::OperatingModes::Request::SINGLE_JOINT)
  {
    if (req.use_custom_profiles)
      result = arm_set_single_joint_operating_mode(req.joint_name, req.mode, req.profile_velocity, req.profile_acceleration);
    else
      result = arm_set_single_joint_operating_mode(req.joint_name, req.mode);
  }

  if (!result)
    return false;

  return true;
}

/// @brief ROS Service that allows the user to set the Position, Velocity, and Feedforward gains used in the motor firmware
/// @param req - custom message of type 'FirmwareGains'. Look at the service message for details
/// @param res [out] - no message is returned
bool RobotArm::arm_set_firmware_pid_gains(interbotix_sdk::FirmwareGains::Request &req, interbotix_sdk::FirmwareGains::Response &res)
{
  bool result;
  const char* log;
  std::vector<uint8_t> ids;

  // if joint_id == 0, then modify PID gains for all motors
  if (req.joint_id == 0)
  {
    for (auto const& joint:all_joints)
      ids.push_back(joint.motor_id);
  }
  else
    ids.push_back(req.joint_id);

  // Write the Position P gain to the 'Position_P_Gain' register for all selected motors
  size_t cntr = 0;
  for (auto const& gain: req.Kp_pos)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Position_P_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Position_P_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Position I gain to the 'Position_I_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.Ki_pos)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Position_I_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Position_I_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Position D gain to the 'Position_D_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.Kd_pos)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Position_D_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Position_D_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Feedforward 1st gain to the 'Feedforward_1st_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.K1)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Feedforward_1st_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Feedforward_1st_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Feedforward 2nd gain to the 'Feedforward_2nd_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.K2)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Feedforward_2nd_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Feedforward_2nd_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Velocity P gain to the 'Velocity_P_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.Kp_vel)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Velocity_P_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Velocity_P_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  // Write the Velocity I gain to the 'Velocity_I_Gain' register for all selected motors
  cntr = 0;
  for (auto const& gain: req.Ki_vel)
  {
    result = dxl_wb.itemWrite(ids.at(cntr), "Velocity_I_Gain", gain, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write Velocity_I_Gain value[%d] to Dynamixel[ID : %d]", gain, ids.at(cntr));
      return false;
    }
    cntr++;
  }

  return true;
}

/// @brief ROS Service that allows the user to change a specific register to a specific value for multiple motors
/// @param req - custom message of type 'RegisterValues'. Look at the service message for details
/// @param res [out] - empty vector of type int32[]
bool RobotArm::arm_set_firmware_register_values(interbotix_sdk::RegisterValues::Request &req, interbotix_sdk::RegisterValues::Response &res)
{
  const char* log;
  bool result = false;

  if (req.cmd == interbotix_sdk::RegisterValues::Request::SINGLE_MOTOR || req.cmd == interbotix_sdk::RegisterValues::Request::GRIPPER)
  {
    uint8_t id = motor_map[req.motor_name];
    if (req.cmd == interbotix_sdk::RegisterValues::Request::GRIPPER)
      id = motor_map["gripper"];
    result = dxl_wb.itemWrite(id, req.addr_name.c_str(), req.value, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", req.value, req.addr_name.c_str(), id);
      return false;
    }
  }
  else if (req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS_AND_GRIPPER || req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS)
  {
    for (auto const& joint: arm_joints)
    {
      if (joint.name != "gripper" || joint.name == "gripper" && req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS_AND_GRIPPER)
      {
        result = dxl_wb.itemWrite(joint.motor_id, req.addr_name.c_str(), req.value, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", req.value, req.addr_name.c_str(), joint.motor_id);
          return false;
        }
      }
    }
  }
  return true;
}

/// @brief ROS Service that allows the user to read a specific register on multiple motors
/// @param req - custom message of type 'RegisterValues'. Look at the service message for details
/// @param res [out] - vector of raw register values
bool RobotArm::arm_get_firmware_register_values(interbotix_sdk::RegisterValues::Request &req, interbotix_sdk::RegisterValues::Response &res)
{
  const char* log;
  bool result = false;
  int32_t value;

  if (req.cmd == interbotix_sdk::RegisterValues::Request::SINGLE_MOTOR || req.cmd == interbotix_sdk::RegisterValues::Request::GRIPPER)
  {
    uint8_t id = motor_map[req.motor_name];
    if (req.cmd == interbotix_sdk::RegisterValues::Request::GRIPPER)
      id = motor_map["gripper"];
    result = dxl_wb.itemRead(id, req.addr_name.c_str(), &value, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Failed to read value on items[%s] from Dynamixel[ID : %d]", req.addr_name.c_str(), id);
      return false;
    }
    res.values.push_back(value);
  }
  else if (req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS_AND_GRIPPER || req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS)
  {
    for (auto const& joint: arm_joints)
    {
      if (joint.name != "gripper" || joint.name == "gripper" && req.cmd == interbotix_sdk::RegisterValues::Request::ARM_JOINTS_AND_GRIPPER)
      {
        result = dxl_wb.itemRead(joint.motor_id, req.addr_name.c_str(), &value, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to read value on items[%s] from Dynamixel[ID : %d]", req.addr_name.c_str(), joint.motor_id);
          return false;
        }
        res.values.push_back(value);
      }
    }
  }
  return true;
}

/// @brief ROS Timer that reads current states from all the motors and publishes them to the joint_states topic
void RobotArm::arm_update_joint_states(const ros::TimerEvent &e)
{
  bool result = false;
  const char* log = NULL;

  sensor_msgs::JointState joint_state_msg;

  int32_t get_current[all_joints.size()];
  int32_t get_velocity[all_joints.size()];
  int32_t get_position[all_joints.size()];

  if (dxl_wb.getProtocolVersion() == 2.0f)
  {

    result = dxl_wb.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                joint_ids_read,
                                all_joints.size(),
                                &log);
    arm_check_error(result, &log);
    result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  joint_ids_read,
                                                  all_joints.size(),
                                                  control_items["Present_Current"]->address,
                                                  control_items["Present_Current"]->data_length,
                                                  get_current,
                                                  &log);
    arm_check_error(result, &log);
    result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  joint_ids_read,
                                                  all_joints.size(),
                                                  control_items["Present_Velocity"]->address,
                                                  control_items["Present_Velocity"]->data_length,
                                                  get_velocity,
                                                  &log);
    arm_check_error(result, &log);
    result = dxl_wb.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  joint_ids_read,
                                                  all_joints.size(),
                                                  control_items["Present_Position"]->address,
                                                  control_items["Present_Position"]->data_length,
                                                  get_position,
                                                  &log);
    arm_check_error(result, &log);

    uint8_t index = 0;
    for(auto const& joint:all_joints)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;
      // Convert raw register values to the metric system
      if (strcmp(dxl_wb.getModelName(joint.motor_id), "XL-320") == 0) effort = dxl_wb.convertValue2Load(get_current[index]);
      else  effort = dxl_wb.convertValue2Current(get_current[index]);
      velocity = dxl_wb.convertValue2Velocity(joint.motor_id, get_velocity[index]);
      position = dxl_wb.convertValue2Radian(joint.motor_id, get_position[index]);
      joint_state_msg.name.push_back(joint.name);
      joint_state_msg.effort.push_back(effort);
      joint_state_msg.velocity.push_back(velocity);
      joint_state_msg.position.push_back(position);
      // If reading the 'gripper' motor, make sure to also add the finger positions in meters
      if (joint.name == "gripper" && use_default_gripper_bar && use_default_gripper_fingers)
      {
        joint_state_msg.name.push_back("left_finger");
        joint_state_msg.name.push_back("right_finger");
        gripper_effort = effort;
        position = arm_calculate_gripper_linear_position(position);
        joint_state_msg.position.push_back(position);
        joint_state_msg.position.push_back(-position);
        // put a '0' placeholder in the velocity and effort fields for the fingers as well
        joint_state_msg.velocity.push_back(0);
        joint_state_msg.velocity.push_back(0);
        joint_state_msg.effort.push_back(0);
        joint_state_msg.effort.push_back(0);
      }
      index++;
    }
  }
  else if(dxl_wb.getProtocolVersion() == 1.0f)
  {
    uint16_t length_of_data = control_items["Present_Position"]->data_length +
                              control_items["Present_Velocity"]->data_length +
                              control_items["Present_Current"]->data_length;
    uint32_t get_all_data[length_of_data];

    for (auto const& joint:all_joints)
    {
      result = dxl_wb.readRegister(joint.motor_id,
                                     control_items["Present_Position"]->address,
                                     length_of_data,
                                     get_all_data,
                                     &log);
      arm_check_error(result, &log);
      int16_t effort_raw = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
      int32_t velocity_raw = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
      int32_t position_raw = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;

      // Convert raw register values to the metric system
      effort = dxl_wb.convertValue2Load(effort_raw);
      velocity = dxl_wb.convertValue2Velocity(joint.motor_id, velocity_raw);
      position = dxl_wb.convertValue2Radian(joint.motor_id, position_raw);

      joint_state_msg.name.push_back(joint.name);
      joint_state_msg.effort.push_back(effort);
      joint_state_msg.velocity.push_back(velocity);
      joint_state_msg.position.push_back(position);
      // If reading the 'gripper' motor, make sure to also add the finger positions in meters
      if (joint.name == "gripper" && use_default_gripper_bar && use_default_gripper_fingers)
      {
        joint_state_msg.name.push_back("left_finger");
        joint_state_msg.name.push_back("right_finger");
        gripper_effort = effort;
        position = arm_calculate_gripper_linear_position(position);
        joint_state_msg.position.push_back(position);
        joint_state_msg.position.push_back(-position);
        // put a '0' placeholder in the velocity and effort fields for the fingers as well
        joint_state_msg.velocity.push_back(0);
        joint_state_msg.velocity.push_back(0);
        joint_state_msg.effort.push_back(0);
        joint_state_msg.effort.push_back(0);
      }
    }
  }
  // Publish the message to the joint_states topic
  joint_state_msg.header.stamp = ros::Time::now();
  joint_states = joint_state_msg;
  pub_joint_states.publish(joint_state_msg);
}

/// @brief ROS Timer that executes a joint trajectory for the arm (excludes gripper)
/// @details - Can follow either position or velocity trajectories and uses the 'time_from_start' field
void RobotArm::arm_execute_joint_trajectory(const ros::TimerEvent&)
{
  // 'cntr' keeps track of which waypoint in the trajectory is currently being executed
  static uint8_t cntr = 0;
  // If there is no joint trajectory to execute, or it has been preempted by the action server...
  if (!execute_joint_traj)
  {
    // If the trajectory has been preempted by the action server and the trajectory has already begun...
    if (cntr != 0)
    {
      // If the operating mode is currently set to velocity when the trajectory was preempted...
      if (arm_operating_mode == State::VELOCITY)
      {
        // Make sure to stop all motors by sending them goal speeds of 0 rad/s
        // In position operating mode, nothing has to be done to stop the motors...
        // They just remain at their last commanded positions.
        double vel_array[joint_num_write] = {0};
        arm_set_joint_velocities(vel_array);
        // if doing pid control, clear the pid errors
        if (use_pid_cntlrs)
          pid_cntlrs.multi_pid_clear();
      }
      ROS_INFO("Joint Trajectory stopped.");
      // Reset counter to 0 so the function is ready to start a new trajectory
      cntr = 0;
    }
    return;
  }

  // Get the size of the trajectory and the current ROS time. Also, get the
  // 'time_from_start' parameter from the waypoint to see at what time the
  // waypoint states should be achieved.
  int traj_size = jnt_tra_msg.points.size();
  double time_now = ros::Time::now().toSec() - joint_start_time;
  double time_from_start = jnt_tra_msg.points[cntr].time_from_start.toSec();

  // If the current time is greater than the current waypoint's 'time_from_start'...
  if (time_now > time_from_start)
  {
    // ...move on to the next waypoint.
    cntr++;
    // Check to make sure that the trajectory has not been completed
    if (cntr < traj_size)
    {
      // If the current operating mode is velocity control...
      if (arm_operating_mode == State::VELOCITY)
      {
        // If you do not want to do velocity-based-position-control on the desired trajectory positions
        // and instead just want to send the velocities in the trajectory message...
        if (!use_pid_cntlrs)
        {
          // ...then command the desired velocities from the waypoint message
          uint8_t counter = 0;
          double vel_array[joint_num_write];
          for (auto const& vel:jnt_tra_msg.points[cntr].velocities)
          {
            vel_array[counter] = vel;
            counter++;
          }
          arm_set_joint_velocities(vel_array);
        }
        // If you want to do velocity-based-position-control on the desired trajectory positions, then
        // set the reference or 'goal' positions for all the joints and use the desired 'goal' velocities
        // as feed-forward terms in the controllers. Then clear the pid errors since new reference positions have been set
        else
        {
          pid_cntlrs.multi_pid_set_refs(jnt_tra_msg.points[cntr].positions);
          pid_cntlrs.multi_pid_set_ffs(jnt_tra_msg.points[cntr].velocities);
          pid_cntlrs.multi_pid_clear();
        }
      }
      // ...otherwise command the desired positions from the waypoint message
      else if (arm_operating_mode == State::POSITION)
      {
        uint8_t counter = 0;
        double pos_array[joint_num_write];
        for (auto const& pos:jnt_tra_msg.points[cntr].positions)
        {
          pos_array[counter] = pos;
          counter++;
        }
        arm_set_joint_positions(pos_array);
      }
    }
    // If the trajectory has been completed...
    else
    {
      ROS_INFO("Trajectory done being executed.");
      // Stop the timer from executing commands and reset 'cntr'
      execute_joint_traj = false;
      cntr = 0;
      // if using pid control, clear the pid errors and stop all motion
      if (use_pid_cntlrs && arm_operating_mode == State::VELOCITY)
      {
        pid_cntlrs.multi_pid_clear();
        double vel_array[joint_num_write] = {0};
        arm_set_joint_velocities(vel_array);
      }
    }
  }
  // for every iteration through the timer callback, calculate and output the corrected
  // velocities from the controller
  if (use_pid_cntlrs && arm_operating_mode == State::VELOCITY && cntr > 0)
  {
    std::vector<double> arm_joint_states;
    double vel_array[joint_num_write];
    for (size_t i{0}; i < joint_num_write; i++)
      arm_joint_states.push_back(joint_states.position.at(i));
    pid_cntlrs.multi_pid_compute_control(vel_array, arm_joint_states);
    arm_set_joint_velocities(vel_array);
  }
}

/// @brief ROS Timer that executes a joint trajectory for the gripper only
/// @details - Can follow position trajectories only and uses the 'time_from_start' field
void RobotArm::arm_execute_gripper_trajectory(const ros::TimerEvent&)
{
  // 'cntr' keeps track of which waypoint in the trajectory is currently being executed
  static uint8_t cntr = 0;
  // If there is no gripper trajectory to execute, or it has been preempted by the action server...
  if (!execute_gripper_traj)
  {
    // If the trajectory has been preempted by the action server and the trajectory has already begun...
    if (cntr != 0)
    {
      ROS_INFO("Gripper Trajectory stopped.");
      // Reset counter to 0 so the function is ready to start a new trajectory
      cntr = 0;
    }
    return;
  }

  // Get the size of the trajectory and the current ROS time. Also, get the
  // 'time_from_start' parameter from the waypoint to see at what time the
  // waypoint states should be achieved.
  int traj_size = gripper_tra_msg.points.size();
  double time_now = ros::Time::now().toSec() - gripper_start_time;
  double time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();

  // If the current time is greater than the current waypoint's 'time_from_start'...
  if (time_now > time_from_start)
  {
    // For whatever reason, MoveIt sometimes sends joint trajectories with waypoints that
    // are so close in time, it's impossible to keep up. Instead, iterate through the
    // trajectory message until a waypoint is found with a 'time_from_start' time that
    // has not already passed.
    while (time_now > time_from_start && cntr < (traj_size - 1))
    {
      cntr++;
      time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();
    }
    // Command the gripper position of the waypoint...
    if (cntr < (traj_size - 1))
    {
      arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
    }
    // ...or if the 'time_from_start' of the final waypoint has already passed, command it anyway
    // so that the gripper ends up at the correct final position.
    else
    {
      arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
      ROS_INFO("Trajectory done being executed.");
      // Stop the timer from executing commands and reset 'cntr'
      execute_gripper_traj = false;
      cntr = 0;
    }
  }
}

/// @brief ROS Action server used to receive joint trajectories from MoveIt
/// @param goal - MoveIt specific trajectory message
void RobotArm::arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  // Check to make sure the trajectory has at least two waypoints
  if (goal->trajectory.points.size() < 2)
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    joint_action_server->setSucceeded(result);
    return;
  }
  // call the callback function to reorganize the data properly
  // and to set 'execute_joint_traj' to true
  arm_joint_trajectory_msg_callback(goal->trajectory);
  // Let the ROS timer do its thing... all this loop does is check
  // to see if MoveIt sent a 'preempt' request.
  ros::Rate r(100);
  while (execute_joint_traj)
  {
    if (joint_action_server->isPreemptRequested())
    {
      execute_joint_traj = false;
      joint_action_server->setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");
      return;
    }
    r.sleep();
  }
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  joint_action_server->setSucceeded(result);
}

/// @brief ROS Action server used to receive the gripper trajectory from MoveIt
/// @param goal - MoveIt specific trajectory message
void RobotArm::arm_gripper_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  // Check to make sure the trajectory has at least two waypoints
  if (goal->trajectory.points.size() < 2)
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gripper_action_server->setSucceeded(result);
    return;
  }
  // call the callback function to set 'execute_joint_traj' to true and do some checks
  arm_gripper_trajectory_msg_callback(goal->trajectory);
  // Let the ROS timer do its thing... all this loop does is check
  // to see if MoveIt sent a 'preempt' request or if the effort being seen by
  // the gripper is too high - in which case, the server preempts itself!
  ros::Rate r(100);
  while (execute_gripper_traj)
  {
    if (gripper_action_server->isPreemptRequested())
    {
      execute_gripper_traj = false;
      gripper_action_server->setPreempted();
      ROS_INFO("Gripper trajectory server preempted by client");
      return;
    }

    if (fabs(gripper_effort) > gripper_max_effort)
    {
      execute_gripper_traj = false;
      gripper_action_server->setPreempted();
      ROS_INFO("Gripper trajectory server preempted itself since max effort reached.");
      return;
    }
    r.sleep();
  }
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  gripper_action_server->setSucceeded(result);
}

/// @brief Sets the operating mode for the arm specifc motors (excluding gripper)
/// @param arm_mode - specified mode like "position", "velocity", and "pwm"
/// @param profile_vel - register value that sets the max velocity for the motors if using the "position" operating mode
/// @param profile_accel - register value that sets the max acceleration for the motors if using the "position" operating mode
bool RobotArm::arm_set_joint_operating_mode(const std::string arm_mode, const int32_t profile_vel, const int32_t profile_accel)
{
  bool arm_mode_set = true;
  if (arm_mode == "position")
    arm_set_joints_to_position_control(profile_vel, profile_accel);
  else if (arm_mode == "velocity")
    arm_set_joints_to_velocity_control(profile_accel);
  else if (arm_mode == "current")
    arm_set_joints_to_current_control();
  else if (arm_mode == "pwm")
    arm_set_joints_to_pwm_control();
  else if (arm_mode == "none")
  {
    arm_operating_mode = State::NONE;
    ROS_INFO("Arm joint control is set to None.");
  }
  else
    arm_mode_set = false;

  if (arm_mode_set)
  {
    for (auto const& joint : arm_joints)
      if (joint.name != "gripper")
        joint_map[joint.name].mode = arm_operating_mode;
  }
  else
    ROS_ERROR("Invalid control mode for the arm joints.");

  return arm_mode_set;
}

/// @brief Sets the operating mode for an individual joint only
/// @param joint_name - name of the joint for which to change the operating mode
/// @param mode - specified mode like "position", "ext_position", "velocity", and "pwm"
/// @param profile_vel - register value that sets the max velocity for the joint if using the "position" or "ext_position" operating modes
/// @param profile_accel - register value that sets the max acceleration for the joint if using the "position", "ext_position", or "velocity" operating modes
bool RobotArm::arm_set_single_joint_operating_mode(std::string joint_name, std::string mode, const int32_t profile_vel, const int32_t profile_accel)
{
  bool single_mode_set = true;
  if (mode == "position")
    arm_set_single_joint_to_position_control(joint_name, profile_vel, profile_accel);
  else if (mode == "ext_position")
    arm_set_single_joint_to_ext_position_control(joint_name, profile_vel, profile_accel);
  else if (mode == "velocity")
    arm_set_single_joint_to_velocity_control(joint_name, profile_accel);
  else if (mode == "current")
    arm_set_single_joint_to_current_control(joint_name);
  else if (mode == "pwm")
    arm_set_single_joint_to_pwm_control(joint_name);
  else if (mode == "none")
  {
    joint_map[joint_name].mode = State::NONE;
    ROS_INFO("%s control is set to None.", joint_name.c_str());
  }
  else
    single_mode_set = false;

  if (!single_mode_set)
    ROS_ERROR("Invalid control mode for the %s joint.", joint_name.c_str());

  return single_mode_set;
}

/// @brief Calculates the linear distance in meters between the gripper fingers given a servo angle
/// @param horn_angle - gripper angle in radians
double RobotArm::arm_calculate_gripper_linear_position(const double horn_angle)
{
  double a1 = horn_radius * sin(horn_angle);
  double c = sqrt(pow(horn_radius,2) - pow(a1,2));
  double a2 = sqrt(pow(arm_radius,2) - pow(c,2));
  return a1 + a2;
}

/// @brief Calculates the angular position in radians of the gripper servo given a linear distance between the gripper fingers
/// @param dist - distance in meters between the gripper fingers
float RobotArm::arm_calculate_gripper_angular_position(const float dist)
{
  double half_dist = dist / 2.0;
  float result = PI/2.0 - acos((pow(horn_radius,2) + pow(half_dist,2) - pow(arm_radius,2)) / (2 * horn_radius * half_dist));
  return result;
}

/// @brief Helper function to check if a function executed successfully
/// @param result - boolean returned from a function that is True if it executed correctly
/// @param log - error message to be output if the function did not run correctly
void RobotArm::arm_check_error(bool result, const char** log)
{
  if (result == false)
    ROS_ERROR("%s", *log);
}
