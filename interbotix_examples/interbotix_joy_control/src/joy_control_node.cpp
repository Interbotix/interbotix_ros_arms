#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_joy_control/JoyControl.h"


// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"GRIPPER_LOW_PWM", 0},          // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_HIGH_PWM", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_LEFT", 4},
                                               {"EE_Y_RIGHT", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"FLIP_EE_Y", 8},
                                               {"HOME_POSE", 9},
                                               {"SLEEP_POSE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_WRIST_ROTATE", 12},
                                               {"ARM_HIGH_SPEED", 13},
                                               {"ARM_LOW_SPEED", 14},
                                               {"ARM_COURSE_SPEED", 15},
                                               {"ARM_FINE_SPEED", 16},
                                               {"EE_X", 0},                     // axes start here
                                               {"EE_Z", 1},
                                               {"WRIST_ROTATE", 3},
                                               {"WRIST_ANGLE", 4}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"GRIPPER_LOW_PWM", 0},          // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_HIGH_PWM", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_LEFT", 4},
                                               {"EE_Y_RIGHT", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"FLIP_EE_Y", 8},
                                               {"HOME_POSE", 9},
                                               {"SLEEP_POSE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_WRIST_ROTATE", 12},
                                               {"EE_X", 0},                     // axes start here
                                               {"EE_Z", 1},
                                               {"WRIST_ROTATE", 3},
                                               {"WRIST_ANGLE", 4},
                                               {"ARM_SPEED_TYPE", 6},
                                               {"ARM_SPEED", 7}};

ros::Publisher pub_joy_cmd;
ros::Subscriber sub_joy_raw;
interbotix_joy_control::JoyControl prev_joy_cmd;
std::map<std::string, int> cntlr;
std::string controller_type;
double threshold;

/// @brief Joystick callback to create custom JoyControl messages to control the Interbotix arm
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool flip_wrist_rotate_cmd = false;
  static bool flip_wrist_rotate_cmd_last_state = false;
  static bool flip_ee_x_cmd = false;
  static bool flip_ee_x_cmd_last_state = false;
  static bool flip_ee_y_cmd = false;
  static bool flip_ee_y_cmd_last_state = false;
  interbotix_joy_control::JoyControl joy_cmd;

  // Check the gripper_cmd
  if (msg.buttons.at(cntlr["GRIPPER_CLOSE"]) == 1)
    joy_cmd.gripper_cmd = interbotix_joy_control::JoyControl::GRIPPER_CLOSE;
  else if (msg.buttons.at(cntlr["GRIPPER_OPEN"]) == 1)
    joy_cmd.gripper_cmd = interbotix_joy_control::JoyControl::GRIPPER_OPEN;

  // Check the waist_cmd
  if (msg.buttons.at(cntlr["WAIST_CCW"]) == 1)
    joy_cmd.waist_cmd = interbotix_joy_control::JoyControl::WAIST_CCW;
  else if (msg.buttons.at(cntlr["WAIST_CW"]) == 1)
    joy_cmd.waist_cmd = interbotix_joy_control::JoyControl::WAIST_CW;

  // Check the robot_pose
  if (msg.buttons.at(cntlr["HOME_POSE"]) == 1)
    joy_cmd.robot_pose = interbotix_joy_control::JoyControl::HOME_POSE;
  else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1)
    joy_cmd.robot_pose = interbotix_joy_control::JoyControl::SLEEP_POSE;

  if (controller_type == "ps3")
  {
    // Check the arm_speed_cmd
    if (msg.buttons.at(cntlr["ARM_HIGH_SPEED"]) == 1)
      joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_HIGH_SPEED;
    else if (msg.buttons.at(cntlr["ARM_LOW_SPEED"]) == 1)
      joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_LOW_SPEED;

    // Check the arm_toggle_speed_cmd
    if (msg.buttons.at(cntlr["ARM_COURSE_SPEED"]) == 1)
      joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_COURSE_SPEED;
    else if (msg.buttons.at(cntlr["ARM_FINE_SPEED"]) == 1)
      joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_FINE_SPEED;
  }
  else if (controller_type == "ps4")
  {
    // Check the arm_speed_cmd
    if (msg.axes.at(cntlr["ARM_SPEED"]) == 1)
      joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_HIGH_SPEED;
    else if (msg.axes.at(cntlr["ARM_SPEED"]) == -1)
      joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_LOW_SPEED;

    // Check the arm_toggle_speed_cmd
    if (msg.axes.at(cntlr["ARM_SPEED_TYPE"]) == 1)
      joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_COURSE_SPEED;
    else if (msg.axes.at(cntlr["ARM_SPEED_TYPE"]) == -1)
      joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_FINE_SPEED;
  }

  // Check the gripper_pwm_cmd
  if (msg.buttons.at(cntlr["GRIPPER_HIGH_PWM"]) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_joy_control::JoyControl::GRIPPER_HIGH_PWM;
  else if (msg.buttons.at(cntlr["GRIPPER_LOW_PWM"]) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_joy_control::JoyControl::GRIPPER_LOW_PWM;

  // Check if the ee_x_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == false)
    flip_ee_x_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == true)
    flip_ee_x_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 0)
    flip_ee_x_cmd_last_state = flip_ee_x_cmd;

  // Check the ee_x_cmd
  if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_FORWARD;
  else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_BACKWARD;
  else if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_BACKWARD;
  else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_FORWARD;

  // Check if the ee_y_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_EE_Y"]) == 1 && flip_ee_y_cmd_last_state == false)
    flip_ee_y_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_EE_Y"]) == 1 && flip_ee_y_cmd_last_state == true)
    flip_ee_y_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_EE_Y"]) == 0)
    flip_ee_y_cmd_last_state = flip_ee_y_cmd;

  // Check the ee_y_cmd
  if (msg.buttons.at(cntlr["EE_Y_LEFT"]) == 1 && flip_ee_y_cmd == false)
    joy_cmd.ee_y_cmd = interbotix_joy_control::JoyControl::EE_LEFT;
  else if (msg.buttons.at(cntlr["EE_Y_RIGHT"]) == 1 && flip_ee_y_cmd == false)
    joy_cmd.ee_y_cmd = interbotix_joy_control::JoyControl::EE_RIGHT;
  else if (msg.buttons.at(cntlr["EE_Y_LEFT"]) == 1 && flip_ee_y_cmd == true)
    joy_cmd.ee_y_cmd = interbotix_joy_control::JoyControl::EE_RIGHT;
  else if (msg.buttons.at(cntlr["EE_Y_RIGHT"]) == 1 && flip_ee_y_cmd == true)
    joy_cmd.ee_y_cmd = interbotix_joy_control::JoyControl::EE_LEFT;

  // Check the ee_z_cmd
  if (msg.axes.at(cntlr["EE_Z"]) >= threshold)
    joy_cmd.ee_z_cmd = interbotix_joy_control::JoyControl::EE_UP;
  else if (msg.axes.at(cntlr["EE_Z"]) <= -threshold)
    joy_cmd.ee_z_cmd = interbotix_joy_control::JoyControl::EE_DOWN;

  // Check if the wrist_rotate_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_WRIST_ROTATE"]) == 1 && flip_wrist_rotate_cmd_last_state == false)
    flip_wrist_rotate_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_WRIST_ROTATE"]) == 1 && flip_wrist_rotate_cmd_last_state == true)
    flip_wrist_rotate_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_WRIST_ROTATE"]) == 0)
    flip_wrist_rotate_cmd_last_state = flip_wrist_rotate_cmd;

  // Check the wrist_rotate_cmd
  if (msg.axes.at(cntlr["WRIST_ROTATE"]) >= threshold && flip_wrist_rotate_cmd == false)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CW;
  else if (msg.axes.at(cntlr["WRIST_ROTATE"]) <= -threshold && flip_wrist_rotate_cmd == false)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CCW;
  else if (msg.axes.at(cntlr["WRIST_ROTATE"]) >= threshold && flip_wrist_rotate_cmd == true)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CCW;
  else if (msg.axes.at(cntlr["WRIST_ROTATE"]) <= -threshold && flip_wrist_rotate_cmd == true)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CW;

  // Check the wrist_angle_cmd
  if (msg.axes.at(cntlr["WRIST_ANGLE"]) >= threshold)
    joy_cmd.wrist_angle_cmd = interbotix_joy_control::JoyControl::WRIST_ANGLE_CCW;
  else if (msg.axes.at(cntlr["WRIST_ANGLE"]) <= -threshold)
    joy_cmd.wrist_angle_cmd = interbotix_joy_control::JoyControl::WRIST_ANGLE_CW;

  // Only publish a JoyControl message if any of the following fields have changed.
  if (!(prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd &&
      prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd &&
      prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd &&
      prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd &&
      prev_joy_cmd.wrist_angle_cmd == joy_cmd.wrist_angle_cmd &&
      prev_joy_cmd.wrist_rotate_cmd == joy_cmd.wrist_rotate_cmd &&
      prev_joy_cmd.gripper_cmd == joy_cmd.gripper_cmd &&
      prev_joy_cmd.arm_speed_cmd == joy_cmd.arm_speed_cmd &&
      prev_joy_cmd.arm_toggle_speed_cmd == joy_cmd.arm_toggle_speed_cmd &&
      prev_joy_cmd.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd &&
      prev_joy_cmd.robot_pose == joy_cmd.robot_pose))
      pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interbotix_joy_control_node");
  ros::NodeHandle n;
  ros::param::get("~threshold", threshold);
  ros::param::get("~controller", controller_type);
  if (controller_type == "ps3")
    cntlr = ps3;
  else
    cntlr = ps4;
  sub_joy_raw = n.subscribe("joy", 100, joy_state_cb);
  pub_joy_cmd = n.advertise<interbotix_joy_control::JoyControl>("joy/commands", 100);
  ros::spin();
  return 0;
}
