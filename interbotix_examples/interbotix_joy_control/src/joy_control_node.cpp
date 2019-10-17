#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_joy_control/JoyControl.h"

// position in joy_message button array
static const int GRIPPER_OPEN = 1;
static const int GRIPPER_CLOSE = 3;
static const int GRIPPER_HIGH_PWM = 2;
static const int GRIPPER_LOW_PWM = 0;
static const int WAIST_CCW = 6;
static const int WAIST_CW = 7;
static const int HOME_POSE = 9;
static const int SLEEP_POSE = 10;
static const int FLIP_EE_X = 11;
static const int FLIP_WRIST_ROTATE = 12;
static const int ARM_HIGH_SPEED = 13;
static const int ARM_LOW_SPEED = 14;
static const int ARM_COURSE_SPEED = 15;
static const int ARM_FINE_SPEED = 16;
// position in joy_message axes array
static const int EE_X = 0;
static const int EE_Z = 1;
static const int WRIST_ROTATE = 3;
static const int WRIST_ANGLE = 4;

ros::Publisher pub_joy_cmd;
ros::Subscriber sub_positions;
interbotix_joy_control::JoyControl prev_joy_cmd;
double threshold;

/// @brief Joystick callback to create custom JoyControl messages to control the Interbotix arm
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool flip_wrist_rotate_cmd = false;
  static bool flip_wrist_rotate_cmd_last_state = false;
  static bool flip_ee_x_cmd = false;
  static bool flip_ee_x_cmd_last_state = false;
  interbotix_joy_control::JoyControl joy_cmd;

  // Check the gripper_cmd
  if (msg.buttons.at(GRIPPER_CLOSE) == 1)
    joy_cmd.gripper_cmd = interbotix_joy_control::JoyControl::GRIPPER_CLOSE;
  else if (msg.buttons.at(GRIPPER_OPEN) == 1)
    joy_cmd.gripper_cmd = interbotix_joy_control::JoyControl::GRIPPER_OPEN;

  // Check the waist_cmd
  if (msg.buttons.at(WAIST_CCW) == 1)
    joy_cmd.waist_cmd = interbotix_joy_control::JoyControl::WAIST_CCW;
  else if (msg.buttons.at(WAIST_CW) == 1)
    joy_cmd.waist_cmd = interbotix_joy_control::JoyControl::WAIST_CW;

  // Check the robot_pose
  if (msg.buttons.at(HOME_POSE) == 1)
    joy_cmd.robot_pose = interbotix_joy_control::JoyControl::HOME_POSE;
  else if (msg.buttons.at(SLEEP_POSE) == 1)
    joy_cmd.robot_pose = interbotix_joy_control::JoyControl::SLEEP_POSE;

  // Check the arm_speed_cmd
  if (msg.buttons.at(ARM_HIGH_SPEED) == 1)
    joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_HIGH_SPEED;
  else if (msg.buttons.at(ARM_LOW_SPEED) == 1)
    joy_cmd.arm_speed_cmd = interbotix_joy_control::JoyControl::ARM_LOW_SPEED;

  // Check the arm_toggle_speed_cmd
  if (msg.buttons.at(ARM_COURSE_SPEED) == 1)
    joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_COURSE_SPEED;
  else if (msg.buttons.at(ARM_FINE_SPEED) == 1)
    joy_cmd.arm_toggle_speed_cmd = interbotix_joy_control::JoyControl::ARM_FINE_SPEED;

  // Check the gripper_pwm_cmd
  if (msg.buttons.at(GRIPPER_HIGH_PWM) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_joy_control::JoyControl::GRIPPER_HIGH_PWM;
  else if (msg.buttons.at(GRIPPER_LOW_PWM) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_joy_control::JoyControl::GRIPPER_LOW_PWM;

  // Check if the ee_x_cmd should be flipped
  if (msg.buttons.at(FLIP_EE_X) == 1 && flip_ee_x_cmd_last_state == false)
    flip_ee_x_cmd = true;
  else if (msg.buttons.at(FLIP_EE_X) == 1 && flip_ee_x_cmd_last_state == true)
    flip_ee_x_cmd = false;
  else if (msg.buttons.at(FLIP_EE_X) == 0)
    flip_ee_x_cmd_last_state = flip_ee_x_cmd;

  // Check the ee_x_cmd
  if (msg.axes.at(EE_X) >= threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_FORWARD;
  else if (msg.axes.at(EE_X) <= -threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_BACKWARD;
  else if (msg.axes.at(EE_X) >= threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_BACKWARD;
  else if (msg.axes.at(EE_X) <= -threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_joy_control::JoyControl::EE_FORWARD;

  // Check the ee_z_cmd
  if (msg.axes.at(EE_Z) >= threshold)
    joy_cmd.ee_z_cmd = interbotix_joy_control::JoyControl::EE_UP;
  else if (msg.axes.at(EE_Z) <= -threshold)
    joy_cmd.ee_z_cmd = interbotix_joy_control::JoyControl::EE_DOWN;

  // Check if the wrist_rotate_cmd should be flipped
  if (msg.buttons.at(FLIP_WRIST_ROTATE) == 1 && flip_wrist_rotate_cmd_last_state == false)
    flip_wrist_rotate_cmd = true;
  else if (msg.buttons.at(FLIP_WRIST_ROTATE) == 1 && flip_wrist_rotate_cmd_last_state == true)
    flip_wrist_rotate_cmd = false;
  else if (msg.buttons.at(FLIP_WRIST_ROTATE) == 0)
    flip_wrist_rotate_cmd_last_state = flip_wrist_rotate_cmd;

  // Check the wrist_rotate_cmd
  if (msg.axes.at(WRIST_ROTATE) >= threshold && flip_wrist_rotate_cmd == false)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CW;
  else if (msg.axes.at(WRIST_ROTATE) <= -threshold && flip_wrist_rotate_cmd == false)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CCW;
  else if (msg.axes.at(WRIST_ROTATE) >= threshold && flip_wrist_rotate_cmd == true)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CCW;
  else if (msg.axes.at(WRIST_ROTATE) <= -threshold && flip_wrist_rotate_cmd == true)
    joy_cmd.wrist_rotate_cmd = interbotix_joy_control::JoyControl::WRIST_ROTATE_CW;

  // Check the wrist_angle_cmd
  if (msg.axes.at(WRIST_ANGLE) >= threshold)
    joy_cmd.wrist_angle_cmd = interbotix_joy_control::JoyControl::WRIST_ANGLE_CCW;
  else if (msg.axes.at(WRIST_ANGLE) <= -threshold)
    joy_cmd.wrist_angle_cmd = interbotix_joy_control::JoyControl::WRIST_ANGLE_CW;

  // Only publish a JoyControl message if any of the following fields have changed.
  if (!(prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd &&
      prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd &&
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
  sub_positions = n.subscribe("/joy", 100, joy_state_cb);
  pub_joy_cmd = n.advertise<interbotix_joy_control::JoyControl>("joy/commands", 100);
  ros::spin();
  return 0;
}
