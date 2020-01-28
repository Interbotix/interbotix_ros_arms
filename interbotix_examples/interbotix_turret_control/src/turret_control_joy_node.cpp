#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_turret_control/TurretJoyControl.h"

// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"PAN_CCW", 6},                  // buttons start here
                                               {"PAN_CW", 7},
                                               {"PAN_TILT_HOME", 9},
                                               {"FLIP_TILT", 11},
                                               {"FLIP_PAN", 12},
                                               {"SPEED_INC", 13},
                                               {"SPEED_DEC", 14},
                                               {"SPEED_COURSE", 15},
                                               {"SPEED_FINE", 16},
                                               {"TILT", 1},                     // axes start here
                                               {"PAN", 3}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"PAN_CCW", 6},                  // buttons start here
                                               {"PAN_CW", 7},
                                               {"PAN_TILT_HOME", 9},
                                               {"FLIP_TILT", 11},
                                               {"FLIP_PAN", 12},
                                               {"TILT", 1},                     // axes start here
                                               {"PAN", 3},
                                               {"SPEED_TYPE", 6},
                                               {"SPEED", 7}};

ros::Publisher pub_joy_cmd;                                                     // ROS Publisher to publish TurretJoyControl messages
ros::Subscriber sub_joy_raw;                                                    // ROS Subscriber to get raw Joy messages
interbotix_turret_control::TurretJoyControl prev_joy_cmd;                       // Stores the previous TurretJoyControl message
std::map<std::string, int> cntlr;                                               // Stores the button mappings of a specific controller
std::string controller_type;                                                    // Stores the selected controller type ('ps3' or 'ps4')
double threshold;                                                               // Sensitivity threshold for the analog stick control on the joystick

/// @brief Joystick callback to create custom TurretJoyControl messages to control the Interbotix Turret
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool flip_pan_cmd = false;
  static bool flip_tilt_cmd = false;
  static bool flip_pan_cmd_last_state = false;
  static bool flip_tilt_cmd_last_state = false;
  interbotix_turret_control::TurretJoyControl joy_cmd;

  // Check if the pan-and-tilt mechanism should be reset
  if (msg.buttons.at(cntlr["PAN_TILT_HOME"]) == 1)
  {
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_TILT_HOME;
    joy_cmd.tilt_cmd = interbotix_turret_control::TurretJoyControl::PAN_TILT_HOME;
  }

  if (controller_type == "ps3")
  {
    // Check the speed_cmd
    if (msg.buttons.at(cntlr["SPEED_INC"]) == 1)
      joy_cmd.speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_INC;
    else if (msg.buttons.at(cntlr["SPEED_DEC"]) == 1)
      joy_cmd.speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_DEC;

    // Check the toggle_speed_cmd
    if (msg.buttons.at(cntlr["SPEED_COURSE"]) == 1)
      joy_cmd.toggle_speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_COURSE;
    else if (msg.buttons.at(cntlr["SPEED_FINE"]) == 1)
      joy_cmd.toggle_speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_FINE;
  }
  else if (controller_type == "ps4")
  {
    // Check the speed_cmd
    if (msg.axes.at(cntlr["SPEED"]) == 1)
      joy_cmd.speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_INC;
    else if (msg.axes.at(cntlr["SPEED"]) == -1)
      joy_cmd.speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_DEC;

    // Check the toggle_speed_cmd
    if (msg.axes.at(cntlr["SPEED_TYPE"]) == 1)
      joy_cmd.toggle_speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_COURSE;
    else if (msg.axes.at(cntlr["SPEED_TYPE"]) == -1)
      joy_cmd.toggle_speed_cmd = interbotix_turret_control::TurretJoyControl::SPEED_FINE;
  }

  // Check if the pan_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_PAN"]) == 1 && flip_pan_cmd_last_state == false)
    flip_pan_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_PAN"]) == 1 && flip_pan_cmd_last_state == true)
    flip_pan_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_PAN"]) == 0)
    flip_pan_cmd_last_state = flip_pan_cmd;

  // Left stick pan control
  if (msg.axes.at(cntlr["PAN"]) <= -threshold && flip_pan_cmd == false)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CW;
  else if (msg.axes.at(cntlr["PAN"]) >= threshold && flip_pan_cmd == false)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CCW;
  else if (msg.axes.at(cntlr["PAN"]) <= -threshold && flip_pan_cmd == true)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CCW;
  else if (msg.axes.at(cntlr["PAN"]) >= threshold && flip_pan_cmd == true)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CW;

  // R & L button pan control
  if (msg.buttons.at(cntlr["PAN_CW"]) == 1 && flip_pan_cmd == false)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CW;
  else if (msg.buttons.at(cntlr["PAN_CCW"]) == 1 && flip_pan_cmd == false)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CCW;
  else if (msg.buttons.at(cntlr["PAN_CW"]) == 1 && flip_pan_cmd == true)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CCW;
  else if (msg.buttons.at(cntlr["PAN_CCW"]) == 1 && flip_pan_cmd == true)
    joy_cmd.pan_cmd = interbotix_turret_control::TurretJoyControl::PAN_CW;

  // Check if the tilt_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_TILT"]) == 1 && flip_tilt_cmd_last_state == false)
    flip_tilt_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_TILT"]) == 1 && flip_tilt_cmd_last_state == true)
    flip_tilt_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_TILT"]) == 0)
    flip_tilt_cmd_last_state = flip_tilt_cmd;

  // Check the tilt_cmd
  if (msg.axes.at(cntlr["TILT"]) <= -threshold && flip_tilt_cmd == false)
    joy_cmd.tilt_cmd = interbotix_turret_control::TurretJoyControl::TILT_DOWN;
  else if (msg.axes.at(cntlr["TILT"]) >= threshold && flip_tilt_cmd == false)
    joy_cmd.tilt_cmd = interbotix_turret_control::TurretJoyControl::TILT_UP;
  else if (msg.axes.at(cntlr["TILT"]) <= -threshold && flip_tilt_cmd == true)
    joy_cmd.tilt_cmd = interbotix_turret_control::TurretJoyControl::TILT_UP;
  else if (msg.axes.at(cntlr["TILT"]) >= threshold && flip_tilt_cmd == true)
    joy_cmd.tilt_cmd = interbotix_turret_control::TurretJoyControl::TILT_DOWN;

  // Only publish a TurretJoyControl message if any of the following fields have changed.
  if (!(prev_joy_cmd.pan_cmd == joy_cmd.pan_cmd &&
      prev_joy_cmd.tilt_cmd == joy_cmd.tilt_cmd &&
      prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd &&
      prev_joy_cmd.toggle_speed_cmd == joy_cmd.toggle_speed_cmd))
      pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interbotix_turret_control");
  ros::NodeHandle n;
  ros::param::get("~threshold", threshold);
  ros::param::get("~controller", controller_type);
  if (controller_type == "ps3")
    cntlr = ps3;
  else
    cntlr = ps4;
  sub_joy_raw = n.subscribe("joy", 100, joy_state_cb);
  pub_joy_cmd = n.advertise<interbotix_turret_control::TurretJoyControl>("joy/commands", 100);
  ros::spin();
  return 0;
}
