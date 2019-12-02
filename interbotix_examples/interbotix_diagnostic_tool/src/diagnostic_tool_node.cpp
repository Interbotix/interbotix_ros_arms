#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_sdk/JointCommands.h"
#include "interbotix_sdk/RobotInfo.h"
#include "interbotix_sdk/RegisterValues.h"
#include "interbotix_diagnostic_tool/MotorTemps.h"

static const float PI = 3.14159265358979f;
sensor_msgs::JointState joint_states;

/// @brief Get the current joint states
/// @details - only needed to find initial joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interbotix_diagnostic_tool");
  ros::NodeHandle n;
  ros::Publisher pub_positions = n.advertise<interbotix_sdk::JointCommands>("joint/commands", 100);
  ros::Publisher pub_temperatures = n.advertise<interbotix_diagnostic_tool::MotorTemps>("joint_temperatures", 100);
  ros::Subscriber sub_joint_states = n.subscribe("joint_states", 100, joint_state_cb);
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_sdk::RobotInfo>("get_robot_info");
  ros::ServiceClient srv_registers = n.serviceClient<interbotix_sdk::RegisterValues>("get_motor_register_values");

  // publish joint position commands arbitrarily at 100 Hz
  int loop_hz = 100;
  ros::Rate loop_rate(loop_hz);
  // Temperature does not change so much over time...so only publish the current temp every 5 seconds
  int temp_freq = 5*loop_hz;
  // Wait until a message appears on the joint_states topic
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Wait until the 'interbotix_sdk' services are up
  // The 'srv_robot_info' service is needed to get the lower and upper joint limits while
  // the 'srv_registers' service is needed to poll the joint temperatures
  srv_robot_info.waitForExistence();
  srv_registers.waitForExistence();
  interbotix_sdk::RobotInfo srv_info;
  srv_robot_info.call(srv_info);
  // Get the number of seconds to run the test for
  int test_duration = 600;
  ros::param::get("~test_duration", test_duration);
  // Get the id of the joint that the user would like to move
  int cmd_joint = 0;
  ros::param::get("~cmd_joint", cmd_joint);
  // Since the motion of the joint will follow a sinusoidal path symmetric around '0' radians, determine which of the limits are closer to '0' and set that
  // as the max angular position of the path
  double pos_limit = std::min(srv_info.response.upper_joint_limits.at(cmd_joint), fabs(srv_info.response.lower_joint_limits.at(cmd_joint)));
  // The 'passive' joints will be commanded to hold their current positions for the duration of the test
  std::vector<double> joint_pos(joint_states.position.begin(), joint_states.position.begin() + srv_info.response.num_joints);
  joint_pos.at(cmd_joint) = 0;
  // Give a second for the user to let go of the robot arm after it is torqued on and holding its position
  ros::Duration(1.0).sleep();
  interbotix_sdk::JointCommands initial_pos_msg;
  initial_pos_msg.cmd = joint_pos;
  pub_positions.publish(initial_pos_msg);
  // Give a second for the joint that will be moving during the test to get to its zero position.
  ros::Duration(1.0).sleep();
  // Now loop for 10 minutes while commanding the 'moving' joint
  int cntr = 0;
  double time_start = ros::Time::now().toSec();
  while (ros::ok() && ros::Time::now().toSec() < (time_start + test_duration))
  {
    interbotix_sdk::JointCommands msg;
    // Sinusoidal trajectory for the joint to follow
    double pos = pos_limit*sin(PI/4.0*(ros::Time::now().toSec() - time_start));
    joint_pos.at(cmd_joint) = pos;
    msg.cmd = joint_pos;
    pub_positions.publish(msg);
    // Every 500 iterations or 5 seconds, publish the current joint temperatures
    if (cntr % temp_freq == 0)
    {
      cntr = 0;
      interbotix_sdk::RegisterValues srv_reg;
      srv_reg.request.cmd = interbotix_sdk::RegisterValues::Request::ARM_JOINTS;
      srv_reg.request.addr_name = "Present_Temperature";
      srv_registers.call(srv_reg);
      interbotix_diagnostic_tool::MotorTemps temp_msg;
      temp_msg.temps = srv_reg.response.values;
      pub_temperatures.publish(temp_msg);
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  // command arm to go to its sleep pose
  interbotix_sdk::JointCommands msg;
  msg.cmd = srv_info.response.sleep_pos;
  pub_positions.publish(msg);
  return 0;
}
