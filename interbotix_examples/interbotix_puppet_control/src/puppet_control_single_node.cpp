#include <ros/ros.h>
#include "interbotix_sdk/OperatingModes.h"
#include "interbotix_sdk/JointCommands.h"
#include "interbotix_sdk/RobotInfo.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

sensor_msgs::JointState joint_states;         // globally available joint_states message

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interbotix_puppet_control_single");
  ros::NodeHandle n;

  // Subscribe to the robot's joint states and publish those states as joint commands so that rosbag can record them
  ros::Subscriber sub_positions = n.subscribe("joint_states", 100, joint_state_cb);
  ros::Publisher pub_positions = n.advertise<interbotix_sdk::JointCommands>("joint/commands", 100);
  ros::Publisher pub_gripper = n.advertise<std_msgs::Float64>("gripper/command", 100);
  // Need to torque off the robot arm as by default, it is torqued on
  ros::ServiceClient srv_torque_off = n.serviceClient<std_srvs::Empty>("torque_joints_off");
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_sdk::RobotInfo>("get_robot_info");

  ros::Rate loop_rate(100);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_positions.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Torque off the robot arm so that the user can easily manipulate it. Conveniently, this also prevents
  // the motors from acting upon the position commands being sent to the 'joint/commands' topic
  std_srvs::Empty e_srv;
  success = srv_torque_off.call(e_srv);
  if (!success)
  {
    ROS_ERROR("Could not torque off arm.");
    return 1;
  }

  // Get the number of joints that the first robot has
  interbotix_sdk::RobotInfo robot_info_srv;
  success = srv_robot_info.call(robot_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get robot info.");
    return 1;
  }

  size_t cntr = 0;
  while (ros::ok())
  {
    // put joint positions from the robot as position commands for itself
    interbotix_sdk::JointCommands pos_msg;
    for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
    {
      pos_msg.cmd.push_back(joint_states.position.at(i));
    }
    pub_positions.publish(pos_msg);

    // Every 10 iterations, send a gripper position command. If this is sent every loop iteration,
    // it will cause the driver node to lag (since the port needs to be accessed twice per loop iteration instead of just once)
    if (cntr == 10)
    {
      std_msgs::Float64 gpr_msg;
      gpr_msg.data = joint_states.position.at(joint_states.position.size()-2)*2;
      pub_gripper.publish(gpr_msg);
      cntr = 0;
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
