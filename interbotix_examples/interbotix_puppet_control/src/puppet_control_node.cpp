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
  ros::init(argc, argv, "interbotix_puppet_control");
  ros::NodeHandle n;
  std::string robot_name_1, robot_name_2;
  ros::param::get("~robot_name_1", robot_name_1);
  ros::param::get("~robot_name_2", robot_name_2);

  // Subscribe to the first robot's joint states and publish those states as joint commands to the second robot
  ros::Subscriber sub_positions = n.subscribe(robot_name_1 + "/joint_states", 100, joint_state_cb);
  ros::Publisher pub_positions = n.advertise<interbotix_sdk::JointCommands>(robot_name_2 + "/joint/commands", 100);
  ros::Publisher pub_gripper = n.advertise<std_msgs::Float64>(robot_name_2 + "/gripper/command", 100);
  // Need to torque off the first robot arm as by default, it is torqued on
  ros::ServiceClient srv_torque_off = n.serviceClient<std_srvs::Empty>(robot_name_1 + "/torque_joints_off");
  // In this demo, let's put the second robot arm into 'position' control for the arm and the gripper.
  // Velocity control could also work but might not be as accurate for very quick motions. Note that this step
  // isn't really neccessary since the second robot's mode is already presest in the launch file.
  ros::ServiceClient srv_op_mode = n.serviceClient<interbotix_sdk::OperatingModes>(robot_name_2 + "/set_operating_modes");
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_sdk::RobotInfo>(robot_name_1 + "/get_robot_info");

  ros::Rate loop_rate(100);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_positions.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Torque off the first robot arm so that the user can easily manipulate it
  std_srvs::Empty e_srv;
  success = srv_torque_off.call(e_srv);
  if (!success)
  {
    ROS_ERROR("Could not torque off arm.");
    return 1;
  }

  // Set the operating mode of the second robot arm
  interbotix_sdk::OperatingModes op_srv;
  op_srv.request.cmd = interbotix_sdk::OperatingModes::Request::ARM_JOINTS_AND_GRIPPER;
  op_srv.request.mode = "position";
  op_srv.request.use_custom_profiles = true;
  success = srv_op_mode.call(op_srv);
  if (!success)
  {
    ROS_ERROR("Could not set operating mode.");
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
    // put joint positions from the first robot as position commands for the second robot
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
