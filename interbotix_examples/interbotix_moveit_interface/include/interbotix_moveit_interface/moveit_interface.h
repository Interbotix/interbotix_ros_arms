#ifndef MOVEIT_INTERFACE_H_
#define MOVEIT_INTERFACE_H_

#include "interbotix_moveit_interface/MoveItPlan.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

class InterbotixMoveItInterface
{
public:
  /// @brief Constructor for the InterbotixMoveItInterface
  explicit InterbotixMoveItInterface(ros::NodeHandle *node_handle);

  /// @brief Destructor for the InterbotixMoveItInterface
  ~InterbotixMoveItInterface();

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified joint positions [rad]
  /// @param joint_group_positions - vector of joint positions [rad] to write to the joints; sequence of positions match the joint name order expected by the 'interbotix_arm' group
  bool moveit_plan_joint_positions(const std::vector<double> joint_group_positions);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector pose
  /// @param pose - desired pose of the end-effector (frame is placed at the 'ee_arm_link') w.r.t. the 'world' frame
  bool moveit_plan_ee_pose(const geometry_msgs::Pose pose);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector position
  /// @param x - translation along the 'X-axis' w.r.t. the 'world' frame
  /// @param y - translation along the 'Y-axis' w.r.t. the 'world' frame
  /// @param z - translation along the 'Z-axis' w.r.t. the 'world' frame
  bool moveit_plan_ee_position(double x, double y, double z);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector orientation
  /// @param quat - desired end-effector orientation expressed as a quaternion
  bool moveit_plan_ee_orientation(const geometry_msgs::Quaternion quat);

  /// @brief Use MoveIt's planner to plan a trajectory to move the end-effector to the specified waypoints
  /// @param waypoints - sequence of goal poses for the end-effector to achieve; goal poses are relative to the 'world' frame
  bool moveit_plan_cartesian_path(const std::vector<geometry_msgs::Pose> waypoints);

  /// @brief Execute a Moveit plan on the robot arm
  bool moveit_execute_plan(void);

  /// @brief Set path constraints for a specific link
  /// @param constrained_link - name of the link to constrain as defined in the URDF
  /// @param reference_link - name of the link that the path of the 'constrained_link' is being constrained against
  /// @param quat - desired orientation of the 'constrained_link' relative to the 'reference_link'
  /// @param tolerance - allowable deviation [rad] from the constraint about the x, y, and z axes
  void moveit_set_path_constraint(const std::string constrained_link, const std::string reference_link, const geometry_msgs::Quaternion quat, const double tolerance);

  /// @brief Remove any path constraints
  void moveit_clear_path_constraints(void);

  /// @brief Return the current end-effector pose relative to the 'world' frame
  geometry_msgs::Pose moveit_get_ee_pose(void);

  /// @brief Scale the end-effector velocity down from the max velocity specified in the robot model
  /// @param factor - a double between 0 and 1.
  void moveit_scale_ee_velocity(const double factor);

private:
  ros::NodeHandle node;                                                         // ROS node handler
  ros::ServiceServer srv_moveit_plan;                                           // Service to plan or execute a goal pose for the end-effector
  Eigen::Isometry3d text_pose;                                                  // Pose of text w.r.t. the 'world' frame in Rviz
  const robot_state::JointModelGroup *joint_model_group;                        // Holds the joints in the 'interbotix_arm' group
  moveit_visual_tools::MoveItVisualTools *visual_tools;                         // Used to display text and other markers in Rviz
  moveit::planning_interface::MoveGroupInterface *move_group;                   // MoveIt object that can actually plan and execute trajectories
  moveit::planning_interface::MoveGroupInterface::Plan saved_plan;              // Plan object that holds the calculated trajectory
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  // Not applied in this demo but would be used to add objects to the world

  /// @brief ROS Service to plan or execute a desired end-effector pose, position, or orientation
  /// @param req - custom message of type 'MoveItPlan'. Look at the service message for details
  /// @param res [out] - a boolean specifying whether the plan or execution was successful and a 'string' message saying likewise
  bool moveit_planner(interbotix_moveit_interface::MoveItPlan::Request &req, interbotix_moveit_interface::MoveItPlan::Response &res);
};

#endif
