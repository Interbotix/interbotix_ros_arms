#include "interbotix_moveit_interface/moveit_interface.h"

/// @brief Constructor for the InterbotixMoveItInterface
InterbotixMoveItInterface::InterbotixMoveItInterface(ros::NodeHandle *node_handle)
  : node(*node_handle)
  {
    srv_moveit_plan = node.advertiseService("moveit_plan", &InterbotixMoveItInterface::moveit_planner, this);
    static const std::string PLANNING_GROUP = "interbotix_arm";
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
    visual_tools->deleteAllMarkers();
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools->publishText(text_pose, "InterbotixMoveItInterface", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools->trigger();

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("moveit_interface", "Reference frame: %s", move_group->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("moveit_interface", "End effector link: %s", move_group->getEndEffectorLink().c_str());
  }

/// @brief Destructor for the InterbotixMoveItInterface
InterbotixMoveItInterface::~InterbotixMoveItInterface()
{
  delete move_group;
  delete visual_tools;
  delete joint_model_group;
}

/// @brief Use MoveIt's planner to plan a trajectory to achieve the specified joint positions [rad]
/// @param joint_group_positions - vector of joint positions [rad] to write to the joints; sequence of positions match the joint name order expected by the 'interbotix_arm' group
bool InterbotixMoveItInterface::moveit_plan_joint_positions(const std::vector<double> joint_group_positions)
{
  visual_tools->deleteAllMarkers();
  move_group->setJointValueTarget(joint_group_positions);
  bool success = (move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools->publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(saved_plan.trajectory_, joint_model_group);
  visual_tools->trigger();
  return success;
}

/// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector pose
/// @param pose - desired pose of the end-effector (frame is placed at the 'ee_arm_link') w.r.t. the 'world' frame
bool InterbotixMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::Pose pose)
{
  visual_tools->deleteAllMarkers();
  move_group->setPoseTarget(pose);
  bool success = (move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(saved_plan.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}

/// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector position
/// @param x - translation along the 'X-axis' w.r.t. the 'world' frame
/// @param y - translation along the 'Y-axis' w.r.t. the 'world' frame
/// @param z - translation along the 'Z-axis' w.r.t. the 'world' frame
bool InterbotixMoveItInterface::moveit_plan_ee_position(double x, double y, double z)
{
  visual_tools->deleteAllMarkers();
  move_group->setPositionTarget(x, y, z);
  bool success = (move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  ROS_INFO("x, y, z: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Position Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(saved_plan.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}

/// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector orientation
/// @param quat - desired end-effector orientation expressed as a quaternion
bool InterbotixMoveItInterface::moveit_plan_ee_orientation(const geometry_msgs::Quaternion quat)
{
  visual_tools->deleteAllMarkers();
  move_group->setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  bool success = (move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  geometry_msgs::Pose pose;
  pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools->publishAxisLabeled(pose, "ee_pose");
  visual_tools->publishText(text_pose, "Orientation Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(saved_plan.trajectory_, joint_model_group);
  visual_tools->trigger();
  ROS_INFO("success: %d", success);
  return success;
}

/// @brief Use MoveIt's planner to plan a trajectory to move the end-effector to the specified waypoints
/// @param waypoints - sequence of goal poses for the end-effector to achieve; goal poses are relative to the 'world' frame
bool InterbotixMoveItInterface::moveit_plan_cartesian_path(const std::vector<geometry_msgs::Pose> waypoints)
{
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Visualizing (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  saved_plan = plan;
  saved_plan.trajectory_ = trajectory;
  bool success = false;

  // If a plan was found for over 90% of the waypoints...
  if (1.0 - fraction < 0.1)
    // consider that a successful planning attempt
    success = true;
  return success;
}

/// @brief Execute a Moveit plan on the robot arm
bool InterbotixMoveItInterface::moveit_execute_plan(void)
{
  bool success = (move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

/// @brief Set path constraints for a specific link
/// @param constrained_link - name of the link to constrain as defined in the URDF
/// @param reference_link - name of the link that the path of the 'constrained_link' is being constrained against
/// @param quat - desired orientation of the 'constrained_link' relative to the 'reference_link'
/// @param tolerance - allowable deviation [rad] from the constraint about the x, y, and z axes
void InterbotixMoveItInterface::moveit_set_path_constraint(const std::string constrained_link, const std::string reference_link, const geometry_msgs::Quaternion quat, const double tolerance)
{
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;

  // this parameter sets the importance of this constraint relative to other constraints that might be present. Closer to '0' means less important.
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // Since there is a constraint, it might take the planner a lot longer to come up with a valid plan - so give it some time
  move_group->setPlanningTime(30.0);
}

/// @brief Remove any path constraints
void InterbotixMoveItInterface::moveit_clear_path_constraints(void)
{
  move_group->clearPathConstraints();

  // Now that there are no constraints, reduce the planning time to the default
  move_group->setPlanningTime(5.0);
}

/// @brief Return the current end-effector pose relative to the 'world' frame
geometry_msgs::Pose InterbotixMoveItInterface::moveit_get_ee_pose(void)
{
  return move_group->getCurrentPose().pose;
}

/// @brief Scale the end-effector velocity down from the max velocity specified in the robot model
/// @param factor - a double between 0 and 1.
void InterbotixMoveItInterface::moveit_scale_ee_velocity(const double factor)
{
  move_group->setMaxVelocityScalingFactor(factor);
}

/// @brief ROS Service to plan or execute a desired end-effector pose, position, or orientation
/// @param req - custom message of type 'MoveItPlan'. Look at the service message for details
/// @param res [out] - a boolean specifying whether the plan or execution was successful and a 'string' message saying likewise
bool InterbotixMoveItInterface::moveit_planner(interbotix_moveit_interface::MoveItPlan::Request &req, interbotix_moveit_interface::MoveItPlan::Response &res)
{
  bool success = false;
  std::string service_type;
  if (req.cmd == interbotix_moveit_interface::MoveItPlan::Request::CMD_PLAN_POSE)
  {
    success = moveit_plan_ee_pose(req.ee_pose);
    service_type = "Planning EE pose";
  }
  else if (req.cmd == interbotix_moveit_interface::MoveItPlan::Request::CMD_PLAN_POSITION)
  {
    success = moveit_plan_ee_position(req.ee_pose.position.x, req.ee_pose.position.y, req.ee_pose.position.z);
    service_type = "Planning EE position";
  }
  else if (req.cmd == interbotix_moveit_interface::MoveItPlan::Request::CMD_PLAN_ORIENTATION)
  {
    success = moveit_plan_ee_orientation(req.ee_pose.orientation);
    service_type = "Planning EE orientation";
  }
  else if (req.cmd == interbotix_moveit_interface::MoveItPlan::Request::CMD_EXECUTE)
  {
    success = moveit_execute_plan();
    service_type = "Execution";
  }
  res.success = success;
  if (success)
    res.msg.data = service_type +" was successful!";
  else
    res.msg.data = service_type + " was not successful.";

  return true;
}
