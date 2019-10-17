#include <ros/ros.h>
#include <interbotix_sdk/JointCommands.h>
#include <interbotix_sdk/RobotInfo.h>
#include <sensor_msgs/JointState.h>

// PID gains for each joint controller
struct pid_gains
{
    double kp;                          // proportional
    double ki;                          // integral
    double kd;                          // derivative
};

// PID state for each joint controller
struct pid_state
{
    struct pid_gains gains;
    double u_max;                       // maximum control effort
    double u_min;                       // minimum control effort
    double ref;                         // reference to track
    double p_error;                     // proportional error
    double i_error;                     // integral error
    double d_error;                     // derivative error
};

double max_value;                       // max allowable pwm/current for control effort
std::vector<double> Kp;                 // Proportional gain for each joint in the robot
std::vector<double> Ki;                 // Integral gain for each joint in the robot
std::vector<double> Kd;                 // Derivative gain for each joint in the robot
sensor_msgs::JointState joint_states;   // Most up-to-date joint states
std::vector<pid_state> pid_states;      // Controller PID state for each joint
ros::Publisher pub_cmds;                // ROS Publisher to command pwms/currents to the joints

/// @brief ROS Subscriber callback function to get joint states
/// @param msg - most up-to-date joint state message
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

/// @brief ROS Timer callback function to run the joint controllers
void pid_controller(const ros::TimerEvent&)
{
  size_t cntr = 0;
  interbotix_sdk::JointCommands msg;

  for (auto &st: pid_states)
  {
    double actual;
    actual = joint_states.position.at(cntr);

    double error = st.ref - actual;
    double i_error = st.i_error + error;

    st.d_error = error - st.p_error;
    st.p_error = error;

    double u = st.gains.kp * st.p_error + st.gains.ki * i_error + st.gains.kd * st.d_error;

    // Don't allow integrator windup
    if (st.u_min < u && u < st.u_max)
      st.i_error = i_error;
    else if (u < st.u_min)
      u = st.u_min;
    else if (u > st.u_max)
      u = st.u_max;
    msg.cmd.push_back(u);
    // ROS_INFO("ref: %.2f, actual: %.2f, value: %.2f", st.ref, actual, u);
    cntr++;
  }
  pub_cmds.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interbotix_pid_controller");
  ros::NodeHandle n;
  pub_cmds = n.advertise<interbotix_sdk::JointCommands>("joint/commands", 100);
  ros::Subscriber sub_joint_states = n.subscribe("joint_states", 100, joint_state_cb);
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_sdk::RobotInfo>("get_robot_info");
  ros::Timer tmr_pid_controller = n.createTimer(ros::Duration(1/100), pid_controller);
  ros::Rate loop_rate(100);
  std::string control_mode;
  // get the desired control mode (pwm/current)
  ros::param::param<std::string>("~control_mode", control_mode, "pwm");
  // get the gains and max control effort from the parameter server
  ros::param::param<double>(control_mode + "/max_value", max_value, 885);
  ros::param::get(control_mode + "/Kp", Kp);
  ros::param::get(control_mode + "/Ki", Ki);
  ros::param::get(control_mode + "/Kd", Kd);

  // Wait until the first joint_states message is received
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Call the 'get_robot_info' service to get the 'home' and 'sleep' positions
  interbotix_sdk::RobotInfo robot_info_srv;
  bool success = srv_robot_info.call(robot_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get robot info.");
    return 1;
  }

  // Initialize pid_states for the pwm/current controller
  for (size_t i {0}; i < robot_info_srv.response.num_joints; i++)
  {
    pid_state st = {0};
    st.gains = {Kp[i], Ki[i], Kd[i]};
    st.u_max = max_value;
    st.u_min = -max_value;
    st.ref = joint_states.position.at(i);
    pid_states.push_back(st);
  }

  // Initialize the reference positions for each joint to their respective 'home' positions
  for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
    pid_states.at(i).ref = robot_info_srv.response.home_pos.at(i);

  double time_start;

  // Give 10 seconds for the robot arm to settle at reference positions
  time_start = ros::Time::now().toSec();
  while(ros::ok() && ros::Time::now().toSec() < (time_start + 10))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize the reference positions for each joint to their respective 'sleep' positions
  for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
    pid_states.at(i).ref = robot_info_srv.response.sleep_pos.at(i);

  // Give 10 seconds for the robot arm to settle at reference positions
  time_start = ros::Time::now().toSec();
  while(ros::ok() && ros::Time::now().toSec() < (time_start + 10))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Command all joints to '0' pwm/current, essentially torquing them off
  interbotix_sdk::JointCommands msg;
  msg.cmd = std::vector<double>(robot_info_srv.response.num_joints, 0);
  pub_cmds.publish(msg);
  return 0;
}
