#ifndef ROBOT_ARM_POSES_H_
#define ROBOT_ARM_POSES_H_

#include <map>
#include <string>

namespace arm_poses
{
  // 'Home' positions for each robot. All joints are commanded to go to '0' radians
  const std::map<std::string, std::vector<double>> home_positions = {
    {"px100", {0, 0, 0, 0}},
    {"px150", {0, 0, 0, 0, 0}},
    {"rx150", {0, 0, 0, 0, 0}},
    {"rx200", {0, 0, 0, 0, 0}},
    {"vx250", {0, 0, 0, 0, 0}},
    {"vx300", {0, 0, 0, 0, 0}},
    {"vx300s", {0, 0, 0, 0, 0, 0}},
    {"wx200", {0, 0, 0, 0, 0}},
    {"wx250", {0, 0, 0, 0, 0}},
    {"wx250s", {0, 0, 0, 0, 0, 0}},
    {"pxxls", {0, 0}},
    {"wxxms", {0, 0}},
    {"wxxmd", {0, 0}},
    {"vxxms", {0, 0}},
    {"vxxmd", {0, 0}}
  };

  // 'Sleep' positions for each robot. All joints are commanded to go to specific positions [rad]
  // such that the arm is brought into a folded, compact state.
  const std::map<std::string, std::vector<double>> sleep_positions = {
    {"px100", {0, -1.88, -1.5, -0.8}},
    {"px150", {0, -1.80, -1.55, -0.8, 0}},
    {"rx150", {0, -1.80, -1.55, -0.8, 0}},
    {"rx200", {0, -1.80, -1.55, -0.8, 0}},
    {"vx250", {0, -1.76, -1.55, -0.8, 0}},
    {"vx300", {0, -1.76, -1.55, -0.8, 0}},
    {"vx300s", {0, -1.76, -1.55, 0, -0.8, 0}},
    {"wx200", {0, -1.80, -1.55, -0.8, 0}},
    {"wx250", {0, -1.80, -1.55, -0.8, 0}},
    {"wx250s", {0, -1.80, -1.55, 0, -0.8, 0}},
    {"pxxls", {0, 0}},
    {"wxxms", {0, 0}},
    {"wxxmd", {0, 0}},
    {"vxxms", {0, 0}},
    {"vxxmd", {0, 0}}
  };
}
#endif
