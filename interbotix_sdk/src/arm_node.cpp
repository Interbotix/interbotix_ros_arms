#include "interbotix_sdk/arm_obj.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "arm");
    ros::NodeHandle n;
    std::string robot_name, robot_model;
    ros::param::get("~robot_name", robot_name);
    ros::param::get("~robot_model", robot_model);
    RobotArm bot(&n, robot_name, robot_model);
    ros::spin();
    return 0;
}
