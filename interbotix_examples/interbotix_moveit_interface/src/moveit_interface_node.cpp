#include "interbotix_moveit_interface/moveit_interface.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "moveit_interface");
    // We need two spinners to run the InterbotixMoveItInterface node
    //    - One spinner allows ROS messages to be processed during the blocking 'move_group.move()' command
    //    - Another spinner allows ROS messages to be processed during a blocking service call (since planning
    //      can take some time and the service does not return until the planning is done)
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle n;
    // Create instance of MoveIt interface
    InterbotixMoveItInterface interface(&n);
    ros::waitForShutdown();
    return 0;
}
