// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var
ros::Publisher _motorCommandTopic;

void missionCallback(std_msgs::String msg)
{

}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_NavigationController_Node");
    ros::NodeHandle nh;

    // Publisher
    _motorCommandTopic = nh.advertise<std_msgs::String>("motorCommandTopic", 1);

    // Subscriber
    ros::Subscriber subMission = nh.subscribe("missionPlannerTopic", 10, missionCallback);


    // ROS Spin: Handle callbacks
    ros::spin();

    // Return
    return 0;
}
