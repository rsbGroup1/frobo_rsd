// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "mr_navigation_controller/performAction.h"
#include "mr_navigation_controller/enable.h"
#include "mr_navigation_controller/followLineToQR.h"
#include "mr_navigation_controller/angularMove.h"
#include "mr_navigation_controller/linearMove.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Skill struct
struct Skill
{
    //
};

// Global var
ros::Publisher _statusTopic;
ros::ServiceClient _serviceLineFollow, _serviceAngularMove,_serviceLinearMove;

// Skills
bool lineUntilQR(int QR)
{
    mr_navigation_controller::followLineToQR obj;
    obj.request.QR = QR;
    if(!_serviceLineFollow.call(obj))
        return obj.response.status;
    else
        return false;
}

bool linearMove(double distance)
{
    mr_navigation_controller::linearMove obj;
    obj.request.distance = distance;
    _serviceLinearMove.call(obj);
    return obj.response.done;
}

bool angularMove(double distance)
{
    mr_navigation_controller::angularMove obj;
    obj.request.angle = distance;
    _serviceAngularMove.call(obj);
    return obj.response.done;
}

bool goToFreePosition(double x, double y)
{

}

bool moveToDispenser()
{

}

bool moveToCharger()
{

}

bool moveFromDispenser()
{

}

bool moveFromCharger()
{

}

bool changeLineWC1()
{

}

bool changeLineWC2()
{

}

bool changeLineWC3()
{

}

std::vector<Skill> graphSearch(int action)
{
    return std::vector<Skill>();
}

bool performActionCallback(mr_navigation_controller::performAction::Request &req, mr_navigation_controller::performAction::Response &res)
{
    // Search in graph how to perform action
    std::vector<Skill> skillVec = graphSearch(req.action);

    // Execute skills
    for(unsigned int i = 0; i<skillVec.size(); i++)
    {
        //
        std_msgs::String msg;
        msg.data = "Blabla";
        _statusTopic.publish(msg);
    }

    // Return status
    return true;
}

// Functions
int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "MR_Navigation_Controller");
    ros::NodeHandle nh, pNh("~");

    // Get parameter names
    std::string lineFollowString, linearMoveString, angularMoveString;
    pNh.param<std::string>("lineFollowService", lineFollowString, "mrLineFollower/followQR");
    pNh.param<std::string>("linearMoveService", linearMoveString, "mrGo/linearMove");
    pNh.param<std::string>("angularMoveService", angularMoveString, "mrGo/angularMove");

    // Service
    _serviceLineFollow = nh.serviceClient<mr_navigation_controller::followLineToQR>(lineFollowString);
    _serviceAngularMove = nh.serviceClient<mr_navigation_controller::angularMove>(angularMoveString);
    _serviceLinearMove = nh.serviceClient<mr_navigation_controller::linearMove>(linearMoveString);
    ros::ServiceServer actionServer = nh.advertiseService("mrNavigationController/performAction", performActionCallback);

    // Publisher
    _statusTopic = nh.advertise<std_msgs::String>("mrNavigationController/status", 10);

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}
