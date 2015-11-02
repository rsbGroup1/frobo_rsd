// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "mr_navigation_controller/performAction.h"
#include "mr_navigation_controller/enable.h"
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
    mr_navigation_controller::enable obj;
    obj.request.enable = true;
    if(!_serviceLineFollow.call(obj))
    {
        ROS_ERROR("Could not start line following!");
        return false;
    }

    while(true)
    {
        // WAIT TILL QR IS FOUND
    }

    obj.request.enable = false;
    if(!_serviceLineFollow.call(obj))
    {
        ROS_ERROR("Could not stop line following!");
        return false;
    }

    return true;
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

void qrCallback(std_msgs::String msg)
{

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
    std::string lineFollowEnableString, linearMoveString, angularMoveString, qrSub;
    pNh.param<std::string>("lineFollowEnableService", lineFollowEnableString, "mrLineFollower/enable");
    pNh.param<std::string>("linearMoveService", linearMoveString, "mrGo/linearMove");
    pNh.param<std::string>("angularMoveService", angularMoveString, "mrGo/angularMove");
    pNh.param<std::string>("qrSub", qrSub, "/mrCameraProcessing/QR");

    // Service
    _serviceLineFollow = nh.serviceClient<mr_navigation_controller::enable>(lineFollowEnableString);
    _serviceAngularMove = nh.serviceClient<mr_navigation_controller::angularMove>(angularMoveString);
    _serviceLinearMove = nh.serviceClient<mr_navigation_controller::linearMove>(linearMoveString);
    ros::ServiceServer actionServer = nh.advertiseService("mrNavigationController/performAction", performActionCallback);

    // Subscriber
    ros::Subscriber qrSubscriber = nh.subscribe(qrSub, 10, qrCallback);

    // Publisher
    _statusTopic = nh.advertise<std_msgs::String>("mrNavigationController/status", 10);

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}
