// Includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include "mr_navigation_controller/performAction.h"
#include "mr_navigation_controller/enable.h"
#include "mr_navigation_controller/move.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "mr_line_follower/followUntilQR.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Defines
#define M_PI		3.14159265358979323846
#define DEG_TO_RAD	(M_PI/180.0)
#define RAD_TO_DEG	(180.0/M_PI)
#define SSTR(x)		dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

/**
 * Skill struct
 */
struct Skill
{
    //
};


class NavigationController{
public:
	/**
	 * Constructors
	 */
	NavigationController() :
		      pNh_("~")
	{	
		// Get parameter names
		
		pNh_.param<std::string>("lineFollowEnableService", srv_lineUntilQR_name_, "mrLineFollower/enable");
		pNh_.param<std::string>("moveService", srv_move_name_, "mrGo/move");
		pNh_.param<std::string>("performAction", srv_action_name_, "mrNavigationController/performAction");
		pNh_.param<std::string>("status", pub_status_name_, "mrNavigationController/status");
		
		// Service
		srv_lineUntilQR_ = nh_.serviceClient<mr_navigation_controller::enable>(srv_lineUntilQR_name_);
		srv_move_ = nh_.serviceClient<mr_navigation_controller::move>(srv_move_name_);
		srv_action_ = nh_.advertiseService (
			srv_action_name_, &NavigationController::performActionCallback, this);
		
		// Publisher
		pub_status_ = nh_.advertise<std_msgs::String>(pub_status_name_, 10);
	}
	
	~NavigationController(){
		//
	}
	
	
	/**
	 * Skills
	 */
	bool lineUntilQR(std::string qr)
	{
		mr_line_follower::followUntilQR lineFollowerCall;
		lineFollowerCall.request.qr = qr;
		lineFollowerCall.request.time_limit = 30;
		return true;
	}
	
	bool linearMove(double distance)
	{
		mr_navigation_controller::move obj;
		obj.request.linear = distance;
		      srv_move_.call(obj);
		return obj.response.done;
	}
	
	bool angularMove(double distance)
	{
		mr_navigation_controller::move obj;
		obj.request.angle = distance;
		      srv_move_.call(obj);
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
	
	bool performActionCallback(mr_navigation_controller::performAction::Request &req,
							   mr_navigation_controller::performAction::Response &res)
	{
		// Search in graph how to perform action
		std::vector<Skill> skillVec = graphSearch(req.action);
		
		// Execute skills
		for(unsigned int i = 0; i<skillVec.size(); i++)
		{
			//
			std_msgs::String msg;
			msg.data = "Blabla";
			         pub_status_.publish(msg);
		}
		
		// Return status
		return true;
	}
	
private:
	ros::NodeHandle nh_, pNh_;
	ros::Publisher pub_status_;
	ros::ServiceClient srv_lineUntilQR_, srv_move_;
	ros::ServiceServer srv_action_;
	std::string srv_lineUntilQR_name_, srv_move_name_, pub_status_name_, srv_action_name_;
};


/**
 * Main
 */
int main(int argc, char** argv)
{
	// Init ROS Node
    ros::init(argc, argv, "MR_Navigation_Controller");
	NavigationController nc;
	ros::Rate rate(30);
    
	// ROS Spin: Handle callbacks
    while(ros::isShuttingDown()){
        ros::spinOnce();
		rate.sleep();
	}
    // Return
    return 0;
}
