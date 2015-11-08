// Includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <functional>

#include <ros/ros.h>
#include "mr_navigation_controller/performAction.h"
#include "mr_line_follower/followUntilQR.h"
#include "mr_go/move.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "skills.h"
#include "graph.h"


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


class NavigationController
{
public:
	/**
	 * Constructors
	 */
	NavigationController() :
		      pNh_("~"),
		      skills_(&srv_lineUntilQR_, &srv_move_)
	{	
		// Get parameter names
		
        pNh_.param<std::string>("lineFollowEnableService", srv_lineUntilQR_name_, "mrLineFollower/lineUntilQR");
		pNh_.param<std::string>("moveService", srv_move_name_, "mrGo/move");
		pNh_.param<std::string>("performAction", srv_action_name_, "mrNavigationController/performAction");
		pNh_.param<std::string>("status", pub_status_name_, "mrNavigationController/status");
		
		// Service
        srv_lineUntilQR_ = 
        nh_.serviceClient<mr_line_follower::followUntilQR>(srv_lineUntilQR_name_);
        srv_move_ = nh_.serviceClient<mr_go::move>(srv_move_name_);
        srv_action_ = nh_.advertiseService(srv_action_name_, &NavigationController::performActionCallback, this);
		
		// Publisher
		pub_status_ = nh_.advertise<std_msgs::String>(pub_status_name_, 10);
		
		// Create the graph
		createGraph();
		//graph_.showGraph();
		
		// Debug
		// Search in graph how to perform action
		graph_.setCurrentNode((char*)"start_line");
		solution_ = graph_.bfs((char*)"conveyor_workcell_1");
		
		// Execute skills
		for(auto& skill : solution_){
			std::cout << "   ";
			skill();
		}
		
	}
	
	~NavigationController(){
		//
	}
	

	/**
	 * Creates the graph with std::function and std::bind
	 * Explanation: http://oopscenities.net/2012/02/24/c11-stdfunction-and-stdbind/
	 * 
	 * CREATE GRAPH HERE
	 * 
	 * 
	 */
	void createGraph(){
		// Nodes
		graph_.addNode((char*)"start_line");
		graph_.addNode((char*)"workcell_1");
		graph_.addNode((char*)"conveyor_workcell_1");
		
		// Skills
		std::vector<std::function<void()>> vertex_1;
		vertex_1.push_back(std::bind(&Skills::lineUntilQR, skills_, "workcell_1"));
		
		std::vector<std::function<void()>> vertex_2;
		vertex_2.push_back(std::bind(&Skills::angularMove, skills_, 90));
		vertex_2.push_back(std::bind(&Skills::changeLineWC1, skills_));
		
		
		// Vertices 
		graph_.addVertex((char*)"start_line", (char*)"workcell_1", 1, vertex_1);
		graph_.addVertex((char*)"workcell_1", (char*)"conveyor_workcell_1", 1, vertex_2);
		
	}
	
	/**
	 * 
	 */
	bool performActionCallback(mr_navigation_controller::performAction::Request &req,
							   mr_navigation_controller::performAction::Response &res)
	{
		// Search in graph how to perform action
		solution_ = graph_.bfs(req.action.c_str());
		
		// Execute skills
		for(auto& skill : solution_)
			skill();
		
		// Return status
		res.success = true;
		return true;
	}
	
private:
	ros::NodeHandle nh_, pNh_;
	ros::Publisher pub_status_;
	ros::ServiceClient srv_lineUntilQR_, srv_move_;
	ros::ServiceServer srv_action_;
	std::string srv_lineUntilQR_name_, srv_move_name_, pub_status_name_, srv_action_name_;
	Skills skills_;
	Graph graph_;
	std::vector<std::function<void()>> solution_;
    
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
    while(ros::isShuttingDown())
    {
        ros::spinOnce();
		rate.sleep();
	}
    // Return
    return 0;
}
