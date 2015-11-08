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
		      skills_(&srv_lineUntilQR_, &srv_move_, &pub_status_),
		      graph_(&pub_current_node_)
	{	
		// Get parameter names
		
        pNh_.param<std::string>("lineFollowEnableService", srv_lineUntilQR_name_, "mrLineFollower/lineUntilQR");
		pNh_.param<std::string>("moveService", srv_move_name_, "mrGo/move");
		pNh_.param<std::string>("performAction", srv_action_name_, "mrNavigationController/performAction");
		pNh_.param<std::string>("status", pub_status_name_, "mrNavigationController/status");
		pNh_.param<std::string>("currentNode", pub_current_node_name_, "mrNavigationController/currentNode");
		pNh_.param<int>("searchLimit", search_limit_, 50);
		
		// Service
        srv_lineUntilQR_ = 
        nh_.serviceClient<mr_line_follower::followUntilQR>(srv_lineUntilQR_name_);
        srv_move_ = nh_.serviceClient<mr_go::move>(srv_move_name_);
        srv_action_ = nh_.advertiseService(srv_action_name_, &NavigationController::performActionCallback, this);
		
		// Publisher
		pub_status_ = nh_.advertise<std_msgs::String>(pub_status_name_, 10);
		pub_current_node_ = nh_.advertise<std_msgs::String>(pub_current_node_name_, 10);
		
		// Create the graph
		createGraph();
		//graph_.showGraph();
		
		// Debug
		// Search in graph how to perform action
		graph_.setCurrentNode((char*)"line_start");
		solution_ = graph_.bfs((char*)"wc3_conveyor", search_limit_);
		
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
	 * 
	 */
	bool performActionCallback(mr_navigation_controller::performAction::Request &req,
							   mr_navigation_controller::performAction::Response &res)
	{
		// Search in graph how to perform action
		solution_ = graph_.bfs(req.action.c_str(), search_limit_);
		
		// Execute skills
		for(auto& skill : solution_)
			skill();
		
		// Return status
		res.success = true;
		return true;
	}
	
	/**
	 * Creates the graph with std::function and std::bind
	 */
	void createGraph(){
		// Nodes
		graph_.addNode((char*)"line_start");
		graph_.addNode((char*)"line_end");
		graph_.addNode((char*)"wc1");
		graph_.addNode((char*)"wc1_conveyor");
		graph_.addNode((char*)"wc1_robot");
		graph_.addNode((char*)"wc2");
		graph_.addNode((char*)"wc2_conveyor");
		graph_.addNode((char*)"wc2_robot");
		graph_.addNode((char*)"wc3");
		graph_.addNode((char*)"wc3_conveyor");
		graph_.addNode((char*)"wc3_robot");
		graph_.addNode((char*)"wc_exit");
		graph_.addNode((char*)"box");
		graph_.addNode((char*)"bricks_pre");
		graph_.addNode((char*)"bricks");
		graph_.addNode((char*)"charging_pre");
		graph_.addNode((char*)"charging");
		
		// Skills
		std::vector<std::function<void()>> line_start_TO_wc1;
		line_start_TO_wc1.push_back(std::bind(&Skills::lineUntilQR, skills_, "workcell_1"));
		
		std::vector<std::function<void()>> wc1_TO_wc2;
		wc1_TO_wc2.push_back(std::bind(&Skills::lineUntilQR, skills_, "workcell_2"));
		
		std::vector<std::function<void()>> wc2_TO_wc3;
		wc2_TO_wc3.push_back(std::bind(&Skills::lineUntilQR, skills_, "workcell_3"));
		
		
		
		std::vector<std::function<void()>> wc1_TO_wc1_conveyor;
		wc1_TO_wc1_conveyor.push_back(std::bind(&Skills::angularMove, skills_, 90));
		wc1_TO_wc1_conveyor.push_back(std::bind(&Skills::lineUntilQR, skills_, "conveyor_stop"));
		
		std::vector<std::function<void()>> wc1_conveyor_TO_wc1_robot;
		wc1_conveyor_TO_wc1_robot.push_back(std::bind(&Skills::changeLineWC1, skills_));
		
		std::vector<std::function<void()>> wc1_robot_TO_wc_exit;
		wc1_robot_TO_wc_exit.push_back(std::bind(&Skills::angularMove, skills_, -180));
		wc1_robot_TO_wc_exit.push_back(std::bind(&Skills::lineUntilQR, skills_, "wc_exit"));
		
		
		
		std::vector<std::function<void()>> wc2_TO_wc2_conveyor;
		wc2_TO_wc2_conveyor.push_back(std::bind(&Skills::angularMove, skills_, 90));
		wc2_TO_wc2_conveyor.push_back(std::bind(&Skills::lineUntilQR, skills_, "conveyor_stop"));
		
		std::vector<std::function<void()>> wc2_conveyor_TO_wc2_robot;
		wc2_conveyor_TO_wc2_robot.push_back(std::bind(&Skills::changeLineWC2, skills_));
		
		std::vector<std::function<void()>> wc2_robot_TO_wc_exit;
		wc2_robot_TO_wc_exit.push_back(std::bind(&Skills::angularMove, skills_, -180));
		wc2_robot_TO_wc_exit.push_back(std::bind(&Skills::lineUntilQR, skills_, "wc_exit"));
		
		
		
		std::vector<std::function<void()>> wc3_TO_wc3_conveyor;
		wc3_TO_wc3_conveyor.push_back(std::bind(&Skills::angularMove, skills_, 90));
		wc3_TO_wc3_conveyor.push_back(std::bind(&Skills::lineUntilQR, skills_, "conveyor_stop"));
		
		std::vector<std::function<void()>> wc3_conveyor_TO_wc3_robot;
		wc3_conveyor_TO_wc3_robot.push_back(std::bind(&Skills::changeLineWC3, skills_));
		
		std::vector<std::function<void()>> wc3_robot_TO_wc_exit;
		wc3_robot_TO_wc_exit.push_back(std::bind(&Skills::angularMove, skills_, -180));
		wc3_robot_TO_wc_exit.push_back(std::bind(&Skills::lineUntilQR, skills_, "wc_exit"));
		
		
		
		std::vector<std::function<void()>> wc_exit_TO_line_end;
		wc_exit_TO_line_end.push_back(std::bind(&Skills::angularMove, skills_, -90));
		wc_exit_TO_line_end.push_back(std::bind(&Skills::lineUntilQR, skills_, "line_end"));
		
		
		
		std::vector<std::function<void()>> line_end_TO_box;
		line_end_TO_box.push_back(std::bind(&Skills::goToFreePosition, skills_, 2, 3));
		
		std::vector<std::function<void()>> box_TO_line_start;
		box_TO_line_start.push_back(std::bind(&Skills::goToFreePosition, skills_, 1, 2));
		
		/*
		 * TODO Changer and bricks
		 */
		
		
		// Vertices 
		graph_.addVertex((char*)"line_start", (char*)"wc1", 1, line_start_TO_wc1);
		graph_.addVertex((char*)"wc1", (char*)"wc2", 1, wc1_TO_wc2);
		graph_.addVertex((char*)"wc2", (char*)"wc3", 1, wc2_TO_wc3);
		graph_.addVertex((char*)"wc_exit", (char*)"line_end", 1, wc_exit_TO_line_end);
		
		graph_.addVertex((char*)"wc1", (char*)"wc1_conveyor", 1, wc1_TO_wc1_conveyor);
		graph_.addVertex((char*)"wc1_conveyor", (char*)"wc1_robot", 1, wc1_conveyor_TO_wc1_robot);
		graph_.addVertex((char*)"wc1_robot", (char*)"wc_exit", 1, wc1_robot_TO_wc_exit);
		
		graph_.addVertex((char*)"wc2", (char*)"wc2_conveyor", 1, wc2_TO_wc2_conveyor);
		graph_.addVertex((char*)"wc2_conveyor", (char*)"wc2_robot", 1, wc2_conveyor_TO_wc2_robot);
		graph_.addVertex((char*)"wc2_robot", (char*)"wc_exit", 1, wc2_robot_TO_wc_exit);
		
		graph_.addVertex((char*)"wc3", (char*)"wc3_conveyor", 1, wc3_TO_wc3_conveyor);
		graph_.addVertex((char*)"wc3_conveyor", (char*)"wc3_robot", 1, wc3_conveyor_TO_wc3_robot);
		graph_.addVertex((char*)"wc3_robot", (char*)"wc_exit", 1, wc3_robot_TO_wc_exit);
		
		
		graph_.addVertex((char*)"line_end", (char*)"box", 1, line_end_TO_box);
		graph_.addVertex((char*)"box", (char*)"line_start", 1, box_TO_line_start);
	}

	
private:
	ros::NodeHandle nh_, pNh_;
	ros::Publisher pub_status_, pub_current_node_;
	ros::ServiceClient srv_lineUntilQR_, srv_move_;
	ros::ServiceServer srv_action_;
	std::string srv_lineUntilQR_name_, srv_move_name_, pub_status_name_, 
		srv_action_name_, pub_current_node_name_;
	Skills skills_;
	Graph graph_;
	std::vector<std::function<void()>> solution_;
	int search_limit_;
	std::string status;
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
