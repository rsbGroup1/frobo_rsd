// Includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <functional>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include "mr_navigation_controller/performAction.h"
#include "mr_navigation_controller/setCurrentNode.h"
#include "mr_line_follower/followUntilQR.h"
#include "mr_go/move.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "msgs/BoolStamped.h"

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
        pNh_ ("~"),
        skills_ (&srv_lineUntilQR_, &srv_move_, &srv_lineUntilLidar_, &pub_status_, 
		 &pub_initialize_, &pub_deadman_, &srv_detect_obstacles_)
    {
        // Get parameter names
        pNh_.param<std::string> ("lineFollowEnableService", srv_lineUntilQR_name_, "mrLineFollower/lineUntilQR");
        pNh_.param<std::string> ("lineFollowEnableLidarService", srv_lineUntilLidar_name_, "mrLineFollower/lineUntilLidar");
        pNh_.param<std::string> ("moveService", srv_move_name_, "mrGo/move");
        pNh_.param<std::string> ("performAction", srv_action_name_, "mrNavigationController/performAction");
        pNh_.param<std::string> ("status", pub_status_name_, "mrNavigationController/status");
        pNh_.param<std::string> ("currentNode", pub_current_node_name_, "mrNavigationController/currentNode");
        pNh_.param<std::string> ("setCurrentNode", srv_set_current_node_name_, "mrNavigationController/setCurrentNode");
		pNh_.param<std::string> ("obstacleDetectorService", srv_detect_obstacles_name_, "/mrObstacleDetector/enabler");
        pNh_.param<int> ("searchLimit", search_limit_, 100);
	pNh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        pNh_.param<std::string> ("sub_battery", sub_battery_name_, "/fmInformation/battery");
        //std::string path_to_node = ros::package::getPath("mrNavigationController");

        // Service
        srv_lineUntilQR_ = nh_.serviceClient<mr_line_follower::followUntilQR> (srv_lineUntilQR_name_);
        srv_lineUntilLidar_ = nh_.serviceClient<mr_line_follower::followUntilLidar> (srv_lineUntilLidar_name_);
        srv_move_ = nh_.serviceClient<mr_go::move> (srv_move_name_);
        srv_action_ = nh_.advertiseService (srv_action_name_, &NavigationController::performActionCallback, this);
        srv_set_current_node_ = nh_.advertiseService (srv_set_current_node_name_, &NavigationController::setCurrentNodeCallback, this);

        // Subscriber
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("amcl_pose", 10, &NavigationController::poseReceived, this);
        sub_battery_ = nh_.subscribe<std_msgs::Float32> (sub_battery_name_, 10, &NavigationController::batteryCallback, this);

        // Publisher
        pub_status_ = nh_.advertise<std_msgs::String> (pub_status_name_, 10);
        pub_current_node_ = nh_.advertise<std_msgs::String> (pub_current_node_name_, 10);
        pub_initialize_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose", 1);
	pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);

        // Create the graph and put the start node from the launch file
        graph_ = new Graph (&pub_current_node_);
        std::string start_node;
        nh_.param<std::string> ("start_node", start_node, "line_start");
        graph_->setCurrentNode (start_node);
        createGraph();
        //graph_->showGraph();

        // Inialize AMCL
        std::ifstream localisationFile;
        //std::string path_to_file = path_to_node +
        localisationFile.open ("localization.csv");
        ROS_INFO ("Waiting for global localisation");
        ros::service::waitForService ("global_localization", ros::Duration (5, 0));


        if (localisationFile.is_open())
        {
            ros::ServiceClient initalize = nh_.serviceClient<std_srvs::Empty> ("global_localization");
            std_srvs::Empty srv;
            initalize.call (srv);

            // Load initialization
            geometry_msgs::PoseWithCovarianceStamped p;
            localisationFile >> p.pose.pose.position.x;
            localisationFile >> p.pose.pose.position.y;
            localisationFile >> p.pose.pose.orientation.x;
            localisationFile >> p.pose.pose.orientation.y;
            localisationFile >> p.pose.pose.orientation.z;
            localisationFile >> p.pose.pose.orientation.w;
            p.header.frame_id = "map";

            ros::Duration d (6, 0);
            d.sleep();
            pub_initialize_.publish (p);

            localisationFile.close();
        }
        else
        {
            // global initialization
            ros::ServiceClient initalize = nh_.serviceClient<std_srvs::Empty> ("global_localization");
            std_srvs::Empty srv;
            initalize.call (srv);
        }
    }

    ~NavigationController()
    {
        //
    }

    bool setCurrentNodeCallback (mr_navigation_controller::setCurrentNode::Request& req,
                                 mr_navigation_controller::setCurrentNode::Response& res)
    {
        bool node_exists = false;
        bool already_here = false;

	// Check if the node exists
        for (auto & node : graph_->getNodes())
        {
            if (node.getName() == req.node.c_str())
                node_exists = true;
        }
        
        // Check if it is already in the node
	if (req.node.c_str() == graph_->getCurrentNode())
	    already_here = true;

	// Handle the output
        if (node_exists == true)
        {
            if (already_here)
            {
                ROS_INFO ("I am already here!");
            }
            else
            {
                graph_->setCurrentNode (req.node.c_str());
                res.success = true;
                return true;
            }
        }
        else
        {
            ROS_ERROR ("The node %s is NOT in the graph", req.node.c_str());
            res.success = false;
            return false;
        }
    }

    /**
     * Service that search for the solution and execute it
     */
    bool performActionCallback (mr_navigation_controller::performAction::Request& req,
                                mr_navigation_controller::performAction::Response& res)
    {
        // Search in graph how to perform action
        solution_ = graph_->bfs (req.action.c_str(), search_limit_);

        // Execute skills
        for (auto & skill : solution_)
            skill();

        // Return status
        res.success = true;
        return true;
    }

    /**
     * Exectue all the skills in the solution_
     */
    void executeSkills()
    {
        for (auto & skill : solution_)
            skill();
    }

    /**
     * Creates the graph with std::function and std::bind
     */
    void createGraph()
    {
        // Nodes
        graph_->addNode ("line_start");
        graph_->addNode ("line_end");
        graph_->addNode ("wc1");
        graph_->addNode ("wc1_conveyor");
        graph_->addNode ("wc1_robot");
        graph_->addNode ("wc2");
        graph_->addNode ("wc2_conveyor");
        graph_->addNode ("wc2_robot");
        graph_->addNode ("wc3");
        graph_->addNode ("wc3_conveyor");
        graph_->addNode ("wc3_robot");
        graph_->addNode ("wc_exit");
        graph_->addNode ("box");
        graph_->addNode ("pre_bricks");
        graph_->addNode ("bricks");
        graph_->addNode ("pre_charge");
        graph_->addNode ("pre_charge_line");
        graph_->addNode ("charge");
        graph_->addNode ("charge_line");


        /*
         *
         * Skills
         *
         */

        std::vector<std::function<void() >> line_start_TO_wc1;
		line_start_TO_wc1.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        line_start_TO_wc1.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_entrance"));
        line_start_TO_wc1.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1"));
		line_start_TO_wc1.push_back (std::bind (&Skills::detectObstacles, &skills_, false));

        std::vector<std::function<void() >> wc1_TO_wc1_conveyor;
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::linearMove, &skills_, 0.6));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_conveyor"));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1_conveyor"));

        std::vector<std::function<void() >> wc1_conveyor_TO_wc1_robot;
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_load"));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1_robot"));

        std::vector<std::function<void() >> wc1_robot_TO_wc_exit;
		wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, -180));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_exit"));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1_exit"));
		wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));


        std::vector<std::function<void() >> wc1_TO_wc2;
        wc1_TO_wc2.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_entrance"));
        wc1_TO_wc2.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2"));

        std::vector<std::function<void() >> wc2_TO_wc2_conveyor;
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_conveyor"));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2_conveyor"));

        std::vector<std::function<void() >> wc2_conveyor_TO_wc2_robot;
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_load"));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2_robot"));

        std::vector<std::function<void() >> wc2_robot_TO_wc_exit;
		wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, -180));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_exit"));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2_exit"));
		wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));



        std::vector<std::function<void() >> wc2_TO_wc3;
        wc2_TO_wc3.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_entrance"));
        wc2_TO_wc3.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3"));

        std::vector<std::function<void() >> wc3_TO_wc3_conveyor;
        wc3_TO_wc3_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc3_TO_wc3_conveyor.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_conveyor"));
        wc3_TO_wc3_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3_conveyor"));

        std::vector<std::function<void() >> wc3_conveyor_TO_wc3_robot;
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_load"));
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3_robot"));

        std::vector<std::function<void() >> wc3_robot_TO_wc_exit;
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, -180));
		wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_exit"));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3_exit"));
		wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));



        std::vector<std::function<void() >> wc_exit_TO_line_end;
		wc_exit_TO_line_end.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc_exit_TO_line_end.push_back (std::bind (&Skills::angularMove, &skills_, -90));
        wc_exit_TO_line_end.push_back (std::bind (&Skills::lineUntilQR, &skills_, "line_out"));
        wc_exit_TO_line_end.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_end"));
		wc_exit_TO_line_end.push_back (std::bind (&Skills::detectObstacles, &skills_, false));



        std::vector<std::function<void() >> line_end_TO_box;
        line_end_TO_box.push_back (std::bind (&Skills::setInitialPoseAMCL, &skills_, 3.615, -2.046, 2.115));
        line_end_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.1, 0.7 , -1.9));
        line_end_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.5, -1.5 , -1.9));
        line_end_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        std::vector<std::function<void() >> box_TO_line_start;
        box_TO_line_start.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.1, 0.7 , 1.3));
        box_TO_line_start.push_back (std::bind (&Skills::goToFreePosition, &skills_, 3.3, -2 , -1.2));
        box_TO_line_start.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_start"));



        std::vector<std::function<void() >> box_TO_pre_bricks;
        box_TO_pre_bricks.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.9, -2.2 , -2.7));
        box_TO_pre_bricks.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_bricks"));

        std::vector<std::function<void() >> pre_bricks_TO_box;
        pre_bricks_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.5, -1.5 , 1.3));
        pre_bricks_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));


        std::vector<std::function<void() >> pre_bricks_TO_bricks;
        pre_bricks_TO_bricks.push_back (std::bind (&Skills::linearMove, &skills_, 0.15));
        pre_bricks_TO_bricks.push_back (std::bind (&Graph::setCurrentNode, graph_, "bricks"));
        std::vector<std::function<void() >> bricks_TO_pre_bricks;
        bricks_TO_pre_bricks.push_back (std::bind (&Skills::linearMove, &skills_, -0.15));
        bricks_TO_pre_bricks.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_bricks"));


        std::vector<std::function<void() >> box_TO_pre_charge;
        box_TO_pre_charge.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.56, -2.41 , -0.2));
        box_TO_pre_charge.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.02, -2.51 , -0.2));
        box_TO_pre_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge"));

        std::vector<std::function<void() >> pre_charge_TO_box;
        pre_charge_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.8, -2.1 , -0.6));
        pre_charge_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        std::vector<std::function<void() >> pre_charge_TO_charge;
        pre_charge_TO_charge.push_back (std::bind (&Skills::linearMove, &skills_, 0.1));
        pre_charge_TO_charge.push_back (std::bind (&Skills::chargeDectectionAndBackupPlan, &skills_, battery_level_, 13.0));
        pre_charge_TO_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "charge"));

        std::vector<std::function<void() >> charge_TO_pre_charge;
        charge_TO_pre_charge.push_back (std::bind (&Skills::linearMove, &skills_, -0.1));
        charge_TO_pre_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge"));

        std::vector<std::function<void() >> line_start_TO_box;
        line_start_TO_box.push_back (std::bind (&Skills::setInitialPoseAMCL, &skills_, 0.1, 0.7 , 1.3));
        line_start_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.1, 0.7 , -1.9));
        line_start_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.5, -1.5 , -1.9));
        line_start_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        // line following charging
        std::vector<std::function<void() >> box_TO_pre_charge_line;
        box_TO_pre_charge_line.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.56, -2.41 , -0.2));
        box_TO_pre_charge_line.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_load"));
        box_TO_pre_charge_line.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge_line"));

        std::vector<std::function<void() >> pre_charge_line_TO_charge_line;
        pre_charge_line_TO_charge_line.push_back (std::bind (&Skills::chargeDectectionAndBackupPlan, &skills_, battery_level_, 13.0));        
	//pre_charge_line_TO_charge_line.push_back (std::bind (&Skills::linearMove, &skills_, 0.1));
	//pre_charge_line_TO_charge_line.push_back (std::bind (&Skills::lineUntilLidar, &skills_, 0.2));
        pre_charge_line_TO_charge_line.push_back (std::bind (&Graph::setCurrentNode, graph_, "charge_line"));

        std::vector<std::function<void() >> pre_charge_line_TO_box;
        pre_charge_line_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.1, 0.7 , -1.9));
        pre_charge_line_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        std::vector<std::function<void() >> charge_line_TO_pre_charge_line;
        charge_line_TO_pre_charge_line.push_back (std::bind (&Skills::linearMove, &skills_, -0.1));
	//charge_line_TO_pre_charge_line.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_load"));
        charge_line_TO_pre_charge_line.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge_line"));


        // Vertices
        graph_->addVertex ("line_start", "wc1", 1, line_start_TO_wc1);
        graph_->addVertex ("wc1", "wc2", 1, wc1_TO_wc2);
        graph_->addVertex ("wc2", "wc3", 1, wc2_TO_wc3);
        graph_->addVertex ("wc_exit", "line_end", 1, wc_exit_TO_line_end);

        graph_->addVertex ("wc1", "wc1_conveyor", 1, wc1_TO_wc1_conveyor);
        graph_->addVertex ("wc1_conveyor", "wc1_robot", 1, wc1_conveyor_TO_wc1_robot);
        graph_->addVertex ("wc1_robot", "wc_exit", 1, wc1_robot_TO_wc_exit);

        graph_->addVertex ("wc2", "wc2_conveyor", 1, wc2_TO_wc2_conveyor);
        graph_->addVertex ("wc2_conveyor", "wc2_robot", 1, wc2_conveyor_TO_wc2_robot);
        graph_->addVertex ("wc2_robot", "wc_exit", 1, wc2_robot_TO_wc_exit);

        graph_->addVertex ("wc3", "wc3_conveyor", 1, wc3_TO_wc3_conveyor);
        graph_->addVertex ("wc3_conveyor", "wc3_robot", 1, wc3_conveyor_TO_wc3_robot);
        graph_->addVertex ("wc3_robot", "wc_exit", 1, wc3_robot_TO_wc_exit);

        graph_->addVertex ("line_end", "box", 1, line_end_TO_box);
        graph_->addVertex ("box", "line_start", 1, box_TO_line_start);

        graph_->addVertex ("box", "pre_charge", 1, box_TO_pre_charge);
        graph_->addVertex ("pre_charge", "box", 1, pre_charge_TO_box);

        graph_->addVertex ("box", "pre_bricks", 1, box_TO_pre_bricks);
        graph_->addVertex ("pre_bricks", "box", 1, pre_bricks_TO_box);

        graph_->addVertex ("pre_charge", "charge", 1, pre_charge_TO_charge);
        graph_->addVertex ("charge", "pre_charge", 1, charge_TO_pre_charge);

        graph_->addVertex ("pre_bricks", "bricks", 1, pre_bricks_TO_bricks);
        graph_->addVertex ("bricks", "pre_bricks", 1, bricks_TO_pre_bricks);

        graph_->addVertex ("line_start", "box", 1, line_start_TO_box); // for Testing
        // alternative charge behavior
        graph_->addVertex ("box", "pre_charge_line", 1, box_TO_pre_charge_line);
        graph_->addVertex ("pre_charge_line", "charge_line", 1, pre_charge_line_TO_charge_line);
        graph_->addVertex ("pre_charge_line", "box", 1, pre_charge_line_TO_box);
        graph_->addVertex ("charge_line", "pre_charge_line", 1, charge_line_TO_pre_charge_line);
    }

    /**
     * @brief Callback for new poses from AMCL. Used to store the pose for initialization on next start
     */
    void poseReceived (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p)
    {
        currentPose_ = *p;
    }


    /**
     * Stores the current position in the localisation file
     */
    void storePosition()
    {
        ROS_INFO ("Storing position");
        // Store last known position
        std::ofstream outputFile;
        outputFile.open ("localization.csv");

        outputFile << currentPose_.pose.pose.position.x << " ";
        outputFile << currentPose_.pose.pose.position.y << " ";
        outputFile << currentPose_.pose.pose.orientation.x << " ";
        outputFile << currentPose_.pose.pose.orientation.y << " ";
        outputFile << currentPose_.pose.pose.orientation.z << " ";
        outputFile << currentPose_.pose.pose.orientation.w << " ";

        outputFile.flush();
        outputFile.close();
    }

    void batteryCallback (std_msgs::Float32 battery)
    {
        battery_level_ = battery.data;
        std::cout<<battery_level_<<std::endl;
    }

private:
    ros::NodeHandle nh_, pNh_;
    ros::Publisher pub_status_, pub_current_node_, pub_initialize_, pub_deadman_;
    ros::ServiceClient srv_lineUntilQR_, srv_move_, srv_lineUntilLidar_, srv_detect_obstacles_;
    ros::ServiceServer srv_action_, srv_set_current_node_;
    ros::Subscriber sub_pose_;
    std::string srv_lineUntilQR_name_, srv_move_name_, pub_status_name_, srv_lineUntilLidar_name_,
        srv_action_name_, pub_current_node_name_, srv_set_current_node_name_,
	srv_detect_obstacles_name_, pub_deadman_name_;
    Skills skills_;
    Graph* graph_;
    std::vector<std::function<void() >> solution_;
    int search_limit_;
    std::string status;
    geometry_msgs::PoseWithCovarianceStamped currentPose_;

    ros::Subscriber sub_battery_;
    std::string sub_battery_name_;
    double battery_level_;
};


/**
 * Main
 */
int main (int argc, char** argv)
{
    // Init ROS Node
    ros::init (argc, argv, "mrNavigationController");
    NavigationController nc;
    ros::Rate rate (30);

    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Store position
    //nc.storePosition();

    // Return
    return 0;
}

