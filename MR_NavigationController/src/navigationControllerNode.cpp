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
#include "mr_line_follower/followUntilLidar.h"
#include "mr_line_follower/followUntilRelative.h"
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

class NavigationController
{
public:
    /**
     * Constructors
     */
    NavigationController() :
        pNh_ ("~"),
        skills_ (&srv_lineUntilQR_, &srv_move_, &srv_lineUntilLidar_, &srv_lineUntilRelative_, &pub_status_,
         &pub_initialize_, &pub_deadman_, &srv_detect_obstacles_, &pub_hmi_)
    {
        // Get parameter names
        pNh_.param<std::string> ("lineFollowEnableService", srv_lineUntilQR_name_, "/mrLineFollower/lineUntilQR");
        pNh_.param<std::string> ("lineFollowEnableLidarService", srv_lineUntilLidar_name_, "/mrLineFollower/lineUntilLidar");
        pNh_.param<std::string> ("lineFollowEnableRelativeService", srv_lineUntilRelative_name_, "/mrLineFollower/lineUntilRelative");
        pNh_.param<std::string> ("moveService", srv_move_name_, "/mrGo/move");
        pNh_.param<std::string> ("performAction", srv_action_name_, "/mrNavigationController/performAction");
        pNh_.param<std::string> ("status", pub_status_name_, "/mrNavigationController/status");
        pNh_.param<std::string> ("currentNode", pub_current_node_name_, "/mrNavigationController/currentNode");
        pNh_.param<std::string> ("setCurrentNode", srv_set_current_node_name_, "/mrNavigationController/setCurrentNode");
		pNh_.param<std::string> ("obstacleDetectorService", srv_detect_obstacles_name_, "/mrObstacleDetector/enabler");
        pNh_.param<int> ("searchLimit", search_limit_, 100);
        pNh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        pNh_.param<std::string> ("sub_battery", sub_battery_name_, "/fmInformation/battery");
        pNh_.param<std::string> ("pub_hmi", pub_hmi_name_, "/mrHMI/status");
		pNh_.param<std::string> ("start_node", start_node_, "charge");

        // Service
        srv_lineUntilQR_ = nh_.serviceClient<mr_line_follower::followUntilQR> (srv_lineUntilQR_name_);
        srv_lineUntilLidar_ = nh_.serviceClient<mr_line_follower::followUntilLidar> (srv_lineUntilLidar_name_);
        srv_lineUntilRelative_ = nh_.serviceClient<mr_line_follower::followUntilRelative> (srv_lineUntilRelative_name_);
        srv_move_ = nh_.serviceClient<mr_go::move> (srv_move_name_);
        srv_action_ = nh_.advertiseService (srv_action_name_, &NavigationController::performActionCallback, this);
        srv_set_current_node_ = nh_.advertiseService (srv_set_current_node_name_, &NavigationController::setCurrentNodeCallback, this);
        srv_detect_obstacles_ = nh_.serviceClient<mr_obstacle_detector::enabler>(srv_detect_obstacles_name_);

        // Subscriber
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("amcl_pose", 10, &NavigationController::poseReceived, this);
        sub_battery_ = nh_.subscribe<std_msgs::Float32> (sub_battery_name_, 10, &NavigationController::batteryCallback, this);

        // Publisher
        pub_status_ = nh_.advertise<std_msgs::String> (pub_status_name_, 10);
        pub_current_node_ = nh_.advertise<std_msgs::String> (pub_current_node_name_, 10);
        pub_initialize_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose", 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);
        pub_hmi_ = nh_.advertise<std_msgs::String> (pub_hmi_name_, 10);

        // Create the graph and put the start node from the launch file
        graph_ = new Graph (&pub_current_node_);
        graph_->setCurrentNode (start_node_);
        createGraph();
        //graph_->showGraph();

        // Inialize AMCL
        std::ifstream localisationFile;
        localisationFile.open ("/home/frobit_pro_group1/Desktop/localization.csv");
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
            for(int i = 0; i < 36 ;i+=7)
		          p.pose.covariance[i] = 0.3;

            p.header.frame_id = "map";

            std::cout << "FROM LOADED FILE: " << p.pose.pose.position.x << " " << p.pose.pose.position.y << " " << p.pose.pose.orientation.x << " " << p.pose.pose.orientation.y << " " << p.pose.pose.orientation.z << " "  << p.pose.pose.orientation.w << std::endl;

            ros::Duration d (5, 0);
            d.sleep();
            pub_initialize_.publish (p);

            localisationFile.close();
        }
        else
        {
            // Global initialization
            ros::ServiceClient initalize = nh_.serviceClient<std_srvs::Empty> ("global_localization");
            ros::Duration d (5, 0);
            d.sleep();
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
            if (node.getName() == req.node.c_str())
                node_exists = true;
        
        // Check if it is already in the node
        if (req.node.c_str() == graph_->getCurrentNode())
            already_here = true;

        // Handle the output
        if (node_exists == true)
        {
            if (already_here)
            {
                ROS_INFO ("I am already here!");
				return true;
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
        graph_->addNode ("charge");

        /*
         *
         * Skills
         *
         */
        std::vector<std::function<void() >> line_start_TO_wc1;
        line_start_TO_wc1.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        line_start_TO_wc1.push_back (std::bind (&Skills::setInitialPoseAMCL, &skills_, 13.87, -11.164, 0.149));
        line_start_TO_wc1.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_entrance"));
        line_start_TO_wc1.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1"));
		line_start_TO_wc1.push_back (std::bind (&Skills::detectObstacles, &skills_, false));

        std::vector<std::function<void() >> wc1_TO_wc1_conveyor;
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::linearMove, &skills_, 0.6));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, -90));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::lineUntilRelative, &skills_, 0.11));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Skills::lineUntilLidar, &skills_, 0.20));
        wc1_TO_wc1_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1_conveyor"));

        std::vector<std::function<void() >> wc1_conveyor_TO_wc1_robot;
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::linearMove, &skills_, -0.4));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::angularMove, &skills_, -50));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::linearMove, &skills_, 0.4));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::angularMove, &skills_, 30));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Skills::lineUntilLidar, &skills_, 0.20));
        wc1_conveyor_TO_wc1_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc1_robot"));

        std::vector<std::function<void() >> wc1_robot_TO_wc_exit;
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::linearMove, &skills_, -0.3));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, -180));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_1_exit"));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Skills::linearMove, &skills_, 0.5));
        wc1_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc_exit"));
		
        std::vector<std::function<void() >> wc1_TO_wc2;
        wc1_TO_wc2.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_entrance"));
        wc1_TO_wc2.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2"));

        std::vector<std::function<void() >> wc2_TO_wc2_conveyor;
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::linearMove, &skills_, 0.6));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, -90));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::lineUntilRelative, &skills_, 0.11));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Skills::lineUntilLidar, &skills_, 0.15));
        wc2_TO_wc2_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2_conveyor"));

        std::vector<std::function<void() >> wc2_conveyor_TO_wc2_robot;
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::linearMove, &skills_, -0.35));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::angularMove, &skills_, 50));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::linearMove, &skills_, 0.6));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::angularMove, &skills_, -35));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Skills::lineUntilLidar, &skills_, 0.13));
        wc2_conveyor_TO_wc2_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc2_robot"));

        std::vector<std::function<void() >> wc2_robot_TO_wc_exit;
        wc2_robot_TO_wc_exit.push_back (std::bind (std::bind (&Skills::linearMove, &skills_, -0.55)));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, 180));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_2_exit"));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc_exit"));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));
        wc2_robot_TO_wc_exit.push_back (std::bind (&Skills::linearMove, &skills_, 0.55));

        std::vector<std::function<void() >> wc2_TO_wc3;
        wc2_TO_wc3.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_entrance"));
        wc2_TO_wc3.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3"));

        std::vector<std::function<void() >> wc3_TO_wc3_conveyor;
        wc3_TO_wc3_conveyor.push_back (std::bind (&Skills::angularMove, &skills_, -90));
        wc3_TO_wc3_conveyor.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_conveyor"));
        wc3_TO_wc3_conveyor.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3_conveyor"));

        std::vector<std::function<void() >> wc3_conveyor_TO_wc3_robot;
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Skills::angularMove, &skills_, -90));
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_load"));
        wc3_conveyor_TO_wc3_robot.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc3_robot"));

        std::vector<std::function<void() >> wc3_robot_TO_wc_exit;
        wc3_robot_TO_wc_exit.push_back (std::bind (std::bind (&Skills::linearMove, &skills_, -0.2)));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::angularMove, &skills_, 180));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::lineUntilQR, &skills_, "wc_3_exit"));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Graph::setCurrentNode, graph_, "wc_exit"));
        wc3_robot_TO_wc_exit.push_back (std::bind (&Skills::detectObstacles, &skills_, false));

        std::vector<std::function<void() >> wc_exit_TO_line_end;
        wc_exit_TO_line_end.push_back (std::bind (&Skills::angularMove, &skills_, 90));
        wc_exit_TO_line_end.push_back (std::bind (&Skills::detectObstacles, &skills_, true));
        //wc_exit_TO_line_end.push_back (std::bind (&Skills::lineUntilQR, &skills_, "line_out"));
        wc_exit_TO_line_end.push_back (std::bind (&Skills::lineUntilRelative, &skills_, 7.5));
        wc_exit_TO_line_end.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_end"));
        wc_exit_TO_line_end.push_back (std::bind (&Skills::detectObstacles, &skills_, false));

        std::vector<std::function<void() >> line_end_TO_box;
        line_end_TO_box.push_back (std::bind (&Skills::setInitialPoseAMCL, &skills_, 3.615, -2.046, 2.115));
        line_end_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.1, 0.7 , -1.9));
        line_end_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.5, -1.5 , -1.9));
        line_end_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        std::vector<std::function<void() >> line_end_TO_line_start;
        line_end_TO_line_start.push_back (std::bind (&Skills::setInitialPoseAMCL, &skills_, 3.615, -2.046, 2.115));
        line_end_TO_line_start.push_back (std::bind (&Skills::goToFreePosition, &skills_, 2.89, -0.66 , -1.69));
        line_end_TO_line_start.push_back (std::bind (&Skills::goToFreePosition, &skills_, 3.3, -2 , -1.2));
        line_end_TO_line_start.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_start"));

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
        pre_bricks_TO_bricks.push_back (std::bind (&Skills::linearMove, &skills_, 0.2));
        pre_bricks_TO_bricks.push_back (std::bind (&Skills::wait, &skills_, 10));
        pre_bricks_TO_bricks.push_back (std::bind (&Graph::setCurrentNode, graph_, "bricks"));
        std::vector<std::function<void() >> bricks_TO_pre_bricks;
        bricks_TO_pre_bricks.push_back (std::bind (&Skills::linearMove, &skills_, -0.2));
        bricks_TO_pre_bricks.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_bricks"));

        std::vector<std::function<void() >> box_TO_pre_charge;
        //box_TO_pre_charge.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.56, -2.41 , -0.2));
        //box_TO_pre_charge.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.02, -2.51 , -0.2));
        box_TO_pre_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge"));

        std::vector<std::function<void() >> pre_charge_TO_box;
        pre_charge_TO_box.push_back (std::bind (&Skills::goToFreePosition, &skills_, -0.8, -2.1 , -0.6));
        pre_charge_TO_box.push_back (std::bind (&Graph::setCurrentNode, graph_, "box"));

        std::vector<std::function<void() >> pre_charge_TO_LineStart;
        pre_charge_TO_LineStart.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.46, 1.58 , -1.9));
        pre_charge_TO_LineStart.push_back (std::bind (&Skills::goToFreePosition, &skills_, 3.3, -2 , -1.2));
        pre_charge_TO_LineStart.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_start"));

        std::vector<std::function<void() >> pre_bricks_TO_LineStart;
        pre_bricks_TO_LineStart.push_back (std::bind (&Skills::goToFreePosition, &skills_, 0.46, 1.58 , -1.9));
        pre_bricks_TO_LineStart.push_back (std::bind (&Skills::goToFreePosition, &skills_, 3.3, -2 , -1.2));
        pre_bricks_TO_LineStart.push_back (std::bind (&Graph::setCurrentNode, graph_, "line_start"));

        std::vector<std::function<void() >> pre_charge_TO_charge;
        //pre_charge_TO_charge.push_back (std::bind (&Skills::linearMove, &skills_, 0.1));
        //pre_charge_TO_charge.push_back (std::bind (&Skills::wait, &skills_, 5.0));
        pre_charge_TO_charge.push_back (std::bind (&Skills::chargeDectectionAndBackupPlan, &skills_, &battery_level_, 12.8));
        pre_charge_TO_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "charge"));

        std::vector<std::function<void() >> charge_TO_pre_charge;
        charge_TO_pre_charge.push_back (std::bind (&Skills::linearMove, &skills_, -0.4));
        charge_TO_pre_charge.push_back (std::bind (&Graph::setCurrentNode, graph_, "pre_charge"));


        // Vertices
        graph_->addVertex ("line_start", "wc1", 1, line_start_TO_wc1);
        //graph_->addVertex ("wc1", "wc2", 1, wc1_TO_wc2);
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
	graph_->addVertex ("line_end", "line_start", 1, line_end_TO_line_start);
        graph_->addVertex ("box", "line_start", 1, box_TO_line_start);
        graph_->addVertex ("pre_charge", "line_start", 1, pre_charge_TO_LineStart);
        graph_->addVertex ("pre_bricks", "line_start", 1, pre_bricks_TO_LineStart);

        graph_->addVertex ("box", "pre_charge", 1, box_TO_pre_charge);
        graph_->addVertex ("pre_charge", "box", 1, pre_charge_TO_box);

        graph_->addVertex ("box", "pre_bricks", 1, box_TO_pre_bricks);
        graph_->addVertex ("pre_bricks", "box", 1, pre_bricks_TO_box);

        graph_->addVertex ("pre_charge", "charge", 1, pre_charge_TO_charge);
        graph_->addVertex ("charge", "pre_charge", 1, charge_TO_pre_charge);

        graph_->addVertex ("pre_bricks", "bricks", 1, pre_bricks_TO_bricks);
        graph_->addVertex ("bricks", "pre_bricks", 1, bricks_TO_pre_bricks);

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
        // Store last known position
        ROS_INFO ("Storing position");
        if(!(currentPose_.pose.pose.position.x != currentPose_.pose.pose.position.x || currentPose_.pose.pose.position.y != currentPose_.pose.pose.position.y || currentPose_.pose.pose.orientation.x != currentPose_.pose.pose.orientation.x || currentPose_.pose.pose.orientation.y != currentPose_.pose.pose.orientation.y || currentPose_.pose.pose.orientation.z != currentPose_.pose.pose.orientation.z || currentPose_.pose.pose.orientation.w != currentPose_.pose.pose.orientation.w))
        {
            std::ofstream outputFile;
            outputFile.open ("/home/frobit_pro_group1/Desktop/localization.csv");

            outputFile << currentPose_.pose.pose.position.x << " ";
            outputFile << currentPose_.pose.pose.position.y << " ";
            outputFile << currentPose_.pose.pose.orientation.x << " ";
            outputFile << currentPose_.pose.pose.orientation.y << " ";
            outputFile << currentPose_.pose.pose.orientation.z << " ";
            outputFile << currentPose_.pose.pose.orientation.w << " ";

            outputFile.flush();
            outputFile.close();
        }
    }

    void batteryCallback (std_msgs::Float32 battery)
    {
        battery_level_ = battery.data;
       // std::cout<<battery_level_<<std::endl;
    }

private:
    ros::NodeHandle nh_, pNh_;
    ros::Publisher pub_status_, pub_current_node_, pub_initialize_, pub_deadman_, pub_hmi_;
    ros::ServiceClient srv_lineUntilQR_, srv_move_, srv_lineUntilLidar_, srv_lineUntilRelative_, srv_detect_obstacles_;
    ros::ServiceServer srv_action_, srv_set_current_node_;
    ros::Subscriber sub_pose_;
    std::string srv_lineUntilQR_name_, srv_move_name_, pub_status_name_, srv_lineUntilLidar_name_, srv_lineUntilRelative_name_,
        srv_action_name_, pub_current_node_name_, srv_set_current_node_name_,
    srv_detect_obstacles_name_, pub_deadman_name_, pub_hmi_name_;
    Skills skills_;
    Graph* graph_;
    std::vector<std::function<void() >> solution_;
    int search_limit_;
	std::string status, start_node_;
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
    ros::init (argc, argv, "MR_Navigation_Controller");
    NavigationController nc;
    ros::Rate rate (30);

    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Store position
    nc.storePosition();

    // Return
    return 0;
}

