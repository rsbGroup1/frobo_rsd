// Includes
#include "skills.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "msgs/BoolStamped.h"

// Defines
#define M_PI		3.14159265358979323846
#define DEG_TO_RAD	(M_PI/180.0)
#define RAD_TO_DEG	(180.0/M_PI)

Skills::Skills (ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move, ros::ServiceClient* srv_lineUntilLidar, ros::ServiceClient* srv_lineUntilRelative,
			ros::Publisher* pub_status, ros::Publisher* pub_initialize, ros::Publisher *pub_deadman,
            ros::ServiceClient* srv_detect_obstacles, ros::Publisher* pub_hmi
 	      )
{
    pub_hmi_ = pub_hmi;
    srv_lineUntilQR_ = srv_lineUntilQR;
    srv_lineUntilLidar_ = srv_lineUntilLidar;
    srv_lineUntilRelative_ = srv_lineUntilRelative;
    srv_move_ = srv_move;
    pub_status_ = pub_status;
    srv_detect_obstacles_ = srv_detect_obstacles;
    pub_initialize_ = pub_initialize;
    pub_deadman_ = pub_deadman;
    move_base_actionclient_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);
}

Skills::~Skills()
{
    delete move_base_actionclient_;
}

void Skills::HMIUpdateIcons(HMI_ICONS state)
{
    std_msgs::String obj;
    obj.data = "00" + SSTR (state) + "0,,";
    pub_hmi_->publish (obj);
}


bool Skills::lineUntilQR (std::string qr)
{
    std::cout << "Skill: Line until QR: " << qr << std::endl;
    lineFollowerCall.request.qr = qr;
    lineFollowerCall.request.time_limit = 300;
    srv_lineUntilQR_->call (lineFollowerCall);

    std_msgs::String msg;
    //msg.data = "following_line " + qr;
    msg.data = "following_line";
    pub_status_->publish (msg);
    return lineFollowerCall.response.success;
}

bool Skills::lineUntilLidar (double distance)
{
    std::cout << "Skill: Line until Lidar distance: " << distance << std::endl;
    followUntilLidarCall.request.lidar_distance = distance;
    followUntilLidarCall.request.time_limit = 300;
    srv_lineUntilLidar_->call (followUntilLidarCall);

    std_msgs::String msg;
    //msg.data = "following_line " + qr;
    msg.data = "following_line";
    pub_status_->publish (msg);
    return followUntilLidarCall.response.success;
}

bool Skills::lineUntilRelative (double distance)
{
    std::cout << "Skill: Line until Relative distance: " << distance << std::endl;
    followUntilRelativeCall.request.relative_distance = distance;
    followUntilRelativeCall.request.time_limit = 300;
    srv_lineUntilRelative_->call (followUntilRelativeCall);

    std_msgs::String msg;
    //msg.data = "following_line " + qr;
    msg.data = "following_line";
    pub_status_->publish (msg);
    return followUntilRelativeCall.response.success;
}

bool Skills::linearMove (double distance)
{
    std::cout << "Skill: Linear move for: " << distance << " m" << std::endl;
    move_call_.request.linear = distance;
    move_call_.request.angular = 0;
    srv_move_->call (move_call_);
    
    std_msgs::String msg;
    //msg.data = "linear_move " + std::to_string(distance);
    msg.data = "linear_move";
    pub_status_->publish (msg);
    return move_call_.response.done;
}

bool Skills::angularMove (double angle)
{
    std::cout << "Skill: Angular move for: " << angle << " degrees" << std::endl;
    move_call_.request.linear = 0;
    move_call_.request.angular = angle;
    srv_move_->call (move_call_);

    std_msgs::String msg;
    //msg.data = "angular_move " + std::to_string(angle);
    msg.data = "angular_move";
    pub_status_->publish (msg);
    return move_call_.response.done;
}

bool Skills::goToFreePosition (double x, double y, double yaw)
{
    // Update HMI
    HMIUpdateIcons(gps);

    //std::cout << "go to free position called" << std::endl;
    ROS_INFO ("Go to free position called - goal(%f, %f, %f)", x, y, yaw);
    bool success = false;

    deadmanThread_ = new boost::thread (&Skills::enableDeadman, this);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (yaw);
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    if (move_base_actionclient_->waitForServer (ros::Duration (5, 0)))
    {
        move_base_actionclient_->sendGoal (goal);
        bool finished = move_base_actionclient_->waitForResult();
        // DSW TEsting: move to recovery position then try again
        move_base_msgs::MoveBaseGoal recovery;

        recovery.target_pose.pose.position.x = 0.46;
        recovery.target_pose.pose.position.y = 1.58;
        recovery.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (-1.9);
        recovery.target_pose.header.frame_id = "map";
        recovery.target_pose.header.stamp = ros::Time::now();

        while (move_base_actionclient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN ("To recovery=>goal");
            move_base_actionclient_->sendGoal (recovery);
            finished = move_base_actionclient_->waitForResult();
            move_base_actionclient_->sendGoal (goal);
            finished = move_base_actionclient_->waitForResult();
        }

        success = true;
        std_msgs::String msg;
        msg.data = "Free navigation to X:" + std::to_string (x) + ", Y:" +  std::to_string (y);
        pub_status_->publish (msg);

    }
    else
    {
        ROS_ERROR ("move_base action server not responding within timeout");
    }

    // Clear
    deadmanThread_->interrupt();

    // Update HMI
    HMIUpdateIcons(gps);

    // Return
    return success;
}

bool Skills::setInitialPoseAMCL (double x, double y, double yaw)
{
	ROS_INFO ("setting initial pose for AMCL to(%f, %f, %f)", x, y, yaw);
	std::cout << "setting initial pose for AMCL" << std::endl;

	geometry_msgs::PoseWithCovarianceStamped p;
	p.pose.pose.position.x = x;
	p.pose.pose.position.y = y;
	p.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	p.header.frame_id = "map";

	pub_initialize_->publish(p);
	return true;
}

bool Skills::detectObstacles (bool state)
{
	detectObstaclesCall.request.enable = state;
	srv_detect_obstacles_->call(detectObstaclesCall);
	return detectObstaclesCall.response.done;
}

void Skills::enableDeadman()
{
    while (true)
    {
        try
        {
            msgs::BoolStamped deadman;
            deadman.data = true;
            deadman.header.stamp = ros::Time::now();
            pub_deadman_->publish (deadman);
            // Sleep for 50 ms = 20Hz
            boost::this_thread::sleep_for (boost::chrono::milliseconds (75));
            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch (const boost::thread_interrupted&)
        {
            break;
        }
    }
}

bool Skills::chargeDectectionAndBackupPlan(double* battery_level, double threshold)
{   
	bool keep_trying = true;
	int tries = 0;
	std::cout << "battery START: " << *battery_level << std::endl;
		
    while (keep_trying)
    {
        //if (battery_level < threshold)
        //{
		//linearMove(-0.2);
		goToFreePosition(-0.56, -2.41 , -0.2);
		lineUntilLidar(0.45);
		linearMove(0.25);
		wait(10.0);
		ros::spinOnce();
		wait(10.0);
		ros::spinOnce();
		std::cout << "battery: " << *battery_level << std::endl;
        if (*battery_level > threshold || tries > 1)
            keep_trying = false;
        else
        {
			tries++;
			linearMove(-0.25);			
		}
	}

	return true;
}

bool Skills::wait(double seconds)
{
    // 10 hz
    ros::Rate r(1/seconds);
    r.sleep();
	return true;
}
