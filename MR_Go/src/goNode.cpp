/**
 * Moves the robot based on services
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>

#include "mr_go/move.h"
#include <boost/thread.hpp>

class Go
{
public:
    /**
     * Default constructor
     */
    Go()
    {
        // Get parameters
        nh_.param<double>("linear_speed", linear_speed_, 0.1);
        nh_.param<double>("theta_speed", theta_speed_, 0.1);
        

        // Get topics name
        nh_.param<std::string>("odometry", sub_odom_name_, "/odom");
        nh_.param<std::string>("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        nh_.param<std::string>("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        nh_.param<std::string>("srv_linear", srv_move_name_, "mrGo/move");

        // Publishers, subscribers, services
        nh_.subscribe<nav_msgs::Odometry>(sub_odom_name_, 1, &Go::odometryCallback, this);

        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);

        nh_.advertiseService(srv_move_name_, &Go::moveCallback, this);

        // Threads
        deadmanThread_ = new boost::thread(&Go::enableDeadman, this);
		
		// Twist stop
		twist_stop_msg_.twist.linear.x = 0;
		twist_stop_msg_.twist.angular.z = 0;
    }

    /**
     * Default constructor
     */
    ~Go()
    {
        deadmanThread_->interrupt();
    }

    /**
     * Odometry callback
     */
    void odometryCallback(const nav_msgs::Odometry odom)
    {
        odom_current_ = odom;
    }
    
    /**
	 * Move service callback
	 */
    bool moveCallback(mr_go::move::Request& req, mr_go::move::Response& res)
	{
		// Desired
		odom_desired_ = odom_current_;
		odom_desired_.twist.twist.linear.x += req.linear;
		odom_desired_.twist.twist.angular.z += req.angular;
		
		// Linear movement
		if (req.linear != 0){
			// Create the movement msg
			if (req.linear < 0) {
				twist_msg_.twist.linear.x = linear_speed_;
				twist_msg_.twist.angular.z = 0;
			} else {
				twist_msg_.twist.linear.x = -linear_speed_;
				twist_msg_.twist.angular.z = 0;
			}
			// Move the robot
			while(odom_current_.twist.twist.linear.x
				< odom_desired_.twist.twist.linear.x) {
				pub_twist_.publish(twist_msg_);
			}
			// Stop the robot
			pub_twist_.publish(twist_stop_msg_);
		}
		
		res.done = true;
		return true;
	}

    /**
     * Necessary to move the robot
     */
    void enableDeadman()
    {
        while(true)
        {
            try
            {
                msgs::BoolStamped deadman;
                deadman.data = true;
                deadman.header.stamp = ros::Time::now();
                pub_deadman_.publish(deadman);

                // Sleep
                usleep(50);    // Sleep for 50 ms = 20Hz

                // Signal interrupt point
                boost::this_thread::interruption_point();
            }
            catch (const boost::thread_interrupted&)
            {
                break;
            }
        }
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_deadman_, pub_twist_;
    ros::ServiceServer srv_move_;

    // Topics name
    std::string sub_odom_name_;
    std::string srv_move_name_;
    std::string pub_twist_name_, pub_deadman_name_;

    // Variables
    nav_msgs::Odometry odom_desired_;
    nav_msgs::Odometry odom_current_;
	geometry_msgs::TwistStamped twist_msg_;
	geometry_msgs::TwistStamped twist_stop_msg_;

    // Threads
    boost::thread* deadmanThread_;

    // Robot speed
    double theta_speed_;
    double linear_speed_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MR_Go");
    Go go;
    ros::Rate rate(30);
	ros::AsyncSpinner spinner(2);

    while(ros::ok())
        spinner.start();
    return 0;
}
