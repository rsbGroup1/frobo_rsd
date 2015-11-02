/**
 * Moves the robot based on services
 */

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>

#include "mr_go/move.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#define PI 3.14159265359
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

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
        nh_.param<double>("angular_speed", angular_speed_, 0.1);
        

        // Get topics name
        nh_.param<std::string>("odometry", sub_odom_name_, "/odom");
        nh_.param<std::string>("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        nh_.param<std::string>("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        nh_.param<std::string>("srv_move", srv_move_name_, "mr_go/move");

        // Publishers, subscribers, services
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(sub_odom_name_, 1, &Go::odometryCallback, this);

        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);

        srv_move_ = nh_.advertiseService(srv_move_name_, &Go::moveCallback, this);

        // Threads
        deadmanThread_ = new boost::thread(&Go::enableDeadman, this);
		stopDeadman();
		
		// Twist stop
		twist_stop_msg_.twist.linear.x = 0;
		twist_stop_msg_.twist.angular.z = 0;
		
		// Debug
		std::cout << "Node initiated!" << std::endl;
    }

    /**
     * Default constructor
     */
    ~Go()
    {
        stopDeadman();
    }

    /**
     * Odometry callback
     */
    void odometryCallback(const nav_msgs::Odometry odom)
    {
        linear_pos_current_ = odom.pose.pose.position.x;
		angular_pos_current_ = tf::getYaw(odom.pose.pose.orientation);
		// Correct the angle just in case the /odom adds 360 degrees
		if (std::abs(angular_pos_current_ - angular_pos_previous_) > PI)
			angular_pos_current_ -= PI*2;
		angular_pos_previous_ = angular_pos_current_;
    }
    
    /**
	 * Move service callback
	 */
    bool moveCallback(mr_go::move::Request& req, mr_go::move::Response& res)
	{
		// Deadman thread
		deadmanThread_->start_thread();
		// Desired
		double linear_desired = linear_pos_current_ + req.linear;
		double angle_desired = angular_pos_current_ + req.angular;
		// Twist msg
		twist_msg_.twist.linear.x = 0;
		twist_msg_.twist.angular.z = 0;
		
		// Move the robot
		double movement_precision = 0.005;
		while(std::abs(linear_desired - linear_pos_current_)
			> movement_precision
			|| std::abs(angle_desired - angular_pos_current_)
			> movement_precision)
		{
			// Create the movement msg
			if (std::abs(linear_desired - linear_pos_current_)
				> movement_precision){
				std::cout << "Linear distance: " << linear_desired - linear_pos_current_ << std::endl;
				if (req.linear > 0)
					twist_msg_.twist.linear.x = linear_speed_;
				else
					twist_msg_.twist.linear.x = -linear_speed_;
			}
			
			if (std::abs(angle_desired - angular_pos_current_)
				> movement_precision){
				std::cout << "Angular distance: " << (angle_desired - angular_pos_current_) * RAD_TO_DEG << std::endl;
				if (req.angular > 0)
					twist_msg_.twist.angular.z = angular_speed_;
				else
					twist_msg_.twist.angular.z = -angular_speed_;
			}
			// Publish the msg
			pub_twist_.publish(twist_msg_);
		}
		// Stop the robot
		pub_twist_.publish(twist_stop_msg_);
		stopDeadman();
		std::cout << "Achieved!" << std::endl;
		
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
				// Sleep for 50 ms = 20Hz
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
                // Signal interrupt point
                boost::this_thread::interruption_point();
            }
            catch (const boost::thread_interrupted&)
            {
                break;
            }
        }
    }
    
    /**
	 * Kills the thread when an interruption_point is found
	 */
    void stopDeadman()
	{
		deadmanThread_->interrupt();
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
	double linear_pos_current_;
	double angular_pos_current_;
	double angular_pos_previous_;
	geometry_msgs::TwistStamped twist_msg_;
	geometry_msgs::TwistStamped twist_stop_msg_;

    // Threads
    boost::thread* deadmanThread_;

    // Robot speed
    double angular_speed_;
    double linear_speed_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mr_go");
    Go go;
    ros::Rate rate(10);
	ros::AsyncSpinner spinner(3);

	while(!ros::isShuttingDown())
        spinner.start();
    return 0;
}
