/**
 * Moves the robot based on services
 */

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>

#include "mr_go/move.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define PI 3.14159265359
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

// Enum
enum HMI_ICONS
{
    tipper = 1,
    lineFollowing = 2,
    gps = 3,
    collectingBricks = 4,
    fixedMovement = 5,
    charging = 6,
};

class Go
{
public:
    /**
     * Default constructor
     */
    Go() : rate_(30)
    {
        ros::NodeHandle pNh_ ("~");

        // Get parameters
        pNh_.param<double> ("linear_speed", linear_speed_, 0.1);
        pNh_.param<double> ("angular_speed", angular_speed_, 0.1);
        pNh_.param<double> ("linear_precision", linear_precision_, 0.005);
        pNh_.param<double> ("angular_precision", angular_precision_, 0.500);

        // Get topics name
		pNh_.param<std::string> ("odometry", sub_odom_name_, "/odom"/*/fmKnowledge/pose*/);
        pNh_.param<std::string> ("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        pNh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        pNh_.param<std::string> ("srv_move", srv_move_name_, "mrGo/move");
        pNh_.param<std::string> ("pub_hmi", pub_hmi_name_, "/mrHMI/status");

        // Publishers, subscribers, services
        srv_move_ = nh_.advertiseService (srv_move_name_, &Go::moveCallback, this);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry> (sub_odom_name_, 1, &Go::odometryCallback, this);
        pub_hmi_ = nh_.advertise<std_msgs::String> (pub_hmi_name_, 10);

        // Twist stop
        twist_stop_msg_.twist.linear.x = 0;
        twist_stop_msg_.twist.angular.z = 0;
    }

    /**
     * Default constructor
     */
    ~Go()
    {
    }

    void HMIUpdateIcons(HMI_ICONS state)
    {
        std_msgs::String obj;
        obj.data = "00" + SSTR (state) + "0,,";
        pub_hmi_.publish (obj);
    }

    /**
     * Odometry callback
     */
    void odometryCallback (const nav_msgs::Odometry odom)
    {
        linear_pos_current_x_ = odom.pose.pose.position.x;
        linear_pos_current_y_ = odom.pose.pose.position.y;
        angular_pos_current_ = tf::getYaw (odom.pose.pose.orientation) * RAD_TO_DEG;

        // Correct the angle just in case the /odom adds 360 degrees
        if((angular_pos_current_ - angular_pos_previous_) > 180)
            angular_pos_current_ -= 360;
        else if((angular_pos_previous_ - angular_pos_current_) > 180)
            angular_pos_current_ += 360;

        angular_pos_previous_ = angular_pos_current_;
    }

    /**
     * Move service callback
     */
    bool moveCallback (mr_go::move::Request& req, mr_go::move::Response& res)
    {
        // Update HMI
        HMIUpdateIcons(fixedMovement);

        // Starts the publishers, subscribers and the deadman
        deadmanThread_ = new boost::thread (&Go::enableDeadman, this);

        // Twist msg
        twist_msg_.twist.linear.x = 0;
        twist_msg_.twist.angular.z = 0;

        // ONLY Linear movement
        if (req.linear != 0)
        {
            // Desired
            double linear_desired = req.linear;
            double distance_moved = 0.0;
            double start_x = linear_pos_current_x_;
            double start_y = linear_pos_current_y_;

            // Move the robot
            while ( (distance_moved - std::abs (linear_desired)) < linear_precision_)
            {
                // Create the movement msg
                //std::cout << "Linear distance: " << linear_desired - sqrt (pow ( (start_x - linear_pos_current_x_), 2.0) +
                //                       pow ( (start_y - linear_pos_current_y_), 2.0)) << std::endl;
                distance_moved = sqrt (pow ( (start_x - linear_pos_current_x_), 2.0) +
                                       pow ( (start_y - linear_pos_current_y_), 2.0));

				if (linear_desired - distance_moved  > 0)
                    twist_msg_.twist.linear.x = linear_speed_;
                else
                    twist_msg_.twist.linear.x = -linear_speed_;

                // Publish the msg
                pub_twist_.publish (twist_msg_);
                
                // Sleep
                rate_.sleep();
            }
        }

        // ONLY Angular movement
        else if (req.angular != 0)
        {
            // Change the sign
            req.angular = -req.angular;
            // Desired
            double angle_desired = angular_pos_current_ + req.angular;

            while (std::abs (angle_desired - angular_pos_current_) > angular_precision_)
            {
                // Move the robot
                std::cout << "Angular distance: " << (angular_pos_current_ - angle_desired) << std::endl;
				if (angle_desired - angular_pos_current_ > 0)
                    twist_msg_.twist.angular.z = angular_speed_;
                else
                    twist_msg_.twist.angular.z = -angular_speed_;

                // Publish the msg
                pub_twist_.publish (twist_msg_);

                // Sleep
                rate_.sleep();
            }
        }

        // Stop the robot and the deadman
        stopDeadman();
        pub_twist_.publish (twist_stop_msg_);
        delete deadmanThread_;

        // Update HMI
        HMIUpdateIcons(fixedMovement);

        // Return
        res.done = true;
        return true;
    }

    /**
     * Necessary to move the robot
     */
    void enableDeadman()
    {
        while (true)
        {
            try
            {
                msgs::BoolStamped deadman;
                deadman.data = true;
                deadman.header.stamp = ros::Time::now();
                pub_deadman_.publish (deadman);

                // Sleep for 50 ms = 20Hz
                boost::this_thread::sleep_for (boost::chrono::milliseconds (50));

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
    ros::Publisher pub_deadman_, pub_twist_, pub_hmi_;
    ros::ServiceServer srv_move_;

    // Topics name
    std::string sub_odom_name_;
    std::string srv_move_name_;
    std::string pub_twist_name_, pub_deadman_name_, pub_hmi_name_;

    // Variables
    double linear_pos_current_x_;
    double linear_pos_current_y_;
    double angular_pos_current_;
    double angular_pos_previous_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::TwistStamped twist_stop_msg_;

    // Threads
    boost::thread* deadmanThread_;

    // Robot speed
    double angular_speed_;
    double linear_speed_;
    double linear_precision_;
    double angular_precision_;

    // Rate
    ros::Rate rate_;
};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "MR_Go");
    Go go;
    ros::Rate rate (30);

    ros::AsyncSpinner spinner (0);

    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
    {
    	spinner.start();
        //ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
