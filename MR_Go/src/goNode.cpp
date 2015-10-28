/**
 * Moves the robot based on services
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>

#include "mr_go/angularMove.h"
#include "mr_go/linearMove.h"
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
        nh_.param<std::string>("srv_linear", srv_linear_name_, "mrGo/linearMove");
        nh_.param<std::string>("srv_angular", srv_angular_name_, "mrGo/angularMove");

        // Publishers, subscribers, services
        nh_.subscribe<nav_msgs::Odometry>(sub_odom_name_, 1, &Go::odometryCallback, this);

        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);

        nh_.advertiseService(srv_linear_name_, &Go::linearCallback, this);
        nh_.advertiseService(srv_angular_name_, &Go::angularCallback, this);

        // Threads
        deadmanThread_ = new boost::thread(&Go::enableDeadman, this);
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
        odom_current = odom;
    }

    /**
     * Linear service callback
     */
    bool linearCallback(mr_go::linearMove::Request& req, mr_go::linearMove::Response& res)
    {
        /*odom_start = odom_current;
        odom_desired = odom_start + req.distance;

        while(odom_desired != odom_current);*/

        return true;
    }

    /**
     * Angular service callback
     */
    bool angularCallback(mr_go::angularMove::Request& req, mr_go::angularMove::Response& res)
    {
        // Turn the amount of req.angle
        // return status to res.done
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
    ros::ServiceServer srv_linear_, srv_angular_;

    // Topics name
    std::string sub_odom_name_;
    std::string srv_linear_name_, srv_angular_name_;
    std::string pub_twist_name_, pub_deadman_name_;

    // Variables
    nav_msgs::Odometry odom_start;
    nav_msgs::Odometry odom_desired;
    nav_msgs::Odometry odom_current;

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

    while(ros::ok())
        ros::spin();

    return 0;
}
