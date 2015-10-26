/**
 * Moves the robot based on services
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>

#include "mr_go/forward.h"
#include "mr_go/backward.h"
#include "mr_go/left.h"
#include "mr_go/right.h"

#include <boost/thread.hpp>

class Go {
public:
    /**
     * Default constructor
     */
    Go() {
        // Get parameters
        nh_.param<double> ("linear_speed", linear_speed_, 0.1);
        nh_.param<double> ("theta_speed", theta_speed_, 0.1);
        // Get topics name
        nh_.param<std::string> ("odometry", sub_odom_name_, "/odom");
        nh_.param<std::string> ("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        nh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        nh_.param<std::string> ("srv_backward", srv_backward_name_, "/mr_go/backward");
        nh_.param<std::string> ("srv_forward", srv_forward_name_, "mr_go/forward");
        nh_.param<std::string> ("srv_left", srv_left_name_, "mr_go/left");
        nh_.param<std::string> ("srv_right", srv_right_name_, "mr_go/right");
        // Publishers, subscribers, services
        nh_.subscribe<nav_msgs::Odometry> (sub_odom_name_, 1, &Go::odometryCallback, this);

        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);

        nh_.advertiseService(srv_forward_name_, &Go::forwardCallback, this);
        nh_.advertiseService(srv_backward_name_, &Go::backwardCallback, this);
        nh_.advertiseService(srv_left_name_, &Go::leftCallback, this);
        nh_.advertiseService(srv_right_name_, &Go::rightCallback, this);
        //Threads
        deadmanThread_ = new boost::thread(&Go::enableDeadman, this);
    }
    /**
     * Default constructor
     */
    ~Go() {
        deadmanThread_->interrupt();
    }
    /**
     * Odometry callback
     */
    void odometryCallback(const nav_msgs::Odometry odom) {
        odom_current = odom;
    }
    /**
     * Forward service callback
     */
    bool forwardCallback(mr_go::forward::Request& req, mr_go::forward::Response& res) {
        if(moveForward(req.distance))
            res.done = true;
        return true;
    }
    /**
     * Backward service callback
     */
    bool backwardCallback(mr_go::backward::Request& req, mr_go::backward::Response& res) {
        if(moveBackward(req.distance))
            res.done = true;
        return true;
    }
    /**
     * Left service callback
     */
    bool leftCallback(mr_go::left::Request& req, mr_go::left::Response& res) {
        if(turnLeft(req.angle))
            res.done = true;
        return true;
    }
    /**
     * Right service callback
     */
    bool rightCallback(mr_go::right::Request& req, mr_go::right::Response& res) {
        if(turnRight(req.angle))
            res.done = true;
        return true;
    }
    /**
     * Moves the robot forward a defined distance
     */
    bool moveForward(double distance) {

    }
    /**
     * Moves the robot backward a defined distance
     */
    bool moveBackward(double distance) {

    }
    /**
     * Turns the robot left a defined distance
     */
    bool turnLeft(double angle) {

    }
    /**
     * Turns the robot right a defined distance
     */
    bool turnRight(double angle) {

    }
    /**
     * Necessary to move the robot
     */
    void enableDeadman() {
        while(true) {
            try {
                msgs::BoolStamped deadman;
                deadman.data = true;
                deadman.header.stamp = ros::Time::now();
                pub_deadman_.publish(deadman);
                // Sleep
                usleep(50);    // Sleep for 50 ms = 20Hz
                // Signal interrupt point
                boost::this_thread::interruption_point();
            } catch
                (const boost::thread_interrupted&) {
                break;
            }
        }
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_deadman_, pub_twist_;;
    ros::ServiceServer srv_forward_, srv_backward_, srv_left_, srv_right_;
    // Topics name
    std::string sub_odom_name_;
    std::string srv_forward_name_, srv_backward_name_, srv_left_name_, srv_right_name_;
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "mr_go");
    Go go;
    ros::Rate rate(30);
    while(ros::ok())
        ros::spin();

    return 0;
}
