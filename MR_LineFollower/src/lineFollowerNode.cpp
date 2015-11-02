/**
 * Receive the processed information from the camera and
 * use it to move the robot
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "msgs/BoolStamped.h"

#include "mr_line_follower/enable.h"

#include <iostream>
#include <string>
#include <boost/thread.hpp>

class lineFollower
{
public:
    /**
     * Default constructor
     */
    lineFollower()
    {
        // Get parameters
        nh_.param<double> ("pid_p", pid_p_, 0.5);
        nh_.param<double> ("pid_i", pid_i_, 0);
        nh_.param<double> ("pid_d", pid_d_, 0);
        nh_.param<double> ("pid_dt", pid_dt_, 0.33);   // 1/rate
        nh_.param<double> ("pid_max", pid_max_, 320);
        nh_.param<double> ("pid_min", pid_min_, -320);
        nh_.param<int> ("reference_point_x", reference_point_x_, 320);
        nh_.param<int> ("reference_point_y", reference_point_y_, 240);
        nh_.param<double> ("robot_speed", robot_speed_, 0.1);

        // Get topics name
        nh_.param<std::string> ("sub_line", sub_line_name_, "/mr_camera_processing/line");
        nh_.param<std::string> ("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        nh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        nh_.param<std::string> ("srv_enable", srv_enable_name_, "/mrLineFollower/enable");

        // Publishers, subscribers, services
        sub_line_ = nh_.subscribe<geometry_msgs::Point> (sub_line_name_, 1, &lineFollower::lineCallback, this);
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);
        srv_enable_ = nh_.advertiseService(srv_enable_name_, &lineFollower::enableCallback, this);

        // Threads
        deadmanThread_ = new boost::thread(&lineFollower::enableDeadman, this);
    }

    /**
     * Default destructor
     */
    ~lineFollower()
    {
        deadmanThread_->interrupt();
    }

    /**
     * Uses the received point into a PID to move the robot
     * Point: negative->left, positive-> right
     */
    void lineCallback(const geometry_msgs::Point detected_point)
    {
        static double pre_error = 0;
        static double integral = 0;

        // Check if it is enabled
        if(enable_)
        {
            /*
             * PID
             */
            geometry_msgs::Point reference_point;
            reference_point.x = reference_point_x_;
            reference_point.y = reference_point_y_;

            // Calculate error
            double pid_error = reference_point.x - detected_point.x;

            // Proportional term
            double Pout = pid_p_ * pid_error;

            // Integral term
            integral += pid_error * pid_dt_;
            double Iout = pid_i_ * integral;

            // Derivative term
            double derivative = (pid_error - pre_error) / pid_dt_;
            double Dout = pid_d_ * derivative;

            // Calculate total output
            double pid_output = Pout + Iout + Dout;

            // Restrict to max/min
            if(pid_output > pid_max_)
                pid_output = pid_max_;
            else
                if(pid_output < pid_min_)
                    pid_output = pid_min_;

            // Save error to previous error
            pre_error = pid_error;

            //std::cout << std::endl;
            //std::cout << "Error PID: " << pid_error << std::endl;
            //std::cout << "Output PID: " << pid_output << std::endl;
            //std::cout << "DTime PID: " << pid_dt << std::endl;
            //std::cout << std::endl;

            /*
             * Robot movement
             */
            // Define the relation between the error and theta
            geometry_msgs::TwistStamped twistStamp_msg;
            const double max_theta = 0.8;
            double theta = max_theta * pid_output / pid_max_;
            // Publish the message
            twistStamp_msg.header.stamp = ros::Time::now();
            twistStamp_msg.twist.linear.x = robot_speed_;
            twistStamp_msg.twist.angular.z = theta;
            pub_twist_.publish(twistStamp_msg);

            std::cout << "Speed: " << robot_speed_ << " | Theta: " << theta << std::endl;
        }
        else
        {
            pre_error = 0;
            integral = 0;
        }
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
            catch(const boost::thread_interrupted&)
            {
                break;
            }
        }
    }

    /**
     * Returns the frecuency of the Node
     */
    double getFrequency()
    {
        return 1 / pid_dt_;
    }

    /**
     * Enables or disables the line following
     */
    bool enableCallback(mr_line_follower::enable::Request& req, mr_line_follower::enable::Response& res)
    {
        if(req.enable == true)
            enable_ = true;
        else
            enable_ = false;

        res.status = enable_;
        return true;
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_deadman_;
    ros::ServiceServer srv_enable_;
    // Threads
    boost::thread* deadmanThread_;
    // PID
    double pid_dt_;
    double pid_max_;
    double pid_min_;
    double pid_p_;
    double pid_d_;
    double pid_i_;
    // Reference point
    int reference_point_x_;
    int reference_point_y_;
    // Robot
    double robot_speed_;
    // Topics name
    std::string sub_line_name_;
    std::string pub_deadman_name_, pub_twist_name_;
    std::string srv_enable_name_;
    // Enable
    bool enable_;
};


int main(int argv, char** argc)
{
    ros::init(argv, argc, "MR_Line_Follower");
    lineFollower cn;
    ros::Rate rate(cn.getFrequency());
    while(ros::ok())
        ros::spin();
}
