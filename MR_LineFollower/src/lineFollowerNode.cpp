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

#include "mr_line_follower/followUntilQR.h"
#include "mr_line_follower/followUntilLidar.h"
#include "mr_camera_processing/enable.h"

#include <iostream>
#include <string>
#include <boost/thread.hpp>

// Lidar stuff
#include <sensor_msgs/LaserScan.h>

class lineFollower
{
public:
    /**
     * Default constructor
     */
    lineFollower()
    {
        // Get parameters
        ros::NodeHandle pNh_ ("~");
        pNh_.param<double> ("pid_p", pid_p_, 0.5);
        pNh_.param<double> ("pid_i", pid_i_, 0);
        pNh_.param<double> ("pid_d", pid_d_, 0);
        pNh_.param<double> ("pid_dt", pid_dt_, 0.33);   // 1/rate
        pNh_.param<double> ("pid_max", pid_max_, 320);
        pNh_.param<double> ("pid_min", pid_min_, -320);
        pNh_.param<int> ("reference_point_x", reference_point_x_, 320);
        pNh_.param<int> ("reference_point_y", reference_point_y_, 240);
        pNh_.param<double> ("robot_speed", robot_speed_, 0.1);
        pNh_.param<double> ("lidar_distance", lidar_distance_, 0.1);

        // Get topics name
        pNh_.param<std::string> ("sub_line", sub_line_name_, "/mrCameraProcessing/line");
        pNh_.param<std::string> ("sub_qr", sub_qr_name_, "/mrCameraProcessing/QR");
        pNh_.param<std::string> ("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        pNh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        pNh_.param<std::string> ("srv_lineQr", srv_lineUntilQR_name_, "/mrLineFollower/lineUntilQR");
        pNh_.param<std::string> ("srv_mr_camera_processing_enable_name", srv_mr_camera_processing_enable_name_, "/mrCameraProcessing/enable");
        pNh_.param<std::string> ("srv_lineLidar", srv_lineUntilLidar_name_, "/mrLineFollower/lineUntilLidar");

        srv_enable_ = nh_.advertiseService (
                          srv_lineUntilQR_name_, &lineFollower::lineUntilQRCallback, this);

        srv_mr_camera_processing_enable_ = nh_.serviceClient<mr_camera_processing::enable> (srv_mr_camera_processing_enable_name_);

        srv_lidar_enable_ = nh_.advertiseService (
                          srv_lineUntilLidar_name_, &lineFollower::lineUntilLidarCallback, this);
		
		pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
		pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);
		
		sub_line_ = nh_.subscribe<geometry_msgs::Point> (sub_line_name_, 1, &lineFollower::lineCallback, this);
		sub_qr_ = nh_.subscribe<std_msgs::String> (sub_qr_name_, 1, &lineFollower::qrCallback, this);
		
    }

    /**
     * Default destructor
     */
    ~lineFollower()
    {
        stopDeadman();
    }

    /**
     * Uses the received point into a PID to move the robot
     * Point: negative->left, positive-> right
     */
    void lineCallback (const geometry_msgs::Point detected_point)
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
        integral_ += pid_error * pid_dt_;
        double Iout = pid_i_ * integral_;

        // Derivative term
        double derivative = (pid_error - pre_error_) / pid_dt_;
        double Dout = pid_d_ * derivative;

        // Calculate total output
        double pid_output = Pout + Iout + Dout;

        // Restrict to max/min
        if (pid_output > pid_max_)
            pid_output = pid_max_;
        else if (pid_output < pid_min_)
            pid_output = pid_min_;

        // Save error to previous error
        pre_error_ = pid_error;

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
        pub_twist_.publish (twistStamp_msg);

        //std::cout << "Speed: " << robot_speed_ << " | Theta: " << theta << std::endl;
    }

    /**
     * Reads the detected qr and stores it in the object
     */
    void qrCallback (const std_msgs::String qr)
    {
        qr_detected_ = qr.data;
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

    /**
     * Kills the thread when an interruption_point is found
     */
    void stopDeadman()
    {
        deadmanThread_->interrupt();
    }

    /**
     * Enables or disables the line following
     */
    bool lineUntilQRCallback (mr_line_follower::followUntilQR::Request& req, mr_line_follower::followUntilQR::Response& res)
    {
        // Start the camera processing
        mr_camera_processing::enable enableCameraProcessing;
        enableCameraProcessing.request.enable = true;
        srv_mr_camera_processing_enable_.call (enableCameraProcessing);

        while (enableCameraProcessing.response.status != true)
        {
            ROS_INFO ("Not possible to start camera processing, trying againg");
            enableCameraProcessing.request.enable = true;
            srv_mr_camera_processing_enable_.call (enableCameraProcessing);
        }

        // Starts the deadman and publishers
        deadmanThread_ = new boost::thread (&lineFollower::enableDeadman, this);

        // Reset the PID and qr
        integral_ = 0;
        pre_error_ = 0;

        /*
         * Starts the line line follower
         */

        // Waits until it finds it or the time is more than the limit
        qr_detected_ = "";
        ros::Time time_start;
        time_start = ros::Time::now();

        while (req.qr != qr_detected_ &&
                (ros::Time::now().toSec() - time_start.toSec()) < req.time_limit)
        {
            ros::spinOnce();
        }

        if (req.qr == qr_detected_)
        {
            std::cout << "QR " << qr_detected_ << " detected!" << std::endl;
            res.success = true;
        }
        else
        {
            ROS_ERROR ("Time limit following the line reached");
            res.success = false;
        }

        // Disables the deadman
        stopDeadman();
		delete deadmanThread_;

        // Disables the camera processing
        enableCameraProcessing.request.enable = false;
        srv_mr_camera_processing_enable_.call (enableCameraProcessing);

        while (enableCameraProcessing.response.status != false)
        {
            ROS_INFO ("Not possible to stop camera processing, trying againg");
            enableCameraProcessing.request.enable = false;
            srv_mr_camera_processing_enable_.call (enableCameraProcessing);
        }

        // Ends!
        return true;
    }

    /**
     * Reads the detected qr and stores it in the object
     */
    void lidarCallback (const sensor_msgs::LaserScan lidar_in)
    {
		int start = (lidar_in.ranges.size()/2 - 10);
		int stop = (lidar_in.ranges.size()/2 + 10);
		double min = 99.0;
        for (int i= start; i<stop;i++){
			if (lidar_in.ranges[i] < min) min = lidar_in.ranges[i];
		}
		lidar_detected_ = min;
    }

    /**
     * Enables or disables the line following based on Lidar
     */
    bool lineUntilLidarCallback (mr_line_follower::followUntilLidar::Request& req, mr_line_follower::followUntilLidar::Response& res)
    {

        // Starts the deadman and publishers
        deadmanThread_ = new boost::thread (&lineFollower::enableDeadman, this);

        // Reset the PID and qr
        integral_ = 0;
        pre_error_ = 0;

        /*
         * Starts the line line follower
         */

        // Starts the subscribers
        sub_line_ = nh_.subscribe<geometry_msgs::Point> (sub_line_name_, 1, &lineFollower::lineCallback, this);
        sub_lidar_ = nh_.subscribe<sensor_msgs::LaserScan> (sub_lidar_name_, 1, &lineFollower::lidarCallback, this);

        // Waits until it finds it or the time is more than the limit
        lidar_detected_ = -1.0;
        ros::Time time_start;
        time_start = ros::Time::now();

        while (req.lidar_distance > lidar_detected_ &&
                (ros::Time::now().toSec() - time_start.toSec()) < req.time_limit)
        {
            ros::spinOnce();
        }

        if (req.lidar_distance <= lidar_detected_)
        {
            std::cout << "Distance to Lidar " << lidar_detected_ << " reached!" << std::endl;
            res.success = true;
        }
        else
        {
            ROS_ERROR ("Time limit following the line reached");
            res.success = false;
        }

        // Disables the deadman
        stopDeadman();
		delete deadmanThread_;

        // Ends!
        return true;

    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_, sub_qr_, sub_lidar_;
    ros::Publisher pub_twist_, pub_deadman_;
    ros::ServiceServer srv_enable_, srv_lidar_enable_;
    ros::ServiceClient srv_mr_camera_processing_enable_;
    // Threads
    boost::thread* deadmanThread_;
    // PID
    double pid_dt_;
    double pid_max_;
    double pid_min_;
    double pid_p_;
    double pid_d_;
    double pid_i_;
    double pre_error_;
    double integral_;
    double lidar_distance_;
    // Reference point
    int reference_point_x_;
    int reference_point_y_;
    // Robot
    double robot_speed_;
    // Topics name
    std::string sub_line_name_, sub_qr_name_,sub_lidar_name_;
    std::string pub_deadman_name_, pub_twist_name_;
    std::string srv_lineUntilQR_name_, srv_mr_camera_processing_enable_name_, srv_lineUntilLidar_name_;
    // QR
    std::string qr_desired_;
    std::string qr_detected_;
	// Lidar
	double lidar_detected_;
	double lidar_desired_;
};

/**
 * Main
 */
int main (int argv, char** argc)
{
    ros::init (argv, argc, "MR_Line_Follower");
    lineFollower cn;
    ros::Rate rate (30);
    ros::AsyncSpinner spinner (3);

    while (!ros::isShuttingDown())
    {
        spinner.start();
        rate.sleep();
    }
}
