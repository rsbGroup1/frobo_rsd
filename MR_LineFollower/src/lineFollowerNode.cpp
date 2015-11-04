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
#include "mr_camera_processing/enable.h"

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
        nh_.param<std::string> ("sub_line", sub_line_name_, "/mrCameraProcessing/line");
		nh_.param<std::string> ("sub_qr", sub_qr_name_, "/mrCameraProcessing/qr");
        nh_.param<std::string> ("pub_twist", pub_twist_name_, "/fmCommand/cmd_vel");
        nh_.param<std::string> ("pub_deadman", pub_deadman_name_, "/fmSafe/deadman");
        nh_.param<std::string> ("srv_lineQr", srv_lineUntilQR_name_, "/mrLineFollower/lineUntilQR");
		nh_.param<std::string> (
			"srv_mr_camera_processing_enable_name", 
			srv_mr_camera_processing_enable_name_, 
			"/mrCameraProcessing/enable");

        // Publishers, subscribers, services
        sub_line_ = nh_.subscribe<geometry_msgs::Point> (
			sub_line_name_, 1, &lineFollower::lineCallback, this);
		sub_qr_ = nh_.subscribe<std_msgs::String> (
			sub_qr_name_, 1, &lineFollower::qrCallback, this);
		
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped> (pub_twist_name_, 1);
        pub_deadman_ = nh_.advertise<msgs::BoolStamped> (pub_deadman_name_, 1);
        
		srv_enable_ = nh_.advertiseService(
			                       srv_lineUntilQR_name_, &lineFollower::lineUntilQRCallback, this);
		
		srv_mr_camera_processing_enable_ = nh_.serviceClient<mr_camera_processing::enable>(srv_mr_camera_processing_enable_name_);

        // Threads
        deadmanThread_ = new boost::thread(&lineFollower::enableDeadman, this);
		stopDeadman();
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
    void lineCallback(const geometry_msgs::Point detected_point)
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
		if(pid_output > pid_max_)
			pid_output = pid_max_;
		else
			if(pid_output < pid_min_)
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
		pub_twist_.publish(twistStamp_msg);

		std::cout << "Speed: " << robot_speed_ << " | Theta: " << theta << std::endl;
	}
	
	/**
	 * Reads the detected qr and stores it in the object
	 */
	void qrCallback(const std_msgs::String qr){
		qr_detected_ = qr.data;
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
            catch(const boost::thread_interrupted&)
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
     * Returns the frecuency of the Node
     */
    double getFrequency()
    {
        return 1 / pid_dt_;
    }

    /**
     * Enables or disables the line following
     */
	bool lineUntilQRCallback(mr_line_follower::followUntilQR::Request& req, mr_line_follower::followUntilQR::Response& res)
    {
		// Start the camera processing
		mr_camera_processing::enable enableCameraProcessing;
		enableCameraProcessing.request.enable = true;
		srv_mr_camera_processing_enable_.call(enableCameraProcessing);
		while (enableCameraProcessing.response.status != true) {
			ROS_INFO("Not possible to start camera processing, trying againg");
			enableCameraProcessing.request.enable = true;
			srv_mr_camera_processing_enable_.call(enableCameraProcessing);
		}
		// Reset the PID and qr
		integral_ = 0;
		pre_error_ = 0;
		// Enables deadman
		deadmanThread_ = new boost::thread(&lineFollower::enableDeadman, this);
		/*
		 * Starts the line line follower
		 */
		// Starts the subscribers
		sub_line_ = nh_.subscribe<geometry_msgs::Point> (
			sub_line_name_, 1, &lineFollower::lineCallback, this);
		sub_qr_ = nh_.subscribe<std_msgs::String> (
			sub_qr_name_, 1, &lineFollower::qrCallback, this);
		
		// Waits until it finds it or the time is more than the limit
		ros::Time time_start;
		time_start = ros::Time::now();
		while (req.qr != qr_detected_ 
			&& (ros::Time::now().toSec() - time_start
			.toSec() > req.time_limit))
		{
			// Nothing
		}
		
		if (req.qr != qr_detected_) 
			res.success = true;
		else 
			res.success = false;

		// Stops the subscribers
		sub_line_.shutdown();
		sub_qr_.shutdown();
		// Disables the deadman
		stopDeadman();
		// Disables the camera processing
		enableCameraProcessing.request.enable = false;
		srv_mr_camera_processing_enable_.call(enableCameraProcessing);
		while(enableCameraProcessing.response.status != false){
			ROS_INFO("Not possible to stop camera processing, trying againg");
			enableCameraProcessing.request.enable = false;
			srv_mr_camera_processing_enable_.call(enableCameraProcessing);
		}
		
		// Ends!
        return true;
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_line_, sub_qr_;
    ros::Publisher pub_twist_, pub_deadman_;
    ros::ServiceServer srv_enable_;
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
    // Reference point
    int reference_point_x_;
    int reference_point_y_;
    // Robot
    double robot_speed_;
    // Topics name
    std::string sub_line_name_, sub_qr_name_;
    std::string pub_deadman_name_, pub_twist_name_;
    std::string srv_lineUntilQR_name_, srv_mr_camera_processing_enable_name_;
	// QR
	std::string qr_desired_;
	std::string qr_detected_;
};

/**
 * Main 
 */
int main(int argv, char** argc)
{
    ros::init(argv, argc, "MR_Line_Follower");
    lineFollower cn;
    ros::Rate rate(cn.getFrequency());
	ros::AsyncSpinner spinner(0);
    while(!ros::isShuttingDown())
		spinner.start();
}
