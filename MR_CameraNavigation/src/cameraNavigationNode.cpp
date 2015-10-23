
/**
 * Receive the processed information from the camera and 
 * use it to move the robot
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"

#include <iostream>
#include <string>


class cameraNavigation{
private:
	ros::Subscriber sub_qr_;
	ros::Subscriber sub_line_;
	ros::Subscriber sub_cross_;
	ros::Publisher pub_twist_;
	
public:
	/**
	 * Default constructor
	 */
	cameraNavigation(){
		ros::NodeHandle nh;
		
		// Publishers and Subscribers
		sub_qr_ = nh.subscribe<std_msgs::String>("/mr_camera_processing/qr", 1,
			&cameraNavigation::qrCallback, this);
		sub_line_ = nh.subscribe<geometry_msgs::Point>("/mr_camera_processing/line", 1,
			&cameraNavigation::lineCallback, this);
		sub_cross_ = nh.subscribe<std_msgs::Bool>("/mr_camera_processing/cross", 1, 
			&cameraNavigation::crossCallback, this);
			
		pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel", 1);
		
	}
	/**
	 * Default destructor
	 */
	~cameraNavigation(){
		// Nothing
	}
	/**
	 * Uses the received point into a PID to move the robot
	 * Point: negative->left, positive-> right
	 */
	void lineCallback(const geometry_msgs::Point detected_point){
		geometry_msgs::Point reference_point;
		reference_point.x = 640/2;
		reference_point.y = 480/2; // Image resolution
		
		ros::WallTime time_end = ros::WallTime::now();
		double pid_dt = 0.33; // 1/rate
		double pid_max = 640/2;
		double pid_min = -640/2;
		double Kp = 1.0;
		double Kd = 0.001;
		double Ki = 0.001;
		double pre_error;
		double integral;
		
		// Calculate error
		double pid_error = reference_point.x -detected_point.x;
		
		// Proportional term
		double Pout = Kp * pid_error;
		
		// Integral term
		integral += pid_error * pid_dt;
		double Iout = Ki * integral;
		
		// Derivative term
		double derivative = ( pid_error - pre_error ) / pid_dt;
		double Dout = Kd * derivative;
		
		// Calculate total output
		double pid_output = Pout + Iout + Dout;
		
		// Restrict to max/min
		if( pid_output > pid_max )
			pid_output = pid_max;
		else if( pid_output < pid_min )
			pid_output = pid_min;
		
		// Save error to previous error
		pre_error = pid_error;
		
		std::cout << std::endl;
		std::cout << "Error PID: " << pid_error << std::endl;
		std::cout << "Output PID: " << pid_output << std::endl;
		std::cout << "DTime PID: " << pid_dt << std::endl;
		std::cout << std::endl;
		
		/*
		 * Robot movement
		 */
		// Define the relation between the error and theta
		geometry_msgs::TwistStamped twistStamp_msg;
		double max_theta = 0.8;
		double theta = max_theta*pid_output/pid_max;
		std::cout << "Theta: " << theta << std::endl;
		// Define the speed
		double speed = 0.1;
		// Publish the message
		twistStamp_msg.header.stamp = ros::Time::now();
		twistStamp_msg.twist.linear.x = speed;
		twistStamp_msg.twist.angular.z = theta;
		
		pub_twist_.publish(twistStamp_msg);	
	}
	
	/**
	 * Process the data of the QR code
	 */
	void qrCallback(const std_msgs::String data){
		
	}
	
	/**
	 * Uses this Bool to detect a cross and take it into account
	 */
	void crossCallback(const std_msgs::Bool cross){
		
	}
	
};



int main(int argv, char** argc){
	ros::init(argv, argc, "mr_camera_navigation");
	cameraNavigation cn;
	ros::Rate rate(30);
	while (ros::ok()){
		ros::spin();
	}
}
