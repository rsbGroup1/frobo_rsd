// Includes
#include <ros/ros.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>


int main(int argc, char** argv)
{	
	// Init ROS Node
	ros::init(argc, argv, "mr_camera_false");
	ros::NodeHandle nh;
	// Create publisher topic
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("mr_camera_false/image", 1);
	
	//Load the image
	if (argc<2){
		std::cout << "Usage rosrun mr_camera_false mr_camera_false PATH_TO_IMAGE";
		return 0;
	}
	
			cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if (!image.data){
		std::cout << "Empty image!";
		return 0;
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	
	ros::Rate loopRate(30);
	
	std::cout << "Fuck yeah!" << std::endl;
	
	while (ros::ok()){
		pub.publish(msg);
		loopRate.sleep();
	}
		
	// Return
	return 0;
}
