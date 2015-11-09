/**
 * Subscribes to the camera's images, analize it to find the line
 * and publishes information for the robot to follow it
 */

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "mr_camera_processing/enable.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <zbar.h>

#include <boost/thread.hpp>

class ImageConverter {
public:
	ImageConverter() : it_(nh_) 
	{
		ros::NodeHandle pNh_("~");
		pNh_.param<std::string>("sub_image", sub_image_name_, "/mrCamera/image");
       	pNh_.param<std::string>("pub_image_line", pub_image_line_name_, "/mrCameraProcessing/line_image");
       	pNh_.param<std::string>("pub_image_qr", pub_image_qr_name_, "/mrCameraProcessing/qr_image");
        pNh_.param<std::string>("pub_qr", pub_qr_name_, "/mrCameraProcessing/qr");
       	pNh_.param<std::string>("pub_line", pub_line_name_, "/mrCameraProcessing/line");
		pNh_.param<std::string>("srv_enable", srv_enable_name_, "/mrCameraProcessing/enable");
		
		pub_line_ = nh_.advertise<geometry_msgs::Point>(pub_line_name_, 1);
		pub_qr_ = nh_.advertise<std_msgs::String>(pub_qr_name_, 1);
		pub_image_line_ = it_.advertise(pub_image_line_name_, 1);
		pub_image_qr_ = it_.advertise(pub_image_qr_name_, 1);
		
		srv_enable_ = nh_.advertiseService(srv_enable_name_, &ImageConverter::enableCallback, this);
	}
	
	~ImageConverter() {
	}
	
	/**
	 * Enables or disables the image processing by subscribing or 
	 * shutingdown the image subscriber
	 */
	bool enableCallback(mr_camera_processing::enable::Request& req, 
						mr_camera_processing::enable::Response& res)
	{
		if (req.enable == true)
			sub_image_ = it_.subscribe(sub_image_name_,1, &ImageConverter::imageCb, this,
									   image_transport::TransportHints("compressed"));
		else
			sub_image_.shutdown();
		
		res.status = req.enable;
		return true;
	}
	
	/**
	 * Receives the image and process it
	 */
	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		// Transform the message to an OpenCV image
		cv_bridge::CvImagePtr image_ptr;
		image_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::TYPE_8UC3);
	
		if (image_ptr->image.empty()){
			ROS_INFO("No image being received");
		} else {
			// Flip
			cv::flip(image_ptr->image, image_ptr->image, -1);
			
			// Detection
			lineThread_ = new boost::thread(&ImageConverter::lineDetector, this, image_ptr->image);
			qrThread_ = new boost::thread(&ImageConverter::qrDetector, this, image_ptr->image);
		}
	}
	
	/**
	 * Search for the lines
	 */
	void lineDetector(cv::Mat image_original){
		/*
		 * Image processing
		 */
		// Croping
		int offset = 20;
		cv::Rect croppedArea(0, image_original.rows/2-offset, image_original.cols, offset*2);
		cv::Mat image_cropped = image_original(croppedArea);

		// Bilateral Filter
		cv::Mat image_filtered; //Necessary for the bilateral filter
		cv::bilateralFilter(image_cropped, image_filtered, 15, 300, 300);
		
		// Color threshold
		cv::inRange(image_filtered, cv::Scalar(0, 0, 0), cv::Scalar(30, 30, 30), image_filtered);
		
		// Canny edge
		unsigned char cannyMinThreshold = 50;
		unsigned char cannyMaxThreshold = cannyMinThreshold * 2;
		cv::Canny(image_filtered, image_filtered, 
				  cannyMinThreshold, cannyMaxThreshold, 3);
		
		/*
		 * Detect point
		 */
		cv::Point detected_point;
		int y_detected = image_filtered.rows/2;
		int x_detected;
		std::vector<int> points_detected;
		
		//In the row of the y_reference search for the detected points
		points_detected.clear();
		for (int i=0; i<image_filtered.cols; i++){
			if (image_filtered.at<uchar>(y_detected, i) == 255) // Is white
				points_detected.push_back(i);
		}
		
		// Do the average of all the points detected to find the center of the line
		if (points_detected.size() >= 2){
			// Average
			int aux = 0;
			for (auto i : points_detected)
				aux+=i;
			x_detected = aux/points_detected.size();
			// Update the point
			detected_point.x = x_detected;
			detected_point.y = y_detected;
			cv::circle(image_filtered, detected_point, 1, cv::Scalar(255), 2);
			cv::circle(image_cropped, detected_point, 1, cv::Scalar(255, 0, 255), 2);
		} else {
			ROS_INFO("Not point found");
		}	
		
		/*
		 * Publish
		 */
		// Image
		sensor_msgs::ImagePtr image_msg;
		image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_cropped).toImageMsg();
		pub_image_line_.publish(image_msg);
		
		// Point
		geometry_msgs::Point point_msg;
		point_msg.x = detected_point.x;
		point_msg.y = detected_point.y;
		pub_line_.publish(point_msg);
		
		// Signal interrupt point
		boost::this_thread::interruption_point();
	}
	
	/**
	 * Search for the QR code and publish its content
	 */
	void qrDetector(cv::Mat image_original){
		// Set up the scanner 
		zbar::ImageScanner scanner;
		scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
		
		// Filter
		cv::Mat image_filtered;
		//cv::bilateralFilter(image_original, image_filtered, 15, 300, 300);

		// Convert image to grayscale
		cv::cvtColor(image_original, image_filtered, CV_RGB2GRAY);
		//cv::inRange(image_original, cv::Scalar(0, 0, 0), cv::Scalar(40, 40, 40), image_filtered);
		
		
		// Prepare the image for reading
		zbar::Image zbar_image(image_filtered.cols, image_filtered.rows, "Y800",
								(uchar*)image_filtered.data , image_filtered.cols * image_filtered.rows);
		
		// Scan the image to find QR code
		scanner.scan(zbar_image);
		
		// String used for telling what the QR or barcode says.
		std::string data;
		
		// Read the image
		for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
			symbol != zbar_image.symbol_end(); ++symbol)
			data = symbol->get_data();

		
		/*
		 * Publish
		 */
		// Image
		sensor_msgs::ImagePtr image_msg;
		image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_filtered).toImageMsg();
		pub_image_qr_.publish(image_msg);
		
		// Data
		std_msgs::String msg;
		msg.data = data;
		pub_qr_.publish(msg);
		
		// Signal interrupt point
		boost::this_thread::interruption_point();
	}
	
private:
	// ROS
	ros::NodeHandle nh_, _pNh;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_image_;
	image_transport::Publisher pub_image_line_;
	image_transport::Publisher pub_image_qr_;
	ros::Publisher pub_line_, pub_qr_;
	ros::ServiceServer srv_enable_;
	std::string sub_image_name_;
	std::string pub_image_qr_name_, pub_image_line_name_, pub_line_name_, pub_qr_name_;
	std::string srv_enable_name_;
	
	// Threads
	boost::thread* qrThread_;
	boost::thread* lineThread_;
};

/**
 * Main
 */
int main ( int argc, char** argv )
{
    ros::init(argc, argv, "MR_Camera_Processing");
	ImageConverter ic;
	while(ros::ok())
		ros::spin();
	return 0;
}
