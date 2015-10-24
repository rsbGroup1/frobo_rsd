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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <zbar.h>

class ImageConverter
{
private:
	ros::NodeHandle nh_, _pNh;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_image_;
	image_transport::Publisher pub_image_;
	ros::Publisher pub_line_;
	ros::Publisher pub_qr_;
	ros::Publisher pub_cross_;
	
public:
	ImageConverter() : it_(nh_), _pNh(ros::this_node::getName() + "/")
	{
		sub_image_ = it_.subscribe("/mr_camera/image",1, &ImageConverter::imageCb,
							 this, image_transport::TransportHints("compressed"));
		pub_line_ = nh_.advertise<geometry_msgs::Point>("mr_camera_processing/line", 1);
		pub_qr_ = nh_.advertise<std_msgs::String>("mr_camera_processing/qr", 1);
		pub_cross_ = nh_.advertise<std_msgs::Bool>("mr_camera_processing/cross", 1);
		pub_image_ = it_.advertise("/outputImage", 1);
	}
	
	~ImageConverter()
	{
		//cv::destroyWindow(OPENCV_WINDOW);
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{;
		
		// Transform the message to an OpenCV image
		cv_bridge::CvImagePtr image_ptr;
		image_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::TYPE_8UC3);
	
		if (image_ptr->image.empty()){
			ROS_INFO("No image being received");
		} else { 
	
		// Flip
		cv::flip(image_ptr->image, image_ptr->image, -1);
		// Detection	
		lineDetector(image_ptr->image);
		//qrDetector(image_ptr->image);
		//crossDetector(image_ptr->image);
		}
	}
	
	void lineDetector(cv::Mat image_original){
		/*
		 * Image processing
		 */
		// Bilateral Filter
		cv::Mat image_filtered; //Necessary for the bilateral filter
		cv::bilateralFilter(image_original, image_filtered, 15, 300, 300);
		
		// Color threshold
		cv::inRange(image_filtered, cv::Scalar(0, 0, 0), cv::Scalar(30, 30, 30), image_filtered);
		
		// Canny edge
		unsigned char cannyMinThreshold = 50;
		unsigned char cannyMaxThreshold = cannyMinThreshold * 2;
		cv::Canny(image_filtered, image_filtered, cannyMinThreshold, cannyMaxThreshold, 3);
		
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
			cv::circle(image_original, detected_point, 1, cv::Scalar(255, 0, 255), 2);
		} else {
			ROS_INFO("Not point found");
		}	
		
		/*
		 * Publish
		 */
		//Image
		sensor_msgs::ImagePtr image_msg;
		image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_original).toImageMsg();
		pub_image_.publish(image_msg);
		
		//Point
		geometry_msgs::Point point_msg;
		point_msg.x = detected_point.x;
		point_msg.y = detected_point.y;
		pub_line_.publish(point_msg);
	}
	
	void qrDetector(cv::Mat image_original){
		// Set up the scanner 
		cv::Mat gray;
		zbar::ImageScanner scanner;
		scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
		
		// Convert image to grayscale
		cv::cvtColor(image_original, gray, CV_RGB2GRAY); // image should always be grayscale!!!
		
		// Prepare the image for reading
		uchar *raw = (uchar*)gray.data;
		zbar::Image image(image_original.cols, image_original.rows, "Y800",
						  raw , image_original.cols * image_original.rows);
		
		// Scan the image to find QR code
		int n = scanner.scan(image);
		
		// String used for telling what the QR or barcode says.
		std::string data_type, data;
		
		// Read the image
		for( zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
		{
			std::vector<cv::Point> vp;
			
			// Write out the symbols and data
			// type name equals type of code QR or Barcode...
			// data equals the data that can be found in the QR or Barcode 
			// cout << "decode" << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' << " " << endl;
			
			//data_type = symbol->get_type_name();
			data = symbol->get_data();
			
			// Get the point for the QR code to show where they are. 
			for(int i = 0; i < n; i++)
				vp.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
			
			cv::RotatedRect r = cv::minAreaRect(vp);
			cv::Point2f pts[4];
			r.points(pts); 
			
			// draw the lines around the QR code. not working fully yet.
			for(int i = 0; i < 4; i++)
				cv::line(image_original, pts[i], pts[i+1], cv::Scalar(255,0,0), 3);
		}
		// Show the image with the QR code + lines around. 
		//cv::namedWindow("decoded image", cv::WINDOW_AUTOSIZE);
		//cv::imshow("decoded image", image_original);
		
		std_msgs::String data_msg;
		data_msg.data = data;
		pub_qr_.publish(data_msg);
	}
	
	void crossDetector(cv::Mat image_original){
		
	}
};

int main ( int argc, char** argv )
{
	ros::init(argc, argv, "mr_camera_processing");
	ImageConverter ic;
	ros::spin();
	return 0;
}
