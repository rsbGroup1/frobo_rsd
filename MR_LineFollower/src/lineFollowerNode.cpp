/**
 * Subscribes to the camera's images, analize it to find the line
 * and publishes information for the robot to follow it
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image filtered";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_;

public:
    ImageConverter() : it_ ( nh_ ) {
        // Subscribe to input video feed and publish output video feed
        sub_image_ = it_.subscribe ( "/camera/image", 1, &ImageConverter::imageCb, this, 
									 image_transport::TransportHints("compressed"));
        pub_image_ = it_.advertise ( "/mr_line_follower/image_filtered", 1 );

        cv::namedWindow ( OPENCV_WINDOW );
    }

    ~ImageConverter() {
        cv::destroyWindow ( OPENCV_WINDOW );
    }

    void imageCb ( const sensor_msgs::ImageConstPtr& msg ) {
        //Transform the message to an OpenCV image
		cv_bridge::CvImagePtr image_ptr;
        try {
			//Load the image in MONO8
            image_ptr = cv_bridge::toCvCopy ( msg, sensor_msgs::image_encodings::MONO8 );
        } catch ( cv_bridge::Exception& e ) {
            ROS_ERROR ( "cv_bridge exception: %s", e.what() );
            return;
        }

       /*
		* Canny Edge detector
		*/
		//To binary
		cv::threshold(image_ptr->image, image_ptr->image, 160, 255, CV_THRESH_BINARY);
		//Blur
		cv::blur(image_ptr->image, image_ptr->image, cv::Size(3,3));
		//Canny edge
		double cannyMinThreshold = 200;
		double cannyMaxThreshold = 255;
		cv::Canny(image_ptr->image, image_ptr->image, cannyMinThreshold, cannyMaxThreshold);
	   
        // Update GUI Window
        cv::imshow ( OPENCV_WINDOW, image_ptr->image );
        cv::waitKey ( 3 );

        // Output modified video stream
        pub_image_.publish ( image_ptr->toImageMsg() );
    }
};

int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "mr_line_follower" );
    ImageConverter ic;
    ros::spin();
    return 0;
}
