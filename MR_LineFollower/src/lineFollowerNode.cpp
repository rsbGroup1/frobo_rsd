/**
 * Subscribes to the camera's images, analize it to find the line
 * and publishes information for the robot to follow it
 */

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Constants
static const std::string OPENCV_WINDOW = "Image filtered";

const int threshold_slider_max = 254;
const int minLength_slider_max = 254;
const int maxLineGap_slider_max = 254;
int threshold_slider=37;
int minLength_slider=66;
int maxLineGap_slider=95;

class ImageConverter
{
    ros::NodeHandle nh_, _pNh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_;

public:
    ImageConverter() : it_(nh_), _pNh(ros::this_node::getName() + "/")
    {
        // Subscribe to input video feed and publish output video feed
        std::string imageSub, imagePub;
        _pNh.param<std::string>("imageInput_sub", imageSub, "/mr_camera_false/image" /*"/mrCamera/image"*/);
        _pNh.param<std::string>("imageFiltered_pub", imagePub, "/mrLineFollower/image_filtered");

        sub_image_ = it_.subscribe(imageSub, 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
        pub_image_ = it_.advertise(imagePub, 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Transform the message to an OpenCV image
        cv_bridge::CvImagePtr image_ptr;

        try
        {
            // Load the image in MONO8
            image_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch( cv_bridge::Exception& e )
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


		// Bilateral Filter
		cv::Mat image_filtered; //Necessary for the bilateral filter
		cv::bilateralFilter(image_ptr->image, image_filtered, 15, 300, 300);
		
		// Color threshold
		cv::inRange(image_filtered, cv::Scalar(0, 0, 0), cv::Scalar(30, 30, 30), image_filtered);
		
        // Canny edge
        unsigned char cannyMinThreshold = 50;
        unsigned char cannyMaxThreshold = 100;
		cv::Canny(image_filtered, image_filtered, cannyMinThreshold, cannyMaxThreshold);
		
		// Probabilistic Hough Lines
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(image_filtered, lines, 1, CV_PI/180, threshold_slider+1, minLength_slider+1, maxLineGap_slider+1 );
		// Drawing
		for( size_t i = 0; i < lines.size(); i++ ) {
			cv::Vec4i l = lines[i];
			line(image_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
		}
		
		// Fusion lines and find the center
		for( size_t i = 0; i < lines.size(); i++ ) {
			
		}
        // Update GUI Window
        cv::createTrackbar( "threshold", OPENCV_WINDOW, &threshold_slider, threshold_slider_max);
		cv::createTrackbar( "minLength", OPENCV_WINDOW, &minLength_slider, minLength_slider_max);
		cv::createTrackbar( "maxLineGap", OPENCV_WINDOW, &maxLineGap_slider, maxLineGap_slider_max);
        cv::imshow(OPENCV_WINDOW, image_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        pub_image_.publish(image_ptr->toImageMsg());
    }
};

int main ( int argc, char** argv )
{
    ros::init(argc, argv, "mr_line_follower");
    ImageConverter ic;
    ros::spin();
    return 0;
}
