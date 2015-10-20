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

		// Fusion lines and find the center
		std::vector<cv::Vec4i> lines_fusioned;
		double fusionThreshold = 30; //pixels
		
		lines_fusioned.clear();
		for(unsigned char i=0; i<lines.size(); i++) {
			cv::Vec4i lineToAdd = lines[i];
			std::cout << "Line added" << lineToAdd << std::endl;
			lines_fusioned.push_back(lineToAdd);
			for(unsigned char j=0; j<lines_fusioned.size(); j++) {
				cv::Vec4i lineToCompare = lines_fusioned[j];
				std::cout << "Line compared" << lineToCompare << std::endl;
				//Check if they are similar. If so, remove it.
				if( ( lineToAdd[0] - lineToCompare[0])<fusionThreshold && 
					( lineToAdd[1] - lineToCompare[1])<fusionThreshold && 
					( lineToAdd[2] - lineToCompare[2])<fusionThreshold && 
					( lineToAdd[3] - lineToCompare[3])<fusionThreshold &&
					( lineToAdd[0] - lineToCompare[0]) != 0 && 
					( lineToAdd[1] - lineToCompare[1]) != 0 && 
					( lineToAdd[2] - lineToCompare[2]) != 0 && 
					( lineToAdd[3] - lineToCompare[3]) != 0 )
				{
						lines_fusioned.pop_back();
						std::cout << "Same line" << std::endl;
				}
			}
		}
		
		std::cout << "\n" << std::endl;		
		
		// Drawing
		for(unsigned char i = 0; i < lines_fusioned.size(); i++) {
			cv::Vec4i l = lines_fusioned[i];
			line(image_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
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
