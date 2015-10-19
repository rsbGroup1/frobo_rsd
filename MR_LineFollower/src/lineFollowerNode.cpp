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
        _pNh.param<std::string>("imageInput_sub", imageSub, "/mrCamera/image");
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
            image_ptr = cv_bridge::toCvCopy ( msg, sensor_msgs::image_encodings::MONO8 );
        }
        catch( cv_bridge::Exception& e )
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Canny Edge detector
        unsigned char binaryThreshold = 30;
        unsigned char erodeDilateSize = 15;
        unsigned char blurSize = 3;
        unsigned char cannyMinThreshold = 50;
        unsigned char cannyMaxThreshold = 100;

        // Flip
        //cv::flip(image_ptr->image, image_ptr->image, 1);

        // To binary
        cv::threshold(image_ptr->image, image_ptr->image, binaryThreshold, 255, CV_THRESH_BINARY_INV);

        // Erode and Dilate
        cv::erode(image_ptr->image, image_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeDilateSize,erodeDilateSize)));
        cv::dilate(image_ptr->image, image_ptr->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeDilateSize,erodeDilateSize)));

        // Blur
        cv::blur(image_ptr->image, image_ptr->image, cv::Size (blurSize,blurSize));

        // Canny edge
        cv::Canny(image_ptr->image, image_ptr->image, cannyMinThreshold, cannyMaxThreshold);

        // Hough Lines
        std::vector<cv::Vec4i> lines_filtered, lines_hough;
		float r, t, lactual, ldiff;
		std::vector<int> lines_ignored;
        cv::HoughLinesP(image_ptr->image, lines_hough, 1, CV_PI/180, 100, 0, 0);
		cv::cvtColor(image_ptr->image, image_ptr->image, cv::COLOR_GRAY2BGR);
		
		// Filter ortogonal lines
		for (size_t i = 0; i < lines_hough.size(); i++) {
			lactual = lines_hough[i][1];
			
			for (size_t j = 0; j < lines_hough.size(); j++) {
				ldiff = fabs(( lines_hough[j][1]) - lactual);
				if (((ldiff > 1.565) && (ldiff < 1.575)) || (ldiff < 0.0005)) {
					std::cout << ldiff << std::endl;
					lines_ignored.push_back(j);
					lines_filtered.push_back(lines_hough[j]);
					lines_hough.erase(lines_hough.begin() + j);
				}
			}
		}
		
		// Print
		for(size_t i = 0; i < lines_filtered.size(); i++)
        {
			float rho = lines_filtered[i][0], theta = lines_filtered[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cv::line(image_ptr->image, pt1, pt2, cv::Scalar(0,255,0), 3, CV_AA);
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, image_ptr->image);
        cv::waitKey(1000);

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
