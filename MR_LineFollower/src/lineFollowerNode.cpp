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
cv::RNG rng(12345);

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
		
		/*
		 * Image processing
		 */
		
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
        unsigned char cannyMaxThreshold = cannyMinThreshold * 2;
		cv::Canny(image_filtered, image_filtered, cannyMinThreshold, cannyMaxThreshold, 3);
		
		
		/* 
		 * MOMENTS
		 * 
		// Find contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		
		cv::findContours( image_filtered, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		
		// Calculate moments
		std::vector<cv::Moments> mu(contours.size() );
		for( size_t i = 0; i < contours.size(); i++ ){ 
			mu[i] = cv::moments( contours[i], false ); 
		}
		cv::vector<cv::Point2f> mc( contours.size() );
		for( size_t i = 0; i < contours.size(); i++ ){ 
			mc[i] = cv::Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) ); 
			
		}
		cv::Mat drawing = cv::Mat::zeros( image_filtered.size(), CV_8UC3 );
		for( size_t i = 0; i< contours.size(); i++ )
		{
			cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
			//cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
		}
		*/
		
		/*
		 * HOUGH
		 * 
		// Probabilistic Hough Lines
		std::vector<cv::Vec4i> lines;
		//cv::HoughLinesP(image_filtered, lines, 1, CV_PI/180, threshold_slider+1, minLength_slider+1, maxLineGap_slider+1 );

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
		
		// Drawing
		for(unsigned char i = 0; i < lines_fusioned.size(); i++) {
			cv::Vec4i l = lines_fusioned[i];
			line(image_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
			line(image_filtered, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
		}
		
		*/
		
		
		
		/*
		 * Find reference point
		 */
		cv::Point reference_point;
		int y_reference = image_filtered.rows/2;
		int x_reference;
		std::vector<int> points_detected;
		
		//In the row of the y_reference search for the detected points
		points_detected.clear();
		for (int i=0; i<image_filtered.cols; i++){
			if (image_filtered.at<uchar>(y_reference, i) == 255){ // Is white
				points_detected.push_back(i);
				//cv::circle(image_filtered, cv::Point(i, y_reference), 1, cv::Scalar(255),1);
			}
		}
		
		// Select the two first ones and find the middle point
		// and update the point
		if ( points_detected.size() >= 2) {
			//Average
			int aux;
			for (auto i : points_detected)
				aux+=i;
			x_reference = aux/points_detected.size();
			aux = 0;
			// Update the point
			reference_point.x = x_reference;
			reference_point.y = y_reference;
			cv::circle(image_filtered, reference_point, 1, cv::Scalar(255),2);
			cv::circle(image_ptr->image, reference_point, 1, cv::Scalar(255),2);
			//std::cout << reference_point << std::endl;
		} else {
			ROS_INFO("Not point found");
		}		
		

		/*
		 * PID
		 */
		
		
        // Update GUI Window
        //cv::createTrackbar( "threshold", OPENCV_WINDOW, &threshold_slider, threshold_slider_max);
		//cv::createTrackbar( "minLength", OPENCV_WINDOW, &minLength_slider, minLength_slider_max);
		//cv::createTrackbar( "maxLineGap", OPENCV_WINDOW, &maxLineGap_slider, maxLineGap_slider_max);
        cv::imshow(OPENCV_WINDOW, image_filtered);
		//cv::imshow("Final", image_ptr->image);
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
