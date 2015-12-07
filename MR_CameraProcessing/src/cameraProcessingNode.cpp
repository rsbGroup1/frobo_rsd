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
#include "mr_camera/enable.h"

class ImageConverter
{
public:
    ImageConverter() : it_(nh_), enabled_(false)
    {
        ros::NodeHandle pNh_("~");
        pNh_.param<std::string>("sub_image", sub_image_name_, "/mrCamera/image");
        pNh_.param<std::string>("pub_image_line", pub_image_line_name_, "/mrCameraProcessing/line_image");
        pNh_.param<std::string>("pub_image_qr", pub_image_qr_name_, "/mrCameraProcessing/qr_image");
        pNh_.param<std::string>("pub_qr", pub_qr_name_, "/mrCameraProcessing/qr");
        pNh_.param<std::string>("pub_line", pub_line_name_, "/mrCameraProcessing/line");
        pNh_.param<std::string>("srv_enable", srv_enable_name_, "/mrCameraProcessing/enable");
        pNh_.param<std::string>("mr_camera_srv_enable", srv_mr_camera_enable_name_, "/mrCamera/enable");
        pNh_.param<double>("QR_min_white_area", minQRConArea_, 200.0);
        pNh_.param<int>("QR_min_black_area", minQRBlackArea_, 1);
        pNh_.param<int>("QR_grayscale_threshold", qrSquareThresh_, 175);

        pub_line_ = nh_.advertise<geometry_msgs::Point>(pub_line_name_, 1);
        pub_qr_ = nh_.advertise<std_msgs::String>(pub_qr_name_, 1);
        pub_image_line_ = it_.advertise(pub_image_line_name_, 1);
        pub_image_qr_ = it_.advertise(pub_image_qr_name_, 1);

        srv_enable_ = nh_.advertiseService(srv_enable_name_, &ImageConverter::enableCallback, this);
        sub_image_ = it_.subscribe(sub_image_name_, 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
        srv_mr_camera_enable = nh_.serviceClient<mr_camera::enable>(srv_mr_camera_enable_name_);
    }

    ~ImageConverter()
    {
    }

    /**
     * Enables or disables the image processing by subscribing or
     * shutingdown the image subscriber
     */
    bool enableCallback(mr_camera_processing::enable::Request& req, mr_camera_processing::enable::Response& res)
    {
        mr_camera::enable mr_camera_call;
        mr_camera_call.request.enable = req.enable;
        srv_mr_camera_enable.call(mr_camera_call);
	
        enabled_ = req.enable;
        res.status = enabled_;
	    
        return true;
    }

    /**
     * Receives the image and process it
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        if (enabled_)
        {
            // Transform the message to an OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3)->image;

            if(image.empty())
            {
                ROS_INFO ("No image being received");
            }
            else
            {
                // Flip
                cv::flip(image, image, -1);

                // Detection
                lineThread_ = new boost::thread(&ImageConverter::lineDetector, this, image);
                qrThread_ = new boost::thread(&ImageConverter::qrDetector, this, image);
            }
        }
    }

    /**
     * Search for the lines
     */
    void lineDetector(cv::Mat image_original)
    {
        /*
         * Image processing
         */
        // Croping
        int croppingHeight = 60;
        int yOffset = 0;
        cv::Mat image_cropped = image_original(cv::Rect(0, image_original.rows / 2 - croppingHeight + yOffset, image_original.cols, croppingHeight * 2));

        // Color threshold
        cv::inRange(image_cropped, cv::Scalar (0, 0, 0), cv::Scalar (50, 50, 50), image_cropped);

        // DEBUG
        //cv::imshow("Color", image_cropped);

        /*
         * Detect point
         */
        int numberOfRows = 10; // image_filtered.rows;
        int rowSpacing = image_cropped.rows/numberOfRows;
        int xLeftWhitePixel = 0;
        int xRightWhitePixel = image_cropped.cols;

        for(int i=0; i<numberOfRows; i++)
        {
            // Get line
            int y = rowSpacing*i;

            // Find min white x
            for(int x=0; x<image_cropped.cols; x++)
            {
                // Check if white
                if(image_cropped.at<uchar>(y,x) == 255)
                {
                    if(x > xLeftWhitePixel)
                    {
                        xLeftWhitePixel = x;
                        break;
                    }
                }
            }

            // Find max white x
            for(int x=image_cropped.cols-1; x>=0; x--)
            {
                // Check if white
                if(image_cropped.at<uchar>(y,x) == 255)
                {
                    if(x < xRightWhitePixel)
                    {
                        xRightWhitePixel = x;
                        break;
                    }
                }
            }
        }

        cv::Mat image_to_show;
		cvtColor(image_cropped, image_to_show, CV_GRAY2RGB);
        cv::Point detected_point(xLeftWhitePixel + (xRightWhitePixel-xLeftWhitePixel)/2, image_cropped.rows/2 + yOffset);
        cv::circle(image_to_show, detected_point, 1, cv::Scalar (0, 255, 255), 2);

        // Debug
		//cv::imshow("Output", image_to_show);

        // DEBUG
        //cv::waitKey(1);

        /*
         * Publish
         */
        // Image
        sensor_msgs::ImagePtr image_msg;
		image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_to_show).toImageMsg();
        pub_image_line_.publish(image_msg);

        // Point
        geometry_msgs::Point point_msg;
        point_msg.x = detected_point.x;
        point_msg.y = detected_point.y;
        pub_line_.publish (point_msg);

        // Signal interrupt point
        boost::this_thread::interruption_point();
    }

    /**
     * Search for the QR code and publish its content
     */
    void qrDetector(cv::Mat image_original)
    {
        // Set up the scanner
        zbar::ImageScanner scanner;

        // Disable all symbologies
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);

        // Enable symbologie for QR
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);;

        // Filter
        cv::Mat image_filtered;

        // Convert image to grayscale
        cv::cvtColor(image_original, image_filtered, CV_RGB2GRAY);

        // String used for telling what the QR or barcode says.
        std::string data;


        // Quick search image if there is something which resembles a QR code
        if(qrContourSearch(image_filtered))
        {
            //ROS_INFO("QR FOUND!");
            // Prepare the image for reading
            zbar::Image zbar_image(image_filtered.cols, image_filtered.rows, "Y800",
                                    (uchar*) image_filtered.data , image_filtered.cols * image_filtered.rows);

            // Scan the image to find QR code
            if(scanner.scan(zbar_image)==1)
            {
                // Get scan data.
                data = zbar_image.symbol_begin()->get_data();
            }
            else
            {
                data = "QR_Contour_Detected";
            }
        }

        /*
         * Publish
         */
        // Image
        sensor_msgs::ImagePtr image_msg;
        image_msg = cv_bridge::CvImage (std_msgs::Header(), "mono8", image_filtered).toImageMsg();
        pub_image_qr_.publish (image_msg);

        // Data
        std_msgs::String msg;
        msg.data = data;
        pub_qr_.publish (msg);

        // Signal interrupt point
        boost::this_thread::interruption_point();
    }

    bool qrContourSearch(cv::Mat imageGray_in)
    {
        // Threshold image
        cv::Mat image_binary;
        cv::threshold(imageGray_in, image_binary, qrSquareThresh_, 255, CV_THRESH_BINARY);

        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        // Find only outermost point for contour.
        cv::findContours(image_binary, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        // Find contours with largest OBB area
        double largestConArea = 0.0;
        int largestConAreaIdx = -1;
        for(unsigned int i = 0; i<contours.size();i++)
        {
            double conArea = cv::contourArea(contours[i]);
            if(conArea > largestConArea)
            {
                // Check if contour area is above a min area.
                if(conArea > minQRConArea_)
                {
                    // Check if contour area contains black
                    // Create mask
                    cv::Mat mask = cv::Mat::zeros(imageGray_in.size(),imageGray_in.type());
                    cv::drawContours(mask,contours,i,cv::Scalar(255),CV_FILLED);
                    // Mask area on image so that we only search inside contour
                    cv::Mat masked_area;
                    cv::bitwise_not(imageGray_in,masked_area,mask);
                    //Find number of black pixels in image:
                    cv::Mat bin_im;
                    cv::threshold(masked_area,bin_im,200,255,CV_THRESH_BINARY);
                    unsigned black_area = countNonZero(bin_im);

                    if(black_area > minQRBlackArea_)
                    {
                        //std::cout<<"White: " << conArea << ", Black:" << black_area << std::endl;
                        largestConArea = conArea;
                        largestConAreaIdx = i;
                    }
                }
            }
        }
        // Return true if contour was found
        if(largestConAreaIdx != -1)
            return true;
       return false;
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
    ros::ServiceClient srv_mr_camera_enable;
    std::string sub_image_name_;
    std::string pub_image_qr_name_, pub_image_line_name_, pub_line_name_, pub_qr_name_;
    std::string srv_enable_name_, srv_mr_camera_enable_name_;
	
	bool enabled_;
    double minQRConArea_;
    int minQRBlackArea_;
    int qrSquareThresh_;

    // Threads
    boost::thread* qrThread_;
    boost::thread* lineThread_;
};

/**
 * Main
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "MR_Camera_Processing");
    ImageConverter ic;

    // Sleep rate
    ros::Rate r(30);

    // ROS Spin: Handle callbacks
    while(!ros::isShuttingDown())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
