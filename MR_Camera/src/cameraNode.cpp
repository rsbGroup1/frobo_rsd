// Includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv/cv.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "mr_camera/enable.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global variables
cv::VideoCapture* _camera;
bool _enabled;

/**
 * Enables or disables the image processing by subscribing or
 * shutingdown the image subscriber
 */
bool enableCallback(mr_camera::enable::Request& req, mr_camera::enable::Response& res)
{
    _enabled = req.enable;
    res.done = _enabled;
    return true;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init (argc, argv, "MR_Camera");
    ros::NodeHandle nh;
    ros::NodeHandle pNh ("~");

    std::string imagePub, enableSrv;
    int frameWidth, frameHeight, cameraFrequency, sharpness, brightness, whiteBalanceTemp;
    bool whiteBalanceAuto;
    pNh.param<std::string> ("image_pub", imagePub, "/mrCamera/image");
    pNh.param<std::string> ("enable", enableSrv, "/mrCamera/enable");
    pNh.param<int> ("frameWidth", frameWidth, 640);
    pNh.param<int> ("frameHeight", frameHeight, 480);
    pNh.param<int> ("cameraFrequency", cameraFrequency, 30);
    pNh.param<int> ("sharpness", sharpness, 2);
    pNh.param<int> ("brightness", brightness, 2);
    pNh.param<int> ("whiteBalanceTemp", whiteBalanceTemp, 4600);
    pNh.param<bool> ("whiteBalanceAuto", whiteBalanceAuto, true);

    // Create publisher topic
    image_transport::ImageTransport it (nh);
    image_transport::Publisher pub = it.advertise (imagePub, 1);

    // Create the service
    ros::ServiceServer srvEnabler = nh.advertiseService(enableSrv, enableCallback);

    // Set loop rate
    ros::Rate loop_rate (cameraFrequency);

    // Loop through possible camera serial devices
    for (int i = 0; i < 2; i++)
    {
        // Open the video camera no. i
        _camera = new cv::VideoCapture (i);
        _camera->set (CV_CAP_PROP_FRAME_WIDTH, frameWidth);
        _camera->set (CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
        _camera->set (CV_CAP_PROP_FPS, cameraFrequency);

        // Change camera parameters
        std::string msg = "v4l2-ctl -d " + SSTR (i) + " -c sharpness=" + SSTR (sharpness);
        std::system (msg.c_str());
        msg = "v4l2-ctl -d " + SSTR (i) + " -c brightness=" + SSTR (brightness);
        std::system (msg.c_str());
        msg = "v4l2-ctl -d " + SSTR (i) + " -c white_balance_temperature_auto=" + (whiteBalanceAuto ? "1" : "0");
        std::system (msg.c_str());

        if (!whiteBalanceAuto)
        {
            msg = "v4l2-ctl -d " + SSTR (i) + " -c white_balance_temperature=" + SSTR (whiteBalanceTemp);
            std::system (msg.c_str());
        }

        // If not success, exit program
        if (!_camera->isOpened())
        {
            delete _camera;
            ROS_ERROR ("Error opening camera feed!");
            return -1;
        }
        else
        {
            ROS_INFO ("Camera opened!");
            break;
        }
    }

    // Spin
    cv::Mat _image;

    while (!ros::isShuttingDown())
    {
        if (_enabled && _camera->read (_image))
        {
            // Convert to ROS format
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage (std_msgs::Header(), "bgr8", _image).toImageMsg();

            // Publish to topic
            pub.publish (msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Delete camera object
    _camera->release();
    delete _camera;

    // Return
    return 0;
}
