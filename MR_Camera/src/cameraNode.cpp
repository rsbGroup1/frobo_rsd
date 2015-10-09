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

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

#define FRAME_WIDTH             640     // 1280, 960, 640, 640, 320, 160
#define FRAME_HEIGHT            480     // 720, 544, 480, 360, 240, 120
#define CAMERA_FREQUENCY        15      // max: 10,  15,  30,  30,  30, 30
#define SHARPNESS               2       // (int): min=1 max=7 step=1 default=2
#define BRIGHTNESS              2       // (int): min=1 max=7 step=1 default=2
#define WHITE_BALANCE_AUTO      true    // (bool): default=1
#define WHITE_BALANCE_TEMP      4600    // (int): min=2800 max=6500 step=1 default=4600

// You can also control: Hue, contrast, saturation, gamma, power line freq, backlight compensation, exposure etc. type "v4l2-ctl --all" in console for more info

// Global variables
cv::VideoCapture *_camera;

int main()
{
    // Loop through possible camera serial devices
    for(int i=0; i<2; i++)
    {
        // Open the video camera no. i
        _camera = new cv::VideoCapture(i);
        _camera->set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH); 
        _camera->set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT); 
        _camera->set(CV_CAP_PROP_FPS, CAMERA_FREQUENCY);

        // Change camera parameters
        std::string msg = "v4l2-ctl -d " + SSTR(i) + " -c sharpness=" + SSTR(SHARPNESS);
        std::system(msg.c_str());
        msg = "v4l2-ctl -d " + SSTR(i) + " -c brightness=" + SSTR(BRIGHTNESS);
        std::system(msg.c_str());
        msg = "v4l2-ctl -d " + SSTR(i) + " -c white_balance_temperature_auto=" + (WHITE_BALANCE_AUTO?"1":"0");
        std::system(msg.c_str());
        if(!WHITE_BALANCE_AUTO)
        {
           msg = "v4l2-ctl -d " + SSTR(i) + " -c white_balance_temperature=" + SSTR(WHITE_BALANCE_TEMP);
           std::system(msg.c_str());
        }

        // If not success, exit program
        if(!_camera->isOpened())
        {
            delete _camera;
            ROS_ERROR("Error opening camera feed!");
        }
        else
        {
            ROS_INFO("Camera opened!");
            break;
        }
    }

    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_Camera_Node");
    ros::NodeHandle nh, pNh("~");

    std::string imagePub;
    pNh.param<std::string>("image_pub", imagePub, "/rcCamera/image");

    // Create publisher topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(imagePub, 1);

    // Set loop rate
    ros::Rate loop_rate(CAMERA_FREQUENCY);

    // Spin
    cv::Mat _image;

    while(nh.ok())
    {
        if(_camera->read(_image))
        {
            // Convert to ROS format
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _image).toImageMsg();

            // Publish to topic
            pub.publish(msg);
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
