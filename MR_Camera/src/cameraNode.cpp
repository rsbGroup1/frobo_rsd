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

#define CAMERA_FREQUENCY		15      // FPS
#define AUTO_FOCUS              false   // Set autofocus: True/false
#define FOCUS                   0       // Set focus to a specific value. High values for nearby objects and low values for distant objects.
#define SHARPNESS               200     // Sharpness (int): min=0 max=255 step=1 default=128 value=128


// Global variables
cv::VideoCapture *_camera;

int main()
{
    // Loop through possible camera serial devices
    for(int i=0; i<2; i++)
    {
        // Open the video camera no. i
        _camera = new cv::VideoCapture(i);
        _camera->set(CV_CAP_PROP_FRAME_WIDTH, 1024); //1920, 1280, 1024, 640
        _camera->set(CV_CAP_PROP_FRAME_HEIGHT, 576); //1080, 720, 576, 480
        _camera->set(CV_CAP_PROP_FPS, CAMERA_FREQUENCY);

        // Change camera parameters
        std::string msg = "v4l2-ctl -d " + SSTR(i) + " -c focus_auto=" + (AUTO_FOCUS?"1":"0");
        std::system(msg.c_str());
        msg = "v4l2-ctl -d " + SSTR(i) + " -c focus_absolute=" + SSTR(FOCUS);
        std::system(msg.c_str());
        msg = "v4l2-ctl -d " + SSTR(i) + " -c sharpness=" + SSTR(SHARPNESS);
        std::system(msg.c_str());

        // If not success, exit program
        if(!_camera->isOpened())
        {
            delete _camera;
            std::cerr << "Error opening camera feed..!" << std::endl;
        }
        else
        {
            std::cout << "Camera " << i << " opened!" << std::endl;
            break;
        }
    }

    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_Camera_Node");
    ros::NodeHandle nh;

    // Create publisher topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

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
