// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>

// Global variables
ros::Publisher _mesMessagePub;
std::string _serverIP;
int _serverPort;

// Functions
void sendMsgCallback(std_msgs::String msg)
{
    // Construct and send message to server
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "mr_messerver");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string mesSub, mesPub;
    pNh.param<std::string>("mesPub", mesPub, "/mrMESServer/msgFromServer");
    pNh.param<std::string>("mesSub", mesSub, "/mrMESServer/msgToServer");
    pNh.param<std::string>("server_ip", _serverIP, "192.168.100.124");
    pNh.param<int>("server_port", _serverPort, 14141);

    // Publishers
    _mesMessagePub = nh.advertise<std_msgs::String>(mesPub, 100);

    // Subscribers
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, sendMsgCallback);

    // Set loop rate
    ros::Rate loop_rate(10);

    while(nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Return
    return 0;
}
