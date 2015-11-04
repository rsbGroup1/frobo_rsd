// Includes
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "tinyxml2.h"
#include "mr_mes_client/server.h"

// Constants
const int BUFFER_SIZE = 1024;

// Global variables
ros::Publisher _mesMessagePub;
std::string _serverIP;
int _serverPort;
int _socket;

// Functions
void sendMsgCallback(std_msgs::String msg)
{
    // Construct and send message to server
}

bool connectToServer()
{
    // Create network variables
    struct sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(_serverIP.c_str());
    addr.sin_port = htons(_serverPort);

    // Create socket
    _socket = socket(AF_INET, SOCK_STREAM, 0);

    // Connect
    connect(_socket, (sockaddr*)&addr, sizeof(addr));

    // Test connection
    int writeSize = write(_socket, "Cell 1", 7);
    if(writeSize < 0)
        return false;
    else
        return true;
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "MR_MES_Client");
    ros::NodeHandle nh;
    ros::NodeHandle pNh(ros::this_node::getName() + "/");

    // Topic names
    std::string mesSub, mesPub;
    pNh.param<std::string>("mesPub", mesPub, "/mrMESClient/msgFromServer");
    pNh.param<std::string>("mesSub", mesSub, "/mrMESClient/msgToServer");
    pNh.param<std::string>("server_ip", _serverIP, "10.115.253.233");
    pNh.param<int>("server_port", _serverPort, 21240);

    // Publishers
    _mesMessagePub = nh.advertise<mr_mes_client::server>(mesPub, 100);

    // Subscribers
    ros::Subscriber mesMessageSub = nh.subscribe(mesSub, 10, sendMsgCallback);

    // Sleep rate
    ros::Rate r(10);

    // Connect to server
    if(connectToServer() == false)
    {
        ROS_ERROR("No connection to MES Server!");
        return -1;
    }

    // Set loop rate
    while(ros::ok())
    {
        char buffer[BUFFER_SIZE];
        int readSize = read(_socket, buffer, BUFFER_SIZE);

        if(readSize <= 0)
        {
            ROS_ERROR("No message from MES Server!");
        }
        else
        {
            std::string msg(buffer);
            msg = msg.substr(0, msg.size()-1);
            std::cout << msg << std::endl;

            // Open document
            tinyxml2::XMLDocument doc;
            if(doc.Parse(msg.c_str()) != 0)
            {
                ROS_ERROR("Error parsing string!");
                return false;
            }

            int cell, mobileRobot, red, blue, yellow;

            // Check if "MESServer"
            if(std::string(doc.RootElement()->Value()) == "MESServer")
            {
                // Get stuff
                cell = atoi(doc.RootElement()->FirstChildElement("Cell")->FirstChild()->Value());
                mobileRobot = atoi(doc.RootElement()->FirstChildElement("MobileRobot")->FirstChild()->Value());
                red = atoi(doc.RootElement()->FirstChildElement("Red")->FirstChild()->Value());
                blue = atoi(doc.RootElement()->FirstChildElement("Blue")->FirstChild()->Value());
                yellow = atoi(doc.RootElement()->FirstChildElement("Yellow")->FirstChild()->Value());

                mr_mes_client::server msg;
                msg.blue = blue;
                msg.cell = cell;
                msg.mobileRobot = mobileRobot;
                msg.yellow = yellow;
                msg.red = red;
                _mesMessagePub.publish(msg);
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    // Return
    close(_socket);
    return 0;
}
