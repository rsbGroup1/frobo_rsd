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
bool _connected = false;
bool _waitForServer = true;
bool _waitForStatus = false;

// Functions
void sendMsgCallback (std_msgs::String msg)
{
    if(_connected)
    {
        // Send data
        write(_socket, msg.data.c_str(), msg.data.size());
        _waitForServer = true;
        _waitForStatus = true;
    }
}

bool connectToServer()
{
    // Create network variables
    struct sockaddr_in addr;
    bzero (&addr, sizeof (addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr (_serverIP.c_str());
    addr.sin_port = htons (_serverPort);

    // Create socket
    _socket = socket (AF_INET, SOCK_STREAM, 0);

    // Connect
    connect (_socket, (sockaddr*) &addr, sizeof (addr));

    // Test connection
<<<<<<< HEAD
    int writeSize = write(_socket, "MR1", 4);
    if(writeSize < 0)
    {
        _connected = false;
=======
    int writeSize = write (_socket, "Cell 1", 7);

    if (writeSize < 0)
>>>>>>> 893c4c9b319c56bcc9b6da73754f4852c4e86178
        return false;
    }
    else
    {
        _connected = true;
        return true;
    }
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init (argc, argv, "MR_MES_Client");
    ros::NodeHandle nh;
    ros::NodeHandle pNh ("~");

    // Topic names
    std::string mesSub, mesPub;
<<<<<<< HEAD
    pNh.param<std::string>("mesPub", mesPub, "/mrMESClient/msgFromServer");
    pNh.param<std::string>("mesSub", mesSub, "/mrMESClient/msgToServer");
    pNh.param<std::string>("server_ip", _serverIP, "127.0.0.1");//10.115.253.233");
    pNh.param<int>("server_port", _serverPort, 21240);
=======
    pNh.param<std::string> ("mesPub", mesPub, "/mrMESClient/msgFromServer");
    pNh.param<std::string> ("mesSub", mesSub, "/mrMESClient/msgToServer");
    pNh.param<std::string> ("server_ip", _serverIP, "10.115.253.233");
    pNh.param<int> ("server_port", _serverPort, 21240);
>>>>>>> 893c4c9b319c56bcc9b6da73754f4852c4e86178

    // Publishers
    _mesMessagePub = nh.advertise<mr_mes_client::server> (mesPub, 100);

    // Subscribers
    ros::Subscriber mesMessageSub = nh.subscribe (mesSub, 10, sendMsgCallback);

    // Sleep rate
    ros::Rate r (10);

    // Connect to server
    if (connectToServer() == false)
    {
        ROS_ERROR ("No connection to MES Server!");
        return -1;
    }

    // Set loop rate
    while (ros::ok())
    {
<<<<<<< HEAD
        if(_waitForServer)
        {
            char buffer[BUFFER_SIZE];
            int readSize = read(_socket, buffer, BUFFER_SIZE);

            if(readSize <= 0)
            {
                ROS_ERROR("No message from MES Server!");
                break;
            }
            else
            {
                std::string MESServer = "</MESServer>";
                std::string msg(buffer);
                size_t found = msg.find(MESServer);
                msg = msg.substr(0, found+12);
                std::cout << msg << std::endl;

                // Open document
                tinyxml2::XMLDocument doc;
                if(doc.Parse(msg.c_str()) != 0)
                {
                    ROS_ERROR("Error parsing string!");
                    break;
                }

                int cell, mobileRobot, red, blue, yellow, status;

                // Check if "MESServer"
                if(std::string(doc.RootElement()->Value()) == "MESServer")
                {
                    mr_mes_client::server msg;

                    if(_waitForStatus)
                    {
                        // Get stuff
                        status = atoi(doc.RootElement()->FirstChildElement("Status")->FirstChild()->Value());

                        msg.mobileRobot = 1;
                        msg.status = status;

                        // Reset
                        _waitForStatus = false;
                    }
                    else
                    {
                        // Get stuff
                        cell = atoi(doc.RootElement()->FirstChildElement("Cell")->FirstChild()->Value());
                        mobileRobot = atoi(doc.RootElement()->FirstChildElement("MobileRobot")->FirstChild()->Value());
                        red = atoi(doc.RootElement()->FirstChildElement("Red")->FirstChild()->Value());
                        blue = atoi(doc.RootElement()->FirstChildElement("Blue")->FirstChild()->Value());
                        yellow = atoi(doc.RootElement()->FirstChildElement("Yellow")->FirstChild()->Value());

                        msg.blue = blue;
                        msg.cell = cell;
                        msg.mobileRobot = mobileRobot;
                        msg.yellow = yellow;
                        msg.red = red;
                    }

                    _mesMessagePub.publish(msg);
                }

                _waitForServer = false;
=======
        char buffer[BUFFER_SIZE];
        int readSize = read (_socket, buffer, BUFFER_SIZE);

        if (readSize <= 0)
        {
            ROS_ERROR ("No message from MES Server!");
        }
        else
        {
            std::string msg (buffer);
            msg = msg.substr (0, msg.size() - 1);
            std::cout << msg << std::endl;

            // Open document
            tinyxml2::XMLDocument doc;

            if (doc.Parse (msg.c_str()) != 0)
            {
                ROS_ERROR ("Error parsing string!");
                return false;
            }

            int cell, mobileRobot, red, blue, yellow;

            // Check if "MESServer"
            if (std::string (doc.RootElement()->Value()) == "MESServer")
            {
                // Get stuff
                cell = atoi (doc.RootElement()->FirstChildElement ("Cell")->FirstChild()->Value());
                mobileRobot = atoi (doc.RootElement()->FirstChildElement ("MobileRobot")->FirstChild()->Value());
                red = atoi (doc.RootElement()->FirstChildElement ("Red")->FirstChild()->Value());
                blue = atoi (doc.RootElement()->FirstChildElement ("Blue")->FirstChild()->Value());
                yellow = atoi (doc.RootElement()->FirstChildElement ("Yellow")->FirstChild()->Value());

                mr_mes_client::server msg;
                msg.blue = blue;
                msg.cell = cell;
                msg.mobileRobot = mobileRobot;
                msg.yellow = yellow;
                msg.red = red;
                _mesMessagePub.publish (msg);
>>>>>>> 893c4c9b319c56bcc9b6da73754f4852c4e86178
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    // Return
    close (_socket);
    return 0;
}
