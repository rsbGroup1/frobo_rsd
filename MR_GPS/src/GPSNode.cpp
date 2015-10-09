#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>


using namespace std;

#define BUFFER_SIZE 50

bool searchForMarker = true;

void stateCallback(const std_msgs::Int8 &stateMsg)
{
    if(stateMsg.data == 1)
    {
        searchForMarker = true;
        cout << "Enable pose" << endl;
    }
    else
    {
        searchForMarker = false;
        cout << "Disabling pose" << endl;
    }
}

bool getPoseFromCamLocalizer(geometry_msgs::PoseStamped &newPose)
{
    // Create network variables
    string requestMsg = "Get position 5";
    struct sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("10.115.253.233");
    addr.sin_port = htons(21212);

    int s = socket(AF_INET, SOCK_STREAM, 0);
    ROS_INFO("Connecting");
    connect(s, (sockaddr*)&addr, sizeof(addr));
    char buffer[BUFFER_SIZE];
    ros::Time requestRosTime = ros::Time::now();
    time_t timer;
    timer = time(NULL);
    //cout << "RosTime: " << requestRosTime << "\tSystem time: " << timer << endl;
    int writeSize = write(s,requestMsg.c_str(),requestMsg.length());
    int readSize = read(s,buffer,BUFFER_SIZE);

    bool returnValue = false;
    if(writeSize < 0)
        ROS_ERROR("NO CONNECTION TO CAM-LOCALIZER!");

    geometry_msgs::PoseStamped pose;
    if(readSize <= 0)
    {
        ROS_ERROR("NO MESSAGE FROM CAM-LOCALIZER!");
    }
    else
    {
        buffer[readSize] = 0;
        long int serverTimeStamp;
        double serverTime;
        float x, y, angle;
        int markerOrder;
        int commasPast = 0;
        string currentBuilding = "";
        for(int i = 0; i < readSize;i++)
        {
            if(buffer[i] == ',')
            {
                switch (commasPast) {
                case 0: // makerNumber
                    markerOrder = atoi(currentBuilding.c_str());
                    break;
                case 1: // timestamp
                    serverTimeStamp = atol(currentBuilding.c_str());
                    serverTime = (double) serverTimeStamp / 1000.0;
                    break;
                case 2: // x
                    x = atof(currentBuilding.c_str());
                    break;
                case 3: // y
                    y = atof(currentBuilding.c_str());
                    break;
                case 4: // angle
                    angle = atof(currentBuilding.c_str());
                    break;
                default:
                    break;
                }
                currentBuilding = "";
                commasPast++;
            }
            else
            {
                if(commasPast==1 && buffer[i] == '.')
                {}
                else
                    currentBuilding += buffer[i];
            }
        }

        //cout << "Receiveds: " << markerOrder << " " << serverTime << " " << x << " " << y << " " << angle << endl;
        //cout << "TimeDifference: " << requestRosTime.toSec() - serverTime << endl;
        newPose.header.stamp = requestRosTime;
        newPose.pose.position.x = x;
        newPose.pose.position.y = y;
        newPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle * (M_PI / 180));
        cout << newPose << endl;
        returnValue = true;
    }
    close(s);
    return returnValue;
}


int main(int argc, char** argv)
{
    srand(time(NULL));


    ros::init(argc, argv, "camLocalisation");
    ros::NodeHandle n;


    // Set parameters
    int port;


    //Publisher
    ros::Publisher statePublisher = n.advertise<geometry_msgs::PoseStamped>("gps_pose",10);

    //Subscribe to topic
    ros::Subscriber stateSubscriber = n.subscribe("state",1,stateCallback);

    geometry_msgs::PoseStamped pStamped;
    // Spin
    ros::Rate r(2);
    while (ros::ok()) {

        ros::spinOnce();
        if(searchForMarker)
        {
            //cout << "Getting pose"<< endl;
            if(getPoseFromCamLocalizer(pStamped)){
                statePublisher.publish(pStamped);
            }
        }
        r.sleep();



        // cv::imshow("Lines",currentImg);
    }

    return 0;
}
