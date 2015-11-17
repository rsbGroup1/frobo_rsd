#include "ros/ros.h"
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>



using namespace std;

double covX, covY, covZ, covQx, covQy, covQz;
ros::Publisher odomWithCov;
nav_msgs::Odometry newOdomMsg;

void odomCallBack (const nav_msgs::Odometry::ConstPtr& msg)
{
    newOdomMsg = *msg;
    newOdomMsg.pose.covariance[0] = (float) covX;
    newOdomMsg.pose.covariance[7] = (float) covY;
    newOdomMsg.pose.covariance[14] = (float) covZ;
    newOdomMsg.pose.covariance[21] = (float) covQx;
    newOdomMsg.pose.covariance[28] = (float) covQy;
    newOdomMsg.pose.covariance[35] = (float) covQz;

    // publish
    odomWithCov.publish (newOdomMsg);
}


int main (int argc, char** argv)
{

    ros::init (argc, argv, "covarianceNode");
    ros::NodeHandle nh ("~");


    nh.param<double> ("covX", covX, 0.75);
    nh.param<double> ("covY", covY, 0.75);
    nh.param<double> ("covZ", covZ, 0.75);
    nh.param<double> ("covVx", covQx, 0.75);
    nh.param<double> ("covVy", covQy, 0.75);
    nh.param<double> ("covVz", covQz, 0.75);


    string publishTopic, subscribeTopic;
    nh.param<string> ("publishTopic", publishTopic, "odom_with_cov");
    nh.param<string> ("subscribeTopic", subscribeTopic, "odom");

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry> (subscribeTopic.c_str(), 1000, odomCallBack);
    odomWithCov = nh.advertise<nav_msgs::Odometry> (publishTopic.c_str(), 1000);

    ros::Rate r (50);

    while (ros::ok())
    {

        ros::spinOnce();
        r.sleep();
        // cv::imshow("Lines",currentImg);
    }
}
