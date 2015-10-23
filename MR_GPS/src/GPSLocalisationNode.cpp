// Includes

#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <queue>
#include <cstdlib>
#include <time.h>
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace ros;

boost::circular_buffer<pair<geometry_msgs::Pose, ros::Time> > cb(100);
pair<geometry_msgs::Pose, ros::Time> current;

Publisher gpsOdomCombinedLocalisationPublisher;

void odomCallback(nav_msgs::Odometry odomPoseMsg)
{
    pair<geometry_msgs::Pose, ros::Time> next;
    next.first = odomPoseMsg.pose.pose;
    next.second = odomPoseMsg.header.stamp;
    cb.push_back(next);
    current = next;
}

void gpsFixCallBack(geometry_msgs::PoseStamped gpsPose)
{
    geometry_msgs::PoseWithCovarianceStamped gpsFixPoseEstimate;

    // find poseAtRequestTime
    bool odomMovementFound = false;
    pair<geometry_msgs::Pose, ros::Time> prevFront;
    if(cb.size() > 0)
    {
        odomMovementFound = true;
        cout << "odom found" << endl;
        prevFront = cb.front();
        if(gpsPose.header.stamp >= cb.front().second)
        {
            while(gpsPose.header.stamp > prevFront.second && cb.size() > 0)
            {
                prevFront = cb.front();
                cb.pop_front();
            }
        }
    }

    // calculate Odom difference
    double deltaX = 0;
    double deltaY = 0;
    if(odomMovementFound)
    {
        deltaX = (current.first.position.x - prevFront.first.position.x);
        deltaY = (current.first.position.y - prevFront.first.position.y);
    }
    gpsFixPoseEstimate.pose.pose.position.x = gpsPose.pose.position.x + deltaX;
    gpsFixPoseEstimate.pose.pose.position.y = gpsPose.pose.position.y + deltaY;
    tf::Quaternion gpsQaut, odomCurrentQuat, odomOldQuat;
    if(odomMovementFound)
    {
        tf::quaternionMsgToTF(current.first.orientation,odomCurrentQuat);
        tf::quaternionMsgToTF(prevFront.first.orientation,odomOldQuat);
    }
    tf::quaternionMsgToTF(gpsPose.pose.orientation,gpsQaut);
    double deltaYaw = 0;
    if(odomMovementFound)
    {
        deltaYaw = (tf::getYaw(odomCurrentQuat) - tf::getYaw(odomOldQuat));
    }
    double newYaw = tf::getYaw(gpsQaut) + deltaYaw;
    gpsFixPoseEstimate.pose.pose.orientation = tf::createQuaternionMsgFromYaw(newYaw);
    cout << "new Yaw: "  << newYaw<< endl;
    // create covariance
    if(odomMovementFound)
    {
        gpsFixPoseEstimate.pose.covariance[0] = sqrt(abs(current.first.position.x - prevFront.first.position.x)); // X
        gpsFixPoseEstimate.pose.covariance[7] = sqrt(abs(current.first.position.y - prevFront.first.position.y)); // Y
        gpsFixPoseEstimate.pose.covariance[35] = sqrt(abs(newYaw)); // Yaw
    }
    else
    {
        gpsFixPoseEstimate.pose.covariance[0]  = 1.5;// X
        gpsFixPoseEstimate.pose.covariance[7]  = 1.5; // Y
        gpsFixPoseEstimate.pose.covariance[35] = 0.5; // Yaw
    }

    // publish new estimated pose
    gpsFixPoseEstimate.header.stamp = ros::Time::now();
    gpsOdomCombinedLocalisationPublisher.publish(gpsFixPoseEstimate);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpsLocalisation");
    ros::NodeHandle n(ros::this_node::getName() + "/");

    string odomTopic = "odom";
    string gpsTopic = "/mrGPS/Pose";
    string gpsPublishTopic = "initialpose";

    n.param<std::string>("odomTopic", odomTopic, "odom");
    n.param<std::string>("gpsPoseTopic", gpsTopic, "/mrGPS/Pose");
    n.param<std::string>("filterResetTopic", gpsPublishTopic, "initialpose");

    ros::Subscriber odomSubscriber = n.subscribe(odomTopic,20,odomCallback);
    ros::Subscriber gpsSubscriber = n.subscribe(gpsTopic,5,gpsFixCallBack);
    gpsOdomCombinedLocalisationPublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(gpsPublishTopic,10);

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}
