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
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;

boost::circular_buffer<pair<geometry_msgs::Pose, ros::Time> > cb (100);
pair<geometry_msgs::Pose, ros::Time> current;

Publisher gpsOdomCombinedLocalisationPublisher;
std::string gpsFrame;
tf::TransformListener* tfListener;

void odomCallback (nav_msgs::Odometry odomPoseMsg)
{
    pair<geometry_msgs::Pose, ros::Time> next;
    next.first = odomPoseMsg.pose.pose;
    next.second = odomPoseMsg.header.stamp;
    cb.push_back (next);
    current = next;
}

void gpsFixCallBack (geometry_msgs::PoseStamped gpsPose)
{
    geometry_msgs::PoseStamped gpsFixPoseEstimate;

    // find poseAtRequestTime
    bool odomMovementFound = false;
    pair<geometry_msgs::Pose, ros::Time> prevFront;

    if (cb.size() > 0)
    {
        odomMovementFound = true;
        cout << "odom found" << endl;
        prevFront = cb.front();

        if (gpsPose.header.stamp >= cb.front().second)
        {
            while (gpsPose.header.stamp > prevFront.second && cb.size() > 0)
            {
                prevFront = cb.front();
                cb.pop_front();
            }
        }
    }

    // calculate Odom difference
    double deltaX = 0;
    double deltaY = 0;

    if (odomMovementFound)
    {
        deltaX = (current.first.position.x - prevFront.first.position.x);
        deltaY = (current.first.position.y - prevFront.first.position.y);
    }

    gpsFixPoseEstimate.pose.position.x = gpsPose.pose.position.x + deltaX;
    gpsFixPoseEstimate.pose.position.y = gpsPose.pose.position.y + deltaY;
    tf::Quaternion gpsQaut, odomCurrentQuat, odomOldQuat;

    if (odomMovementFound)
    {
        tf::quaternionMsgToTF (current.first.orientation, odomCurrentQuat);
        tf::quaternionMsgToTF (prevFront.first.orientation, odomOldQuat);
    }

    tf::quaternionMsgToTF (gpsPose.pose.orientation, gpsQaut);
    double deltaYaw = 0;

    if (odomMovementFound)
    {
        deltaYaw = (tf::getYaw (odomCurrentQuat) - tf::getYaw (odomOldQuat));
    }

    double newYaw = tf::getYaw (gpsQaut) + deltaYaw;
    gpsFixPoseEstimate.pose.orientation = tf::createQuaternionMsgFromYaw (newYaw);
    cout << "new Yaw: "  << newYaw << endl;
    // create covariance


    // publish new estimated pose
    gpsFixPoseEstimate.header.stamp = ros::Time::now();
    gpsFixPoseEstimate.header.frame_id = gpsFrame;
    geometry_msgs::PoseStamped inMapFrame;

    try
    {
        tfListener->transformPose ("/map", ros::Time::now(), gpsFixPoseEstimate, gpsFrame, inMapFrame);
        geometry_msgs::PoseWithCovarianceStamped resultPose;
        resultPose.pose.pose = inMapFrame.pose;
        resultPose.header = inMapFrame.header;


        if (odomMovementFound)
        {
            resultPose.pose.covariance[0] = sqrt (abs (current.first.position.x - prevFront.first.position.x)); // X
            resultPose.pose.covariance[7] = sqrt (abs (current.first.position.y - prevFront.first.position.y)); // Y
            resultPose.pose.covariance[35] = sqrt (abs (newYaw)); // Yaw
        }
        else
        {
            resultPose.pose.covariance[0]  = 0.1;// X
            resultPose.pose.covariance[7]  = 0.1; // Y
            resultPose.pose.covariance[35] = 0.1; // Yaw
        }

        gpsOdomCombinedLocalisationPublisher.publish (resultPose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR ("%s", ex.what());
    }

}

int main (int argc, char** argv)
{
    ROS_DEBUG ("Init");
    ros::init (argc, argv, "gpsLocalisation");
    ROS_DEBUG ("Node handle");
    ros::NodeHandle n, pn ("~");

    tfListener = new tf::TransformListener;
    string odomTopic, gpsTopic, gpsPublishTopic;

    pn.param<std::string> ("odomTopic", odomTopic, "odom");
    pn.param<std::string> ("gpsPoseTopic", gpsTopic, "/mrGPS/Pose");
    pn.param<std::string> ("filterResetTopic", gpsPublishTopic, "initialpose");
    pn.param<std::string> ("gpsFrame", gpsFrame, "gps_frame");

    ROS_DEBUG ("Subscriber and publisher");
    ros::Subscriber odomSubscriber = n.subscribe (odomTopic, 20, odomCallback);
    ros::Subscriber gpsSubscriber = n.subscribe (gpsTopic, 5, gpsFixCallBack);
    gpsOdomCombinedLocalisationPublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped> (gpsPublishTopic, 10);

    ros::Rate rate (10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}
