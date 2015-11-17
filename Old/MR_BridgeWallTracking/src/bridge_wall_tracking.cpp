#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
//#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include "linreg.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>

using namespace std;
using namespace cv;

#define PI 3.14159265
#define ROW_WIDTH 120

struct LinePolar
{
    double distance;
    double normalVectorAngle;

    LinePolar (double dist, double ang)
    {
        distance = dist;
        normalVectorAngle = ang;
    }
};

struct Line
{
    double a;
    double b;

    Line (double A, double B)
    {
        a = A;
        b = B;
    }

    Line (Point2d p1, Point2d p2)
    {
        double denom = (p2.x - p1.x);

        if (denom > 0.00001)
            a = (p2.y - p1.y) / (p2.x - p1.x);
        else
            a = 9999999;

        b = p2.y - a * p2.x;
    }

    Line (Vec4f vectorAndPoint)
    {
        double denom = vectorAndPoint[0];

        if (denom > 0.00001)
            a = vectorAndPoint[1] / (denom);
        else
            a = 9999999;

        b = vectorAndPoint[2] - a * vectorAndPoint[3];
    }

    double getY (double x)
    {
        return a * x + b;
    }

    double distanceToPoint (Point2d p)
    {
        return abs ( (-a) * p.x + p.y - b) / sqrt (1 + (-a) * (-a));
    }

    void print (bool newLine = true)
    {
        cout << "y=" << a << "x + " << b;

        if (newLine)
            cout << endl;
    }
};

bool newImage = false;
ros::Publisher img_publisher, angleOnWalls, distanceOfCenter, visualOdometry;
bool runOnce = false;
double minScoreForPublish, covX, covYaw, ransacThreshold, laserMaxDistance;

void FindBestLineRemoveInliers (vector<Point2d>& points, vector<Line>& lines, double inlierTreshold, int iterations = 30, bool removeInliers = true)
{
    Line best (0, 0);
    int bestInliers = 0;

    if (points.size() < 3)
        return;

    for (int i = 0; i < iterations; i++)
    {
        // Pick two random
        int index2, index1 = rand() % points.size();

        do
        {
            index2 = rand() % points.size();
        }
        while (index2 == index1);

        // Calculate model
        Line testLine (points[index1], points[index2]);

        // Find inlier
        int inliers = 0;

        for (int pointIndex = 0; pointIndex < points.size(); pointIndex++)
        {
            if (testLine.distanceToPoint (points[pointIndex]) < inlierTreshold)
            {

                inliers++;
            }
        }

        if (inliers > bestInliers)
        {
            best.a = testLine.a;
            best.b = testLine.b;
            bestInliers = inliers;
        }
    }

    vector<Point2d> pointsNew;
    vector<Point2f> inliersVec;

    for (int i = 0; i < points.size(); i++)
    {
        if (best.distanceToPoint (points[i]) >= inlierTreshold)
        {
            pointsNew.push_back (points[i]);
        }
        else
        {
            Point2f p ( (float) points[i].x, (float) points[i].y);
            inliersVec.push_back (p);
        }
    }

    LinearRegression lr;

    for (int i = 0; i < inliersVec.size(); i++)
    {
        lr.addXY (inliersVec[i].x, inliersVec[i].y);
    }

    Line bestFit (lr.getB(), lr.getA());

    if (removeInliers)
    {
        points = pointsNew;
    }

    if (bestInliers > 0)
        lines.push_back (bestFit);
}

pair<Line, Line> getLines (const sensor_msgs::LaserScanPtr& scanMsg, double& qualityMeasure)
{
    // Method 1
    cv::Mat img (900, 900, CV_8U, cv::Scalar (0, 0, 0));
    // insert points
    vector<Point2d> points, pointsRight;

    for (int i = 0; i < scanMsg->ranges.size(); i++)
    {
        // Calculate x and y
        double x, y;

        if (scanMsg->ranges[i] < laserMaxDistance && scanMsg->ranges[i] > 0.05)
        {
            //x = 450 + (scanMsg->ranges[i] * 100) * cos(((225 + (3 * i)) % 360) * PI / 180);
            // y = 450 + (scanMsg->ranges[i] * 100) * sin(((225 + (3 * i)) % 360) * PI / 180);
            x = 450 + (scanMsg->ranges[i] * 100) * cos (scanMsg->angle_min + i * scanMsg->angle_increment);
            y = 450 + (scanMsg->ranges[i] * 100) * sin (scanMsg->angle_min + i * scanMsg->angle_increment);
            Point2d p (x, y);

            if (y > 450)
                pointsRight.push_back (p);
            else
                points.push_back (p);
        }
    }

    // Find best line (RANSAC)
    vector<Line> lines;

    for (int i = 0; i < points.size(); i++)
        cv::circle (img, points[i], 3, cv::Scalar (200, 200, 255), 2);

    for (int i = 0; i < pointsRight.size(); i++)
        cv::circle (img, pointsRight[i], 3, cv::Scalar (200, 200, 255), 2);

    cv::circle (img, Point2f (450, 450), 3, cv::Scalar (255, 255, 255), 2);
    float prePoints = points.size() + pointsRight.size();
    FindBestLineRemoveInliers (pointsRight, lines, ransacThreshold, 50);
    FindBestLineRemoveInliers (points, lines, ransacThreshold, 50);

    cout << "percent inliers: " << (prePoints - (points.size() + pointsRight.size())) / prePoints << endl;

    // Determine quality (parrallel, distance)
    if (lines.size() == 2 && (prePoints - (points.size() + pointsRight.size())) / prePoints > 0.75)
    {
        // Parallelity - dotproduct 1=parrallel 0=perpendicular
        double dotProduct = (1 * 1 + lines[0].a * lines[1].a) / (sqrt (1 + lines[0].a * lines[0].a) * sqrt (1 + lines[1].a * lines[1].a));
        Point2d robotPos (450, 450);
        double distanceBetweenLinesAtRobot = lines[1].distanceToPoint (robotPos) + lines[0].distanceToPoint (robotPos);
        double distanceFromOptimal = abs (distanceBetweenLinesAtRobot - ROW_WIDTH);
        distanceFromOptimal = distanceFromOptimal / (ROW_WIDTH / 3);
        double score = dotProduct;

        // Determine left and right line
        int left, right;

        if (lines[0].getY (450) > 450)
        {
            // 1 = right    0 = left
            right = 1;
            left = 0;
        }
        else
        {
            // 0 = right    1 = left
            right = 0;
            left = 1;
        }

        float distaneToCenterLine = (float) lines[right].distanceToPoint (robotPos) - (float) lines[left].distanceToPoint (robotPos);
        std_msgs::Float32 distanceMsg;
        distanceMsg.data = distaneToCenterLine;
        distanceOfCenter.publish (distanceMsg);
        // cout << "right: " << lines[right].distanceToPoint(robotPos) << "\tleft: " << lines[left].distanceToPoint(robotPos) << "    Off-center: " << distaneToCenterLine << endl;

        cout << "Distance bestween: " << distanceBetweenLinesAtRobot << "    Dot: " << dotProduct << "\tScore: " << score << "    Off-center: " << distaneToCenterLine << endl;

        float angleOnWallValue = atan2 ( (lines[0].a + lines[1].a) / 2, 1);
        std_msgs::Float32 angleMsg;
        angleMsg.data = angleOnWallValue * 180 / PI;
        angleOnWalls.publish (angleMsg);

        if (score >= minScoreForPublish)
        {
            // Publish the Odometry message
            nav_msgs::Odometry odoMsg;
            odoMsg.pose.pose.position.y = distaneToCenterLine / 100.0;
            tf::quaternionTFToMsg (tf::createQuaternionFromRPY (0, 0, angleOnWallValue), odoMsg.pose.pose.orientation);

            for (int i = 0; i < odoMsg.pose.covariance.size(); i++)
                odoMsg.pose.covariance[i] = 0.01;

            odoMsg.pose.covariance[0] = (float) covX;
            odoMsg.pose.covariance[35] = (float) covYaw;

            odoMsg.header.stamp = ros::Time::now();
            visualOdometry.publish (odoMsg);
        }
    }

    // Draw lines
    for (int i = 0; i < lines.size(); i++)
    {
        //lines[i].print();
        Point pt1, pt2;
        pt1.x = 1;
        pt1.y = lines[i].getY (pt1.x);
        pt2.x = 890;
        pt2.y = lines[i].getY (pt2.x);
        line (img, pt1, pt2, Scalar (125, 255, 255), 1, CV_AA);
    }

    newImage = true;

    if (newImage)
    {
        sensor_msgs::ImagePtr ptr = cv_bridge::CvImage (std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg();
        img_publisher.publish (ptr);
        newImage = false;
    }
}

// Laser scan call back
void laserScanCallBack (const sensor_msgs::LaserScanPtr& scanMsg)
{
    double temp = 0;

    if (!runOnce)
        getLines (scanMsg, temp);
}

int main (int argc, char** argv)
{
    srand (time (NULL));


    ros::init (argc, argv, "mr_bridgewalltracking");
    ros::NodeHandle n;

    // Set parameters
    n.param<double> ("/BridgeWallTracking/minScoreForPublishing", minScoreForPublish, 0.6);
    n.param<double> ("/BridgeWallTracking/covX", covX, 0.8);
    n.param<double> ("/BridgeWallTracking/covYaw", covYaw, 0.8);
    n.param<double> ("/BridgeWallTracking/ransacThreshold", ransacThreshold, 3);
    n.param<double> ("/BridgeWallTracking/laserMaxDistance", laserMaxDistance, 2);

    //Subscribe to topic
    ros::Subscriber sub = n.subscribe ("scan", 1000, laserScanCallBack);

    //Publish topics
    img_publisher = n.advertise<sensor_msgs::Image> ("wall_finding_image", 1000);
    angleOnWalls = n.advertise<std_msgs::Float32> ("angle_on_wall", 1);
    distanceOfCenter = n.advertise<std_msgs::Float32> ("distanceOfCenter", 1);
    visualOdometry = n.advertise<nav_msgs::Odometry> ("visualOdometry", 1);



    // Spin
    ros::Rate r (10);

    while (ros::ok())
    {

        ros::spinOnce();
        r.sleep();
        // cv::imshow("Lines",currentImg);
    }

    return 0;
}
