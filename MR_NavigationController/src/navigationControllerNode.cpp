// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "msgs/BoolStamped.h"

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

#define FUNC_B_TRANSROT         225 // y=225*x when mapping from translation to rotation
#define INTERVAL_TRANS          0.4 // +- in meters
#define INTERVAL_ROT            90  // +- in degrees
#define SPEED_NORMAL            0.5 // m/s
#define SPEED_SLOW              0.1
#define SPEED_STOP              0.0

// PID
#define COEFF_P                 0.1
#define COEFF_I                 0.00001
#define COEEF_D                 0.001
#define MAX_I                   1000


// System mode enum
enum MODES
{
    SLOW = 0,
    ERROR,
    STOP,
    START
};

// Global var
bool _running = false;
MODES _systemMode = STOP;
double _speed = SPEED_STOP;
ros::Publisher _motorCommandTopic, _deadmanTopic;
boost::mutex _motorCommandMutex;
double _angleSpeed = 0.0;

// Function prototype
double calculateSpeed(double pidError = 0);

void missionCallback(std_msgs::String msg)
{
    if(msg.data == "start")
    {
        _systemMode = START;
        _running = true;
    }
    else if(msg.data == "stop")
    {
        _systemMode = STOP;
        _running = false;
    }
    else if(msg.data == "slowDown")
    {
        _systemMode = SLOW;

    }
    else if(msg.data == "errorStop")
    {
        _systemMode = ERROR;
    }
}

double calculateSpeed(double pidError)
{
    double returnSpeed = 0;

    if(_running)
    {
        switch(_systemMode)
        {
            case START:
                returnSpeed = SPEED_NORMAL;
                break;

            case STOP:
                returnSpeed = SPEED_STOP;
                break;

            case ERROR:
                returnSpeed = SPEED_STOP;
                break;

            case SLOW:
                returnSpeed = SPEED_SLOW;
                break;

            default:
                returnSpeed = SPEED_STOP;
                break;
        }
    }
    else
        return SPEED_STOP;

    // Include PID error
    //returnSpeed pidError blabla

    return returnSpeed;
}

double calculateError(double deltaX, double deltaTheta)
{
    // We transform the error for the translation into an rotational error
    double deltaXTheta = 0;

    // 4 Cases
    if(deltaX>=0 && deltaTheta>=0)
        deltaXTheta = 90-fabs(deltaTheta);
    else if(deltaX>=0 && deltaTheta<0)
        deltaXTheta = 90+fabs(deltaTheta);
    else if(deltaX<0 && deltaTheta<0)
        deltaXTheta = fabs(deltaTheta) - 90;
    else if(deltaX<0 && deltaTheta>=0)
        deltaXTheta = -(90+fabs(deltaTheta));

    // Now we got an error in angle domain with respect to both translation and rotation
    // Weight these two based on the size of the translational error

    // Use this function
    // y = 1/(20*x)
    double weight = 1.0/(20.0 * fabs(deltaX));

    if(weight>1.0)
        weight = 1;

    // Return weight
    deltaXTheta = (weight*(-deltaTheta) + (1.0-weight)*deltaXTheta);

    // Use the shortest angle
    if(fabs(deltaXTheta) > 180.0)
        deltaXTheta = ((deltaX >= 0)?(-1):(1)) * 360.0 + deltaXTheta;

    return deltaXTheta;
}

void kalmanCallback(geometry_msgs::PoseWithCovarianceStamped pose)
{
    static double oldError = 0.0, integral = 0.0;

    if(_running)
    {
        double deltaX = pose.pose.pose.position.x;
        double deltaTheta = pose.pose.pose.orientation.x;

        // Calculate error
        double error = calculateError(deltaX, deltaTheta);

        // Reset integral part when error changes sign
        if(oldError*error < 0.0)
            integral = 0.0;

        // Sum integral part
        integral += error;

        // Reset if above max value
        if(integral > MAX_I)
            integral = MAX_I;
        else if(integral < -MAX_I)
            integral = -MAX_I;

        // Calculate derivative
        double derivative = error - oldError;

        // Calculate speed and correction for theta
        _motorCommandMutex.lock();
        _angleSpeed = COEFF_P * error + COEFF_I * integral + COEEF_D * derivative;
        _speed = calculateSpeed();
        _motorCommandMutex.unlock();

        // Store new as old
        oldError = error;
    }
    else
    {
        oldError = integral = 0.0;
    }
}

void sendMotorCommand(double speed, double theta)
{
    // Create deadman stuff
    msgs::BoolStamped boolStamp;
    boolStamp.header.stamp = ros::Time::now();
    boolStamp.data = true;

    // Create motor stuff
    geometry_msgs::TwistStamped twistStamp;
    twistStamp.header.stamp = ros::Time::now();
    twistStamp.twist.linear.x = speed;
    twistStamp.twist.angular.z = theta;

    // Send commands
    _deadmanTopic.publish(boolStamp);
    _motorCommandTopic.publish(twistStamp);
}

void motorUpdateThread()
{
    while(true)
    {
        // Send motor command
        _motorCommandMutex.lock();
        sendMotorCommand(_speed, _angleSpeed);
        _motorCommandMutex.unlock();

        // Sleep
        usleep(50); // Sleep for 50 ms = 20Hz
    }
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_NavigationController_Node");
    ros::NodeHandle nh;

    std::string deadmanParameter, motorParameter;
    nh.param<std::string>("deadman_pub", deadmanParameter, "/fmSafe/deadman");
    nh.param<std::string>("cmd_vel_pub", motorParameter, "/fmCommand/cmd_vel");

    _motorCommandTopic = nh.advertise<geometry_msgs::TwistStamped>(motorParameter, 1);
    _deadmanTopic = nh.advertise<msgs::BoolStamped>(deadmanParameter, 1);

    // Subscriber
    ros::Subscriber subMissionPlanner = nh.subscribe("missionPlannerTopic", 10, missionCallback);
    ros::Subscriber subKalman = nh.subscribe("kalmanTopic", 10, kalmanCallback);

    // Start motor update thread
    boost::thread motorPublishThread(motorUpdateThread);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Return
    return 0;
}
