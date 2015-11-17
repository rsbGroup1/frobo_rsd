// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "msgs/BoolStamped.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// System mode enum
enum MODES
{
    M_RUN = 0,
    M_IDLE,
    M_MANUAL,
    M_NORMAL,
    M_SLOW,
    M_STOP
};

// Global var
bool _running = false;
MODES _systemMode = M_IDLE;
ros::Publisher _motorCommandTopic, _deadmanTopic, _deltaXTopic, _deltaThetaTopic;
boost::mutex _runningMutex;
double _angleSpeed = 0.0, _forwardSpeed = 0.0;
double _coeffP, _coeffI, _coeffD, _maxI;
double _maxAngleSpeed;
double _speedNormal, _speedSlow, _speedStop;
int _motorUpdateRate;
double _funcDen, _funcNom;
boost::thread* _motorPublishThread;

// Function prototype
void setSpeed (double pidError = 0);

void missionCallback (std_msgs::String msg)
{
    _runningMutex.lock();

    if (msg.data == "run")
    {
        _systemMode = M_RUN;
        _running = true;
    }
    else if (msg.data == "runStop")
    {
        _systemMode = M_STOP;
        _running = false;
    }
    else if (msg.data == "runSlow")
    {
        _systemMode = M_SLOW;
        _running = true;
    }
    else if (msg.data == "idle")
    {
        _systemMode = M_IDLE;
        _running = false;
    }
    else if (msg.data == "manual")
    {
        _systemMode = M_MANUAL;
        _running = false;
    }

    _runningMutex.unlock();
}

void setSpeed (double pidError)
{
    // TODO: Include PID error to speed here
    //returnSpeed pidError blabla

    _runningMutex.lock();

    if (_running)
    {
        _runningMutex.unlock();

        switch (_systemMode)
        {
        case M_RUN:
            _forwardSpeed = _speedNormal;
            break;

        case M_STOP:
            _forwardSpeed = _speedStop;
            _angleSpeed = 0.0;
            break;

        case M_SLOW:
            _forwardSpeed = _speedSlow;
            break;

        case M_MANUAL:
        case M_IDLE:
        default:
            _forwardSpeed = 0.0;
            break;
        }
    }
    else
    {
        _runningMutex.unlock();
        _forwardSpeed = 0.0;
    }
}

double calculateError (double deltaX, double deltaTheta)
{
    // We transform the error for the translation into an rotational error
    double deltaXTheta = 0;

    // 4 Cases
    if (deltaX >= 0 && deltaTheta >= 0)
        deltaXTheta = 90 - fabs (deltaTheta);
    else if (deltaX >= 0 && deltaTheta < 0)
        deltaXTheta = 90 + fabs (deltaTheta);
    else if (deltaX < 0 && deltaTheta < 0)
        deltaXTheta = fabs (deltaTheta) - 90;
    else if (deltaX < 0 && deltaTheta >= 0)
        deltaXTheta = - (90 + fabs (deltaTheta));

    // Now we got an error in angle domain with respect to both translation and rotation
    // Weight these two based on the size of the translational error

    // Use a weighting function
    double weight = _funcNom / (_funcDen * fabs (deltaX));

    if (weight > 1.0)
        weight = 1;

    // Return weight
    deltaXTheta = (weight * (-deltaTheta) + (1.0 - weight) * deltaXTheta);

    // Use the shortest angle
    if (fabs (deltaXTheta) > 180.0)
        deltaXTheta = ( (deltaX >= 0) ? (-1) : (1)) * 360.0 + deltaXTheta;

    return deltaXTheta * DEGREETORAD;
}

void kalmanCallback (geometry_msgs::PoseWithCovarianceStamped msg)
{
    static double oldError = 0.0, integral = 0.0;

    // Get deltaX
    double deltaX = msg.pose.pose.position.y;

    // Get theta
    tf::Quaternion quat;
    tf::quaternionMsgToTF (msg.pose.pose.orientation, quat);
    double deltaTheta = tf::getYaw (quat) * RADTODEGREE;

    // Publish received values
    std_msgs::Float64 msgF;
    msgF.data = deltaX;
    _deltaXTopic.publish (msgF);
    msgF.data = deltaTheta;
    _deltaThetaTopic.publish (msgF);

    _runningMutex.lock();

    if (_running)
    {
        _runningMutex.unlock();

        // Calculate error
        double error = calculateError (deltaX, deltaTheta);

        // Reset integral part when error changes sign
        if (oldError * error < 0.0)
            integral = 0.0;

        // Sum integral part
        integral += error;

        // Reset if above max value
        if (integral > _maxI)
            integral = _maxI;
        else if (integral < -_maxI)
            integral = -_maxI;

        // Calculate derivative
        double derivative = error - oldError;

        // Calculate speed and correction for theta
        _angleSpeed = _coeffP * error + _coeffI * integral + _coeffD * derivative;

        // Set limits
        if (_angleSpeed > _maxAngleSpeed)
            _angleSpeed = _maxAngleSpeed;
        else if (_angleSpeed < -_maxAngleSpeed)
            _angleSpeed = -_maxAngleSpeed;

        // Store new as old
        oldError = error;
    }
    else
    {
        _runningMutex.unlock();
        oldError = integral = 0.0;
    }
}

void sendMotorCommand (double speed, double theta)
{
    // Create deadman stuff
    msgs::BoolStamped boolStamp;
    boolStamp.header.stamp = ros::Time::now();

    _runningMutex.lock();
    boolStamp.data = _running;
    _runningMutex.unlock();

    // Create motor stuff
    geometry_msgs::TwistStamped twistStamp;
    twistStamp.header.stamp = ros::Time::now();
    twistStamp.twist.linear.x = speed;
    twistStamp.twist.angular.z = theta;

    // Send commands
    _deadmanTopic.publish (boolStamp);
    _motorCommandTopic.publish (twistStamp);
}

void motorUpdateThread()
{
    while (true)
    {
        try
        {
            _runningMutex.lock();

            if (_running)
            {
                _runningMutex.unlock();

                // Send motor command
                setSpeed();
                sendMotorCommand (_forwardSpeed, _angleSpeed);
            }
            else
                _runningMutex.unlock();

            // Sleep
            usleep (_motorUpdateRate); // Sleep for 50 ms = 20Hz

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch (const boost::thread_interrupted&)
        {
            break;
        }
    }
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init (argc, argv, "RSD_NavigationController_Node");
    ros::NodeHandle nh, pNh ("~");

    // Get parameters
    pNh.param<double> ("speed_normal", _speedNormal, 0.6);
    pNh.param<double> ("speed_slow", _speedSlow, 0.2);
    pNh.param<double> ("speed_error", _speedStop, 0.0);
    pNh.param<double> ("max_angle_speed", _maxAngleSpeed, 0.8);
    pNh.param<double> ("pid_coeff_p", _coeffP, 0.5);
    pNh.param<double> ("pid_coeff_i", _coeffI, 0.0001);
    pNh.param<double> ("pid_coeff_d", _coeffD, 0.01);
    pNh.param<double> ("pid_max_i", _maxI, 1000);
    pNh.param<int> ("motor_update_rate", _motorUpdateRate, 50); // Sleep for 50 ms = 20Hz
    pNh.param<double> ("func_nominator", _funcNom, 1.0);
    pNh.param<double> ("func_denominator", _funcDen, 40.0);

    // Get topic names
    std::string deadmanParameter, motorParameter, missionPlanParam, kalmanParam, deltaXParam, deltaThetaParam;
    pNh.param<std::string> ("deadman_pub", deadmanParameter, "/fmSafe/deadman");
    pNh.param<std::string> ("cmd_vel_pub", motorParameter, "/fmCommand/cmd_vel");
    pNh.param<std::string> ("mr_missionplan_sub", missionPlanParam, "/mrMissionPlanner/status");
    pNh.param<std::string> ("mr_kalman_sub", kalmanParam, "/mrKalman/data");
    pNh.param<std::string> ("deltaX_pub", deltaXParam, "/mrNavigationController/deltaX");
    pNh.param<std::string> ("deltaTheta_pub", deltaThetaParam, "mrNavigationController/deltaTheta");

    // Publisher
    _motorCommandTopic = nh.advertise<geometry_msgs::TwistStamped> (motorParameter, 1);
    _deadmanTopic = nh.advertise<msgs::BoolStamped> (deadmanParameter, 1);
    _deltaXTopic = nh.advertise<std_msgs::Float64> (deltaXParam, 1);
    _deltaThetaTopic = nh.advertise<std_msgs::Float64> (deltaThetaParam, 1);

    // Subscriber
    ros::Subscriber subMissionPlanner = nh.subscribe (missionPlanParam, 1, missionCallback);
    ros::Subscriber subKalman = nh.subscribe (kalmanParam, 1, kalmanCallback);

    // Start motor update thread
    _motorPublishThread = new boost::thread (motorUpdateThread);

    // ROS Spin: Handle callbacks
    while (ros::ok)
        ros::spinOnce();

    // Close thread
    _motorPublishThread->interrupt();

    // Return
    return 0;
}
