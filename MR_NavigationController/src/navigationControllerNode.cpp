// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "msgs/BoolStamped.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

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
ros::Publisher _motorCommandTopic, _deadmanTopic;
boost::mutex _motorCommandMutex;
double _angleSpeed = 0.0, _forwardSpeed = 0.0;
double _coeffP, _coeffI, _coeffD, _maxI;
double _maxAngleSpeed;
double _speedNormal, _speedSlow, _speedError;
int _motorUpdateRate;

// Function prototype
void setSpeed(double pidError = 0);

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

void setSpeed(double pidError)
{
    // TODO: Include PID error to speed here
    //returnSpeed pidError blabla

    if(_running)
    {
        switch(_systemMode)
        {
            case START:
                _forwardSpeed = _speedNormal;
                break;


            case ERROR:
                _forwardSpeed = _speedError;
                break;

            case SLOW:
                _forwardSpeed = _speedSlow;
                break;

            case STOP:
            default:
                _forwardSpeed = 0.0;
                break;
        }
    }
    else
        _forwardSpeed = 0.0;
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

    return deltaXTheta * DEGREETORAD;
}

void kalmanCallback(nav_msgs::Odometry msg)
{
    static double oldError = 0.0, integral = 0.0;

    if(_running)
    {
	// Get deltaX
        double deltaX = msg.pose.pose.position.x;

	// Get theta
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
        double deltaTheta = tf::getYaw(quat);

        // Calculate error
        double error = calculateError(deltaX, deltaTheta);

        // Reset integral part when error changes sign
        if(oldError*error < 0.0)
            integral = 0.0;

        // Sum integral part
        integral += error;

        // Reset if above max value
        if(integral > _maxI)
            integral = _maxI;
        else if(integral < -_maxI)
            integral = -_maxI;

        // Calculate derivative
        double derivative = error - oldError;

        // Calculate speed and correction for theta
        _motorCommandMutex.lock();
        _angleSpeed = _coeffP * error + _coeffI * integral + _coeffD * derivative;

        // Set limits
        if(_angleSpeed > _maxAngleSpeed)
            _angleSpeed = _maxAngleSpeed;
        else if(_angleSpeed < -_maxAngleSpeed)
            _angleSpeed = -_maxAngleSpeed;

        setSpeed();
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
    boolStamp.data = _running;

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
        sendMotorCommand(_forwardSpeed, _angleSpeed);
        _motorCommandMutex.unlock();

        // Sleep
        usleep(_motorUpdateRate); // Sleep for 50 ms = 20Hz
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

    // Get parameters
    nh.param<double>("/MR_NavigationController/NavigationController/speed_normal", _speedNormal, 0.6);
    nh.param<double>("/MR_NavigationController/NavigationController/speed_slow", _speedSlow, 0.2);
    nh.param<double>("/MR_NavigationController/NavigationController/speed_error", _speedError, 0.0);
    nh.param<double>("/MR_NavigationController/NavigationController/max_angle_speed", _maxAngleSpeed, 0.8);
    nh.param<double>("/MR_NavigationController/NavigationController/pid_coeff_p", _coeffP, 0.5);
    nh.param<double>("/MR_NavigationController/NavigationController/pid_coeff_i", _coeffI, 0.0001);
    nh.param<double>("/MR_NavigationController/NavigationController/pid_coeff_d", _coeffD, 0.01);
    nh.param<double>("/MR_NavigationController/NavigationController/pid_max_i", _maxI, 1000);
    nh.param<int>("/MR_NavigationController/NavigationController/motor_update_rate", _motorUpdateRate, 50); // Sleep for 50 ms = 20Hz

    // Get topic names
    std::string deadmanParameter, motorParameter, missionPlanParam, kalmanParam;
    nh.param<std::string>("/MR_NavigationController/NavigationController/deadman_pub", deadmanParameter, "/fmSafe/deadman");
    nh.param<std::string>("/MR_NavigationController/NavigationController/cmd_vel_pub", motorParameter, "/fmCommand/cmd_vel");
    nh.param<std::string>("/MR_NavigationController/NavigationController/mr_missionplan_sub", missionPlanParam, "/mrMissionPlanner/Status");
    nh.param<std::string>("/MR_NavigationController/NavigationController/mr_kalman_sub", kalmanParam, "/mrKalman/data");

    // Publisher
    _motorCommandTopic = nh.advertise<geometry_msgs::TwistStamped>(motorParameter, 1);
    _deadmanTopic = nh.advertise<msgs::BoolStamped>(deadmanParameter, 1);

    // Subscriber
    ros::Subscriber subMissionPlanner = nh.subscribe(missionPlanParam, 10, missionCallback);
    ros::Subscriber subKalman = nh.subscribe(kalmanParam, 10, kalmanCallback);

    // Start motor update thread
    boost::thread motorPublishThread(motorUpdateThread);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close thread
    motorPublishThread.interrupt();

    // Return
    return 0;
}
