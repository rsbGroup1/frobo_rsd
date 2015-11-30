#ifndef SKILLS_H
#define SKILLS_H

#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include "mr_line_follower/followUntilQR.h"
#include "mr_line_follower/followUntilLidar.h"
#include "mr_line_follower/followUntilRelative.h"
#include "mr_obstacle_detector/enabler.h"
#include "mr_go/move.h"

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

class Skills
{
public:
    /**
     *
     */
    Skills (ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move, ros::ServiceClient* srv_lineUntilLidar, ros::ServiceClient* srv_lineUntilRelative,
			ros::Publisher* pub_status, ros::Publisher* pub_initialize , ros::Publisher* pub_deadman,
			ros::ServiceClient* srv_detect_obstacles);

    /**
     *
     */
    ~Skills();

    /**
     * Follows the line with the camera until it finds the specified qr
     * @param qr the qr code to find
     */
    bool lineUntilQR (std::string qr);

    /**
     * Follows the line with the camera until the frontal lidar distance is less than 
     * @param distance in 20 frontal lidar values
     */
    bool lineUntilLidar (double distance);

    /**
     * Follows the line with the camera until the frontal lidar distance is less than 
     * @param distance in 20 frontal lidar values
     */
    bool lineUntilRelative (double distance);

    /**
     * Moves the robot for an specified distance
     * @param distance the distance to move straight. It can be positive or negative
     */
    bool linearMove (double distance);

    /**
     * Turns a defined angle
     * @param angle The angle to turn. NEED to be in radians
     */
    bool angularMove (double angle);


    /**
     *
     */
    bool goToFreePosition (double x, double y, double yaw);

    /**
     *
     */
    bool setInitialPoseAMCL (double x, double y, double yaw);
    
    /**
     * Activates the obstacle detection
     */
    bool detectObstacles(bool state);

    /**
     * checks battery level, if < @param threshold, we determine the robot is not charging
     * and execute the backup behaviour
     */
    bool chargeDectectionAndBackupPlan(double* battery_level, double threshold);

    /**
     * waits for @param seconds
     */
    bool wait(double seconds);
    

private:
    ros::ServiceClient* srv_lineUntilQR_;
    ros::ServiceClient* srv_lineUntilLidar_;
    ros::ServiceClient* srv_lineUntilRelative_;
    ros::ServiceClient* srv_move_;
    ros::ServiceClient* srv_detect_obstacles_;
    ros::Publisher* pub_status_;

    ros::Publisher* pub_initialize_;
    ros::Publisher* pub_deadman_;

    // Threads
    boost::thread* deadmanThread_;

    mr_go::move move_call_;
    mr_line_follower::followUntilQR lineFollowerCall;
    mr_line_follower::followUntilLidar followUntilLidarCall;
    mr_line_follower::followUntilRelative followUntilRelativeCall;
	mr_obstacle_detector::enabler detectObstaclesCall;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_actionclient_;
    int goal_id_;

    void enableDeadman();
};

#endif // SKILLS_H
