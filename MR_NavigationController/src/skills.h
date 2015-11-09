#ifndef SKILLS_H
#define SKILLS_H

#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include "mr_line_follower/followUntilQR.h"
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
	Skills(ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move, 
		   ros::Publisher * pub_status);
	
	/**
	 * 
	 */
	~Skills();
	
    /**
     * Follows the line with the camera until it finds the specified qr
     * @param qr the qr code to find
     */
    bool lineUntilQR(std::string qr);

    /**
     * Moves the robot for an specified distance
     * @param distance the distance to move straight. It can be positive or negative
     */
    bool linearMove(double distance);

    /**
     * Turns a defined angle
     * @param angle The angle to turn. NEED to be in radians
     */
    bool angularMove(double angle);


    /**
     *
     */
    bool goToFreePosition(double x, double y, double yaw);

private:
	ros::ServiceClient* srv_lineUntilQR_;
	ros::ServiceClient* srv_move_;
	ros::Publisher* pub_status_;

	mr_go::move move_call_;
	mr_line_follower::followUntilQR lineFollowerCall;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_actionclient_;
    int goal_id_;
};

#endif // SKILLS_H
