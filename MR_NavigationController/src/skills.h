#ifndef SKILLS_H
#define SKILLS_H

#include <ros/ros.h>
#include "mr_line_follower/followUntilQR.h"
#include "mr_go/move.h"

class Skills
{
public:
	/**
	 * 
	 */
	Skills(ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move);
	
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
    bool goToFreePosition(double x, double y);

    /**
     *
     */
    bool moveToDispenser();

    /**
     *
     */
    bool moveToCharger();

    /**
     *
     */
    bool moveFromDispenser();

    /**
     *
     */
    bool moveFromCharger();

    /**
     *
     */
    bool changeLineWC1();

    /**
     *
     */
    bool changeLineWC2();

    /**
     *
     */
    bool changeLineWC3();

private:
	ros::ServiceClient* srv_lineUntilQR_;
	ros::ServiceClient* srv_move_;
	mr_go::move move_call_;
	mr_line_follower::followUntilQR lineFollowerCall;
};

#endif // SKILLS_H