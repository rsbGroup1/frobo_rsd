#include "skills.h"


#define M_PI		3.14159265358979323846
#define DEG_TO_RAD	(M_PI/180.0)
#define RAD_TO_DEG	(180.0/M_PI)


Skills::Skills(ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move)
{
	srv_lineUntilQR_ = srv_lineUntilQR;
	srv_move_ = srv_move;
}

Skills::~Skills()
{
	// Nothing
}

bool Skills::lineUntilQR(std::string qr)
{
	std::cout << "Skill: Line until QR: " << qr << std::endl;
	//lineFollowerCall.request.qr = qr;
	//lineFollowerCall.request.time_limit = 30;
	//srv_lineUntilQR_->call(lineFollowerCall);
	return true;
}

bool Skills::linearMove(double distance)
{
	std::cout << "Skill: Linear move for: " << distance << " m"<< std::endl;
	//move_call_.request.linear = distance;
	//move_call_.request.angular = 0;
	//srv_move_->call(move_call_);
	//return move_call_.response.done;
	return true;
}

bool Skills::angularMove(double angle)
{
	std::cout << "Skill: Angular move for: " << angle << " degrees"<< std::endl;
	move_call_.request.linear = 0;
	move_call_.request.angular = angle;
	srv_move_->call(move_call_);
	return move_call_.response.done;
}

bool Skills::changeLineWC1()
{
	std::cout << "Skill: Change to workcell 1 robot line"<< std::endl;
	return true;
}

bool Skills::changeLineWC2()
{
	std::cout << "Skill: Change to workcell 2 robot line"<< std::cout;
	return true;
}

bool Skills::changeLineWC3()
{
	return true;
}

bool Skills::goToFreePosition(double x, double y)
{
	return true;
}

bool Skills::moveFromCharger()
{
	return true;
}

bool Skills::moveFromDispenser()
{
	return true;
}

bool Skills::moveToCharger()
{
	return true;
}

bool Skills::moveToDispenser()
{
	return true;
}

