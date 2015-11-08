#include "skills.h"

#include "std_msgs/String.h"

#define M_PI		3.14159265358979323846
#define DEG_TO_RAD	(M_PI/180.0)
#define RAD_TO_DEG	(180.0/M_PI)


Skills::Skills(ros::ServiceClient* srv_lineUntilQR, ros::ServiceClient* srv_move,
               ros::Publisher * pub_status
              )
{
    srv_lineUntilQR_ = srv_lineUntilQR;
    srv_move_ = srv_move;
    pub_status_ = pub_status;
}

Skills::~Skills()
{
    // Nothing
}

bool Skills::lineUntilQR(std::string qr)
{
    std::cout << "Skill: Line until QR: " << qr << std::endl;
    lineFollowerCall.request.qr = qr;
    lineFollowerCall.request.time_limit = 30;
    srv_lineUntilQR_->call(lineFollowerCall);

    //TODO Time limit + achivement checking

	std_msgs::String msg;
	msg.data = "Following line until " + qr;
	pub_status_->publish(msg);
    return true;
}

bool Skills::linearMove(double distance)
{
    std::cout << "Skill: Linear move for: " << distance << " m"<< std::endl;
    move_call_.request.linear = distance;
    move_call_.request.angular = 0;
    srv_move_->call(move_call_);

    //TODO Time limit + achivement checking

	std_msgs::String msg;
	msg.data = "Linear movement of " + std::to_string(distance);
    pub_status_->publish(msg);
    return true;
}

bool Skills::angularMove(double angle)
{
    std::cout << "Skill: Angular move for: " << angle << " degrees"<< std::endl;
    move_call_.request.linear = 0;
    move_call_.request.angular = angle;
    srv_move_->call(move_call_);

    //TODO Time limit + achivement checking

	std_msgs::String msg;
	msg.data = "Linear movement of " + std::to_string(angle);
	pub_status_->publish(msg);
    return move_call_.response.done;
}

bool Skills::goToFreePosition(double x, double y)
{

	std_msgs::String msg;
	msg.data = "Free navigation to X:" + std::to_string(x) + ", Y:" +  std::to_string(y);
	pub_status_->publish(msg);
    return true;
}



