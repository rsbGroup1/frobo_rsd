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

    // action client for move_base
    move_base_actionclient_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true); //actionlib::ActionClient<move_base_msgs::MoveBaseActionGoal>("move_base/goal");
    goal_id_ = 0;
}

Skills::~Skills()
{
    delete move_base_actionclient_;
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

bool Skills::goToFreePosition(double x, double y, double yaw)
{
    std::cout << "go to free position called" << std::endl;
    ROS_INFO("Go to free position called - goal(%f, %f, %f)",x,y,yaw);
    bool success = false;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    goal.target_pose.header.frame_id ="map";
    goal.target_pose.header.stamp = ros::Time::now();


    if(move_base_actionclient_->waitForServer(ros::Duration(5,0)))
    {

        move_base_actionclient_->sendGoal(goal);
        bool finished = move_base_actionclient_->waitForResult();
/* ORIGINAL
        if(move_base_actionclient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            success = true;
            std_msgs::String msg;
            msg.data = "Free navigation to X:" + std::to_string(x) + ", Y:" +  std::to_string(y);
            pub_status_->publish(msg);
        }
        else
        {
            ROS_WARN("Free navigation was unable to achieve goal(%f, %f, %f)",x,y,yaw);
	    //DSW TEsting
	    move_base_msgs::MoveBaseGoal recovery;
	    recovery.target_pose.pose.position.x = -0.30;
	    recovery.target_pose.pose.position.y = -2.5;
	    recovery.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.6);
	    recovery.target_pose.header.frame_id ="map";
	    recovery.target_pose.header.stamp = ros::Time::now();
	    move_base_actionclient_->sendGoal(recovery);
	    move_base_actionclient_->sendGoal(goal);
        }
*/
	// DSW TEsting: move to recovery position then try again
	    move_base_msgs::MoveBaseGoal recovery;
	    recovery.target_pose.pose.position.x = -0.1;
	    recovery.target_pose.pose.position.y = -0.15;
	    recovery.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1.9);
	    recovery.target_pose.header.frame_id ="map";
	    recovery.target_pose.header.stamp = ros::Time::now();
        while(move_base_actionclient_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
		ROS_WARN("To recovery=>goal");
		move_base_actionclient_->sendGoal(recovery);
		finished = move_base_actionclient_->waitForResult();
		move_base_actionclient_->sendGoal(goal);
		finished = move_base_actionclient_->waitForResult();			
        }

            success = true;
            std_msgs::String msg;
            msg.data = "Free navigation to X:" + std::to_string(x) + ", Y:" +  std::to_string(y);
            pub_status_->publish(msg);

    }
    else
    {
        ROS_ERROR("move_base action server not responding within timeout");
    }




    goal_id_++;
    return success;
}



