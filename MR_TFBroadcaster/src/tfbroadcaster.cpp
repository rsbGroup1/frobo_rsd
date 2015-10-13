#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string turtle_name;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
/*  tf::Transform transform;
  transform.setOrigin( tf::Vector3(1.0, 2.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 3.0);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
*/


  geometry_msgs::TransformStamped lidar_trans;
  geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
  //const geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  double tmp = tf::getYaw(odom_quat);
  odom_quat = tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0);

  lidar_trans.header.stamp = ros::Time::now();
  lidar_trans.header.frame_id = "lidar_frame";
  lidar_trans.child_frame_id = "odom_combined";
  lidar_trans.transform.translation.x = -0.13;//msg->pose.pose.position.x;
  lidar_trans.transform.translation.y = 0.0;//msg->pose.pose.position.y;
  lidar_trans.transform.translation.z = 0.14;
  lidar_trans.transform.rotation = odom_quat;

  br.sendTransform(lidar_trans);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_tf_broadcaster");
  turtle_name = "test";
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/fmKnowledge/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
