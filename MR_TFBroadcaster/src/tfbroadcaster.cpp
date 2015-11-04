#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_tf");
    ros::NodeHandle node, pn("~");

    std::string lidarFrame, vehicleFrame;
    pn.param<std::string>("lidar_frame", lidarFrame, "lidar_frame");
    pn.param<std::string>("vehicle_frame", vehicleFrame, "base_footprint");

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);

    while (node.ok())
    {
        transform.setOrigin( tf::Vector3(0.2, 0.0, 0.0) );
        transform.setRotation( tf::createQuaternionFromRPY(0,0,0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), vehicleFrame, lidarFrame));
        rate.sleep();
    }

    return 0;
}
