#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include "boost/thread/thread.hpp"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class PoseNode 
{
public:
	PoseNode();
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	// ros::Publisher pub;
	void run();
	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
};

PoseNode::PoseNode()
{
	ROS_INFO("hello3");
	sub = nh.subscribe("odom", 10, &PoseNode::odom_callback, this);

}

void PoseNode::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) 
{
	ROS_INFO("hello");
	ROS_INFO("odom: %f", odom_msg->pose.pose.position.x);
	ROS_INFO("    : %f", odom_msg->pose.pose.position.y);
	ROS_INFO("    : %f", odom_msg->pose.pose.orientation.z);
	ROS_INFO("    : %f", odom_msg->pose.pose.orientation.w);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pose_node");
	PoseNode p;
	ros::spin();
}
