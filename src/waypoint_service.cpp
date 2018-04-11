#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("waypoint", 1000);
    ros::Rate loop_rate(1000);
    float x, y;
    geometry_msgs::Twist way;
    while (ros::ok()) {
        scanf("%f %f", &x, &y);
        ROS_INFO("SENDING_WAYPOINT: %f %f", x, y);
        way.linear.x = x;
        way.linear.y = y;
        way.linear.z = 0;
        pub.publish(way);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}