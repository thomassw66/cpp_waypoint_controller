#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include "boost/thread/thread.hpp"
#include <math.h>
#include <stdlib.h>

class WaypointController 
{
public:
    WaypointController(std::string turtle_name);

private:
    std::string turtle_name;
    
    enum state {
        LISTENING_FOR_WAYPOINT,
        NOT_FACING_WAYPOINT,
        MOVING_TOWARDS_WAYPOINT
    };

    state my_state;                     // finite automata state 

    bool pose_ready;
    bool waypoint_ready;
    double pose_x, pose_y, pose_theta;  // internal pose state
    double x, y;                        // goal waypoint

    double linear_vel;
    double angular_vel;

    ros::NodeHandle nh;                 // node handle 
    ros::NodeHandle ph;                 // publishing handle

    ros::Publisher  cmd_vel_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber waypoint_sub;

    ros::Timer timer;

    boost::mutex pose_mutex, waypoint_mutex;

 //   void pose_callback(const turtlesim::PoseConstPtr& msg);
    void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void waypoint_update_callback(const geometry_msgs::Twist::ConstPtr & next_waypoint);
    void publish_cmd_vel();
    void run();
};

WaypointController::WaypointController(std::string turtle_name): 
    my_state(LISTENING_FOR_WAYPOINT), 
    pose_ready(false),
    linear_vel(0.0), 
    angular_vel(0.0), 
    nh(), 
    ph() 
{

    cmd_vel_pub = ph.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 1000);
    // pose_sub = nh.subscribe(turtle_name+"/pose", 10, &WaypointController::pose_callback, this);
    waypoint_sub = nh.subscribe(turtle_name + "/waypoint", 10, &WaypointController::waypoint_update_callback, this);
    pose_sub = nh.subscribe(turtle_name + "/odom", 10, &WaypointController::pose_callback, this);

    timer = nh.createTimer(ros::Duration(0.1), boost::bind(&WaypointController::run, this));
}

void WaypointController::run()
{
    if (! pose_ready ) return;
    if (! waypoint_ready ) return;
    boost::mutex::scoped_lock(pose_mutex);
    boost::mutex::scoped_lock(waypoint_mutex);
    // where is waypoint 
    double dx = x - pose_x;
    double dy = y - pose_y;
    double dist = sqrt(dx*dx + dy*dy);
    double theta = atan2(dy,dx);
    if (theta < 0.0) theta += 2 * M_PI;
    double diff_angle = theta - pose_theta; 
    if (diff_angle < 0) diff_angle = -diff_angle;
    if (2.0*M_PI - diff_angle < diff_angle) diff_angle = 2.0*M_PI - diff_angle;
    ROS_INFO("pose_theta: %f theta: %f diff_angle: %f M_PI: %f", pose_theta, theta, diff_angle, M_PI);
            
    if (diff_angle > 0.03) { // check if we are facing our waypoint 
        double vel_magnitude = diff_angle;
        double vel_direction = (pose_theta > theta) ? 
                            ((pose_theta - theta > theta - pose_theta + 2*M_PI) ? 1: -1) : 
                            ((theta - pose_theta > pose_theta - theta + 2*M_PI) ? -1: 1) ;

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = vel_direction * vel_magnitude;
        cmd_vel_pub.publish(cmd_vel_msg);                
    } else if (dist > 0.3) { // check to see if we are not within 0.3 of our waypoint 
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = dist;
        cmd_vel_pub.publish(cmd_vel_msg);
    } else {
        cmd_vel_pub.publish(*new geometry_msgs::Twist());
        waypoint_ready = false;
    }
}

void WaypointController::pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
// void WaypointController::pose_callback(const turtlesim::PoseConstPtr& msg)
{
    // ROS_INFO("my x: %f my y: %f \t theta: %f", msg->x, msg->y, msg->theta);
    // boost::mutex::scoped_lock(pose_mutex);
    // pose_x = msg->x;    
    // pose_y = msg->y;
    // pose_theta = msg->theta;
    // if (pose_theta < 0.0) pose_theta += 2.0* M_PI; 
    // pose_ready = true;
    boost::mutex::scoped_lock(pose_mutex);
    pose_x = odom_msg->pose.pose.position.x;
    pose_y = odom_msg->pose.pose.position.y;
    geometry_msgs::Quaternion q = odom_msg->pose.pose.orientation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    pose_theta = atan2(siny, cosy);
    // convert to radians???
    if (pose_theta < 0.0) pose_theta += 2.0 * M_PI;
    pose_ready = true;
    ROS_INFO("pose_x: %f, pose_y: %f, pose_theta: %f", pose_x, pose_y, pose_theta);
}

void WaypointController::waypoint_update_callback(const geometry_msgs::Twist::ConstPtr & next_waypoint)
{
    boost::mutex::scoped_lock(waypoint_mutex);
    x = next_waypoint->linear.x;
    y = next_waypoint->linear.y;
    waypoint_ready = true;
}

int main(int argc, char ** argv) 
{ 
    ros::init(argc, argv, "waypoint_node");
    WaypointController control("");
    
    ros::spin();   
}
