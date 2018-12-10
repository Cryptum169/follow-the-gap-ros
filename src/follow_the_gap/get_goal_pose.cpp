#include <ros/ros.h>
#include <ros/rate.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ROS_INFO_STREAM("Goal node initiated");
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Twist>("goal_state", 10);
    geometry_msgs::Twist goal;
    // make arbitrary goal position at (6,-3)
    goal.linear.x = 6;
    goal.linear.y = 6;
    while (ros::ok()){
        goal_pub.publish(goal);
        rate.sleep();
    }
    
}