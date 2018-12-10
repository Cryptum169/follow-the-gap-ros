#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/ColorRGBA.h>
#include <stdlib.h>
#include <vector>
#include <tf/transform_datatypes.h>

geometry_msgs::Twist rbt_pose;
geometry_msgs::Twist gap_pose;
geometry_msgs::Twist goal_pose;
ros::Publisher cmd_vel;
ros::Publisher marker_pub;

float max_angular_abs = 1;
float max_linear_abs = 1;
int once = 0;
float init_x = 0.0f;
float init_y = 0.0f;
float init_theta = 0.0f;

void gapMsgCallback(const geometry_msgs::Twist);
void goalMsgCallback(const geometry_msgs::Twist);
void rbtMsgCallback(const nav_msgs::Odometry);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_gap_execute");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Execution Node Initiated");
    ros::Subscriber gap_sub = nh.subscribe("/gap_dir", 100, &gapMsgCallback);
    ros::Subscriber goal_sub = nh.subscribe("/goal_state", 100, &goalMsgCallback);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 100, &rbtMsgCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_dir", 100);
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 50);
    // cmd_vel = nh.advertise<geometry_msgs::Twist>("info_debug", 50);
    ros::spin();
    return 0;
}

double quaternionToEuler(const nav_msgs::Odometry msg) {
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void gapMsgCallback(const geometry_msgs::Twist msg) {
    gap_pose = msg;
}

void goalMsgCallback(const geometry_msgs::Twist msg)
{
    goal_pose = msg;
}

void rbtMsgCallback(const nav_msgs::Odometry msg)
{
    if (once == 0) {
        init_x = msg.pose.pose.position.x;
        init_y = msg.pose.pose.position.y;
        init_theta = quaternionToEuler(msg);
        once = 1;
    }

    float curr_heading = quaternionToEuler(msg) - init_theta;

    float x_err = goal_pose.linear.x - (msg.pose.pose.position.x - init_x);
    float y_err = goal_pose.linear.y - (msg.pose.pose.position.y - init_y);
    ROS_INFO_STREAM("X err: " << x_err << "; Y err: " << y_err);
    float dist_to_goal = sqrt(x_err * x_err + y_err * y_err);
    ROS_INFO_STREAM("Distance to goal" << dist_to_goal);
    // spatial frame angle
    float goal_dir = atan2(y_err, x_err);

    float gap_dir = gap_pose.angular.z + curr_heading;
    float min_gap_dist = gap_pose.linear.x;
    float alpha = 1.0f;

    ROS_INFO_STREAM("Gap angle: " << gap_dir);

    if (min_gap_dist <= 0.05) {
        min_gap_dist = 0.5;
    }
    float end_dir;
    ROS_INFO_STREAM("Min gap is" << min_gap_dist);
    // gap_dir = 0.0f;
    end_dir = (1.75f / min_gap_dist * gap_dir + goal_dir) / (alpha / min_gap_dist + 1);

    // end_dir = (gap_dir + goal_dir) / 2;

    // if (fabs(gap_dir) > 0.2) {
    //     end_dir = gap_dir + curr_heading;
    // } else {
    //     end_dir = goal_dir;
    // }

    visualization_msgs::Marker line;
    line.header.frame_id = "/base_footprint";
    line.header.stamp = ros::Time::now();
    line.ns = "direction";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0; // Add relative robot pose
    line.id = 0;
    line.type = visualization_msgs::Marker::POINTS;
    line.scale.x = 0.01;
    line.scale.y = 0.01;
    line.color.r = 1.0;
    line.color.a = 1.0;


    // P controller
    geometry_msgs::Twist cmd;
    cmd.linear.x = std::max(std::min(0.5f * (dist_to_goal), 0.4f), -0.4f);
    // cmd.angular.z = std::max(std::min(1.0f * (goal_dir - curr_heading), 0.8f), -0.8f);
    cmd.angular.z = std::max(std::min(1.5f * (end_dir - curr_heading), 1.2f), -1.2f);

    for (float i = 0; i < 7; i += 0.1)
    {
        geometry_msgs::Point g;
        g.z = 0;
        g.x = i * cos(end_dir - curr_heading);
        g.y = i * sin(end_dir - curr_heading);
        line.points.push_back(g);
    }

    marker_pub.publish(line);

    if (cmd.linear.x < 0.1) {
        cmd.linear.x = 0;
    }

    if (fabs(cmd.angular.z) < 0.01) {
        cmd.angular.z = 0;
    }

    cmd_vel.publish(cmd);
}