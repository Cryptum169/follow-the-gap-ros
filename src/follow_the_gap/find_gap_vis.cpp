#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <queue>
#include <std_msgs/ColorRGBA.h>
#include <stdlib.h>
using namespace std;
void laserMsgCallback(const sensor_msgs::LaserScan);

class Gap
{
    int start_angle;
    int size;
    float dist;

  public:
    Gap(int _start, int _size, float _dist)
    {
        start_angle = _start;
        size = _size;
        dist = _dist;
    }
    void setSize(int val) {size = val;}
    void setStart(int val) {start_angle = val;}
    float getDistance() const { return dist; }
    int getStartAngle() const { return start_angle; }
    int getSize() const { return size; }
};

class gapComparator
{
  public:
    int operator()(const Gap &g1, const Gap &g2)
    {
        return g1.getSize() < g2.getSize();
    }
};

Gap currLarge(0,0,0);
ros::Publisher marker_pub;
ros::Publisher dir_pub;

float x = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_Laser");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Laser Marker initiated");
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    dir_pub = nh.advertise<geometry_msgs::Twist>("gap_dir",10);
    string sensor_name = getenv("TURTLEBOT_3D_SENSOR");
    string subtopic = "/scan";
    if (sensor_name == "hokuyo") {
        subtopic = "/laserscan";
    }
    ros::Subscriber sub = nh.subscribe(subtopic, 1000, &laserMsgCallback);
    ros::spin();

    return 0;
}

void laserMsgCallback(const sensor_msgs::LaserScan msg) {
    // Min gap 
    // sensor_msgs::LaserScan msg = msg;
    priority_queue <Gap, vector<Gap>, gapComparator> pq;
    // Gap initial_gap(-1,-1,0); // in case no gap
    int start_nan_idx = 0;
    int size = 0;
    bool prev = true;

    visualization_msgs::Marker obstacle;
    obstacle.header.frame_id = "/base_footprint";
    obstacle.header.stamp = ros::Time::now();
    obstacle.ns = "lines";
    obstacle.action = visualization_msgs::Marker::ADD;
    obstacle.pose.orientation.w = 1.0; // Add relative robot pose  
    obstacle.id = 0;
    obstacle.type = visualization_msgs::Marker::POINTS;
    obstacle.scale.x = 0.1;
    obstacle.scale.y = 0.1;
    obstacle.color.b = 1.0;
    obstacle.color.a = 1.0;

    float left_dist = -1;
    float gap_dist = 10;
    float min_dist = 10;

    for (std::vector<float>::size_type it = 0; it < msg.ranges.size(); ++it)
    {
        if (prev){
            if (isinf(msg.ranges[it]) || isnan(msg.ranges[it]) || msg.ranges[it] > 3.0f)
            {
                ++size;
            } else {
                if (size > 1) {
                    if (left_dist == -1) {
                        gap_dist = msg.ranges[it];
                    } else {
                        gap_dist = (msg.ranges[it] + left_dist) / 2;
                        // gap_dist = std::min(msg.ranges[it], left_dist);
                    }
                    pq.push(Gap(start_nan_idx, size, gap_dist));
                    
                    if (gap_dist < min_dist) {
                        min_dist = gap_dist;
                    }

                    if (left_dist < min_dist)
                    {
                        min_dist = left_dist;
                    }
                }
                start_nan_idx = -1;
                left_dist = msg.ranges[it];
                size = 0;
                prev = false;
            }
        } else {
            if (isinf(msg.ranges[it]) || isnan(msg.ranges[it]) || msg.ranges[it] > 3.0f)
            {
                start_nan_idx = it;
                left_dist = msg.ranges[it - 1];
                size = 1;
                prev = true;
            }
        }

        if (!(isinf(msg.ranges[it]) || isnan(msg.ranges[it]) || msg.ranges[it] > 3.0f))
        {
            float dist = msg.ranges[it];
            geometry_msgs::Point p;
            p.z = 0;
            float angle = msg.angle_min + msg.angle_increment * it;
            p.x = dist * cos(angle);
            p.y = dist * sin(angle);
            obstacle.points.push_back(p);
        }
    }

    // Wrap up for the last one
    if (prev) {
        pq.push(Gap(start_nan_idx, size, left_dist));
    }

    if (left_dist < min_dist)
    {
        min_dist = left_dist;
    }

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
    line.color.g = 1.0;
    line.color.a = 1.0;

    float angle;
    if (pq.size() != 0){
        ROS_INFO_STREAM("Gap count:" << pq.size());
        currLarge = pq.top();
        ROS_INFO_STREAM("Distance: " << currLarge.getDistance() << "size: " << currLarge.getSize());
        angle = (currLarge.getStartAngle() + currLarge.getSize() / 2) * msg.angle_increment + msg.angle_min;
    } else {
        angle = 0;
    }

    geometry_msgs::Twist target_angle;
    target_angle.angular.z = angle;
    target_angle.linear.x = min_dist;
    dir_pub.publish(target_angle);
    // dir_pub.publish(angle);

    for (float i = 0; i < 7; i += 0.1) {
        geometry_msgs::Point g;
        g.z = 0;
        g.x = i * cos(angle);
        g.y = i * sin(angle);
        line.points.push_back(g);
        geometry_msgs::Point min;
        min.z = 0;
        min.x = i * cos(msg.angle_min);
        min.y = i * sin(msg.angle_min);
        line.points.push_back(min);
        geometry_msgs::Point max;
        max.z = 0;
        max.x = i * cos(msg.angle_max);
        max.y = i * sin(msg.angle_max);
        line.points.push_back(max);
    }

    marker_pub.publish(line);
    marker_pub.publish(obstacle);
}






