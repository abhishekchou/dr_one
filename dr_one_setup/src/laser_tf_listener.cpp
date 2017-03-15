#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


using namespace ros;
Publisher laser_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    tf::TransformListener listener;
    listener.waitForTransform("/laser","/base_link",Time(0),Duration(5.0));
    sensor_msgs::LaserScan converted_scan;
    converted_scan.header = msg->header;
    converted_scan.angle_min = msg->angle_min;
    converted_scan.angle_max = msg->angle_max;
    converted_scan.angle_increment = msg->angle_increment;
    converted_scan.time_increment = msg->time_increment;
    converted_scan.scan_time = msg->scan_time;
    converted_scan.range_min = msg->range_min;
    converted_scan.range_max = msg->range_max;
    converted_scan.ranges.resize(msg->ranges.size());

    for(int i=0; i < msg->ranges.size(); i++)
    {
        float range = msg->ranges[i];
        float angle = msg->angle_min + (i * msg->angle_increment);
        geometry_msgs::PointStamped laser_pt;

        laser_pt.header.frame_id = "/laser";
        laser_pt.header.stamp = Time();
        laser_pt.point.x = range*cos(angle);
        laser_pt.point.y = range*sin(angle);
        laser_pt.point.z = 0.0;

        try
        {
            geometry_msgs::PointStamped converted_pt;
            listener.transformPoint("/base_link",laser_pt,converted_pt);
            double converted_range, converted_x, converted_y;
            converted_x = converted_pt.point.x;
            converted_y = converted_pt.point.y;
            converted_range = sqrt(pow(converted_x,2)+pow(converted_y,2));
            converted_scan.ranges[i] = converted_range;
        }

        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Exception received: %s",ex.what());
        }
    }//end for

    laser_pub.publish(converted_scan);
}

int main(int argc, char* argv[])
{
    init(argc,argv,"laser_tf_listener");
    NodeHandle n;
    laser_pub = n.advertise<sensor_msgs::LaserScan>("scan_converted",10);
    Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("scan",10,scanCallback);
    spin();
}
