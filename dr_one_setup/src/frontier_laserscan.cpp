#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

using namespace ros;
double _scan_threshold_frontier;
Publisher laser_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan throttled_scan;
    throttled_scan.header = msg->header;
    throttled_scan.angle_min = msg->angle_min;
    throttled_scan.angle_max = msg->angle_max;
    throttled_scan.angle_increment = msg->angle_increment;
    throttled_scan.time_increment = msg->time_increment;
    throttled_scan.scan_time = msg->scan_time;
    throttled_scan.range_min = msg->range_min;
    throttled_scan.range_max = _scan_threshold_frontier;
    throttled_scan.ranges.resize(msg->ranges.size());

}

int main(int argc, char const *argv[]) {
  init(argv, argc, "laser_tf_listener");
  NodeHandle n;

  //Read from parameter sever to limit the range of the scan that the frontier node can see
  n.getParam("scan_threshold_frontier",_scan_threshold_frontier);

  laser_pub = n.advertise<sensor_msgs::LaserScan>("/frontier_scan",10);
  Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("/gazebo_scan",scanCallback);
  return 0;
}
