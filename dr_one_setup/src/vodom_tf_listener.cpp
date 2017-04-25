#include <math.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>

using namespace ros;
using namespace tf;
Publisher odom_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    TransformListener listener;
    listener.waitForTransform("vodom","map",Time(0), Duration(5.0));

    nav_msgs::Odometry odom;
    odom.header = msg->header;
    odom.child_frame_id = "/dr_one";

    geometry_msgs::PoseStamped vodom_pose;
//    geometry_msgs::QuaternionStamped vodom_quat;
//    geometry_msgs::TwistStamped vodom_twist;
    geometry_msgs::Vector3Stamped vodom_twist_linear;
    geometry_msgs::Vector3Stamped vodom_twist_angular;

    geometry_msgs::PoseStamped odom_pose;
//    geometry_msgs::QuaternionStamped odom_quat;
    geometry_msgs::TwistStamped odom_twist;
    geometry_msgs::Vector3Stamped odom_twist_linear;
    geometry_msgs::Vector3Stamped odom_twist_angular;

    vodom_pose.header = msg->header;
    vodom_pose.pose.position = msg->pose.pose.position;

//    vodom_quat.header = msg->header;
//    vodom_quat.quaternion = msg->pose.pose.orientation;
    odom_pose.header.frame_id = "/odom";
//    odom_quat.header = msg->header;

    vodom_twist_angular.header = msg->header;
    vodom_twist_angular.header.frame_id = "/vodom";
    vodom_twist_linear.header.stamp = Time();
    vodom_twist_linear.header.frame_id = "/vodom";

    vodom_twist_angular.vector = msg->twist.twist.angular;
    vodom_twist_linear.vector = msg->twist.twist.linear;

    odom_twist.header = msg->header;
    odom_twist_angular.header.stamp = Time();
    odom_twist_angular.header.frame_id = "/odom";
    odom_twist_linear.header.stamp = Time();
    odom_twist_linear.header.frame_id = "/odom";

    try
    {
//        StampedTransform vodom_to_map;
        listener.waitForTransform("vodom","map",Time(0), Duration(5.0));
        listener.transformPose("map",vodom_pose,odom_pose);
//        listener.transformQuaternion("/map",vodom_quat,odom_quat);
        listener.transformVector("map",vodom_twist_angular,odom_twist_angular);
        listener.transformVector("map",vodom_twist_linear,odom_twist_linear);

    }
    catch(TransformException& ex)
    {
        ROS_ERROR("_VODOM TF LISTENER_: Exception received: %s", ex.what());
    }

    odom.header.frame_id = "/odom";
    odom.pose.pose.position = odom_pose.pose.position;
    odom.twist.twist.linear = odom_twist_linear.vector;
    odom.twist.twist.angular= odom_twist_angular.vector;
    odom_pub.publish(odom);

}

int main(int argc, char* argv[])
{
    init(argc,argv,"vodom_tf_listener");
    NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",100);
    Subscriber vodom_sub = n.subscribe<nav_msgs::Odometry>("/vodom", 10, odomCallback);

    spin();
}
