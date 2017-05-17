#include <ros/ros.h>
#include <cmath>

#include <actionlib/server/simple_action_server.h>
#include <dr_one_move/move_droneAction.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
// #include <dr_one_move/move_base.h>
#include <tf/transform_datatypes.h>

using namespace ros;
using namespace std;
using namespace actionlib;
using namespace tf;
using namespace dr_one_move;

bool verbose(true);


class move_drone
{

protected:
    NodeHandle nh_;
    SimpleActionServer<dr_one_move::move_droneAction> action;
    string mover_drone;
    dr_one_move::move_droneFeedback feedback_;
    dr_one_move::move_droneResult result_;

private:
    // void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void exCallback(const dr_one_move::move_droneGoalConstPtr &target);
    double getYAW_(geometry_msgs::Quaternion quat);
    double distance(geometry_msgs::Pose2D start, geometry_msgs::PoseStamped end);

public:
    // move_drone(string name);
    // ~move_drone();
    Subscriber pose_sub;
    Publisher goal_pub;
    string _pose_topic, _goal_topic;

    geometry_msgs::Pose2D pose_2d; //Save pose from mavros here
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped pose;
    // vector<geometry_msgs::Pose2D> path;

move_drone(string name) :
    action(nh_, name, boost::bind(&move_drone::exCallback, this, _1), false),
    mover_drone(name)
{
    action.start();
    ROS_WARN("_move_drone_:Constructing an object of class MOVE_DRONE");
    nh_.getParam("pose_topic",_pose_topic);
    nh_.getParam("goal_topic",_goal_topic);
    pose_sub = nh_.subscribe<nav_msgs::Odometry>(_pose_topic.data(), 5, &move_drone::poseCallback, this);
    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>(_goal_topic.data(),5);
}

~move_drone()
{
    ROS_WARN("_move_drone_:Deleting an object of class MOVE_DRONE");
}
};


double move_drone::getYAW_(geometry_msgs::Quaternion quat)
{
    double yaw;
    yaw = getYaw(quat);
    return yaw;
}

double move_drone::distance(geometry_msgs::Pose2D start, geometry_msgs::PoseStamped end)
{
    double distance;
    double x_diff = start.x - end.pose.position.x;
    double y_diff = start.y - end.pose.position.y;
    distance = sqrt( pow(x_diff,2) + pow(y_diff,2) );
    return distance;
}

// //Save pose from EKF_POSE/MAVROS_POSE in a PoseStamped variable
// void move_drone::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     pose.header = msg->header;
//     pose.pose = msg->pose;
//
//     //Save from PoseStamped to Pose2D
//     pose_2d.x = pose.pose.position.x;
//     pose_2d.y = pose.pose.position.y;
//     pose_2d.theta = getYAW_(pose.pose.orientation);
// }

//Save pose from EKF_POSE/MAVROS_POSE in a PoseStamped variable
void move_drone::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    //Save from PoseStamped to Pose2D
    pose_2d.x = pose.pose.position.x;
    pose_2d.y = pose.pose.position.y;
    pose_2d.theta = getYAW_(pose.pose.orientation);
}

void move_drone::exCallback(const dr_one_move::move_droneGoalConstPtr &target)
{
    Rate loop_rate(100);
    // move_base_msgs::MoveBaseGoal new_target;
    move_droneGoal new_target;
    bool success = false;
    ROS_WARN("_move_drone_:Sending Setpoint to mavros for drone to navigate");

    while(ok() && !success)
    {
        if(action.isPreemptRequested())
        {
            ROS_WARN("_move_drone_:Action Preempted");
            if(action.isNewGoalAvailable())
            {
                ROS_WARN("_move_drone_:New Goal Received from Exploration");
                new_target = *action.acceptNewGoal();

                goal.pose.position = target->target_pose.pose.position;
                goal.pose.position.z = 1.0;
                goal.pose.orientation = target->target_pose.pose.orientation;

                // goal.x = new_target.target_pose.pose.position.x;
                // goal.y = new_target.target_pose.pose.position.y;
                // goal.theta = getYAW_(new_target.target_pose.pose.orientation);
            }
            else
                action.setPreempted();
        }

        // goal.x = target->target_pose.pose.position.x;
        // goal.y = target->target_pose.pose.position.y;
        // goal.theta = getYAW_(target->target_pose.pose.orientation);

        goal.pose.position = target->target_pose.pose.position;
        goal.pose.position.z = 1.0;
        goal.pose.orientation = target->target_pose.pose.orientation;


        if(verbose){
          ROS_WARN("_move_drone_:Quaternion Goal being to mavros Pose(x=%f, y=%f, z=%f) & Orientation(x=%f, y=%f, z=%f, w=%f)\n",
                  goal.pose.position.x,
                  goal.pose.position.y,
                  goal.pose.position.z,
                  goal.pose.orientation.x,
                  goal.pose.orientation.y,
                  goal.pose.orientation.z,
                  goal.pose.orientation.w);}

        // ROS_WARN("_move_drone_:Euler Goal being to mavros Pose(x=%f, y=%f, theta=%f)\n",
        //           goal.x,
        //           goal.y,
        //           goal.theta);

        goal_pub.publish(goal);
        spinOnce;

        loop_rate.sleep();
        if(distance(pose_2d, goal) < 0.20)//If within 20cm of goal
        {
            feedback_.current_pose = pose;
            action.publishFeedback(feedback_);
            success = true;
        }
    }//end while
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_drone");
  move_drone move_drone("move_drone");
  ros::spin();

  return 0;
}
