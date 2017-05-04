#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <dr_one_move/move_droneAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base/move_base.h>
#include <tf/transform_datatypes.h>

using namespace ros;
using namespace std;
using namespace actionlib;
using namespace tf;


class move_drone
{

protected:
    NodeHandle nh_;
    SimpleActionServer<dr_one_move::move_droneAction> action;
    string mover_drone;
    dr_one_move::move_droneFeedback feedback_;
    dr_one_move::move_droneResult result_;

public:

    move_drone(string name);
    ~move_drone();

    Subscriber pose_sub;
    Publisher goal_pub;
    string _pose_topic, _goal_topic;

    geometry_msgs::Pose2D current_pose; //Save pose from mavros here
    geometry_msgs::Pose2D goal;
    vector<geometry_msgs::Pose2D> path;

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void exCallback(const dr_one_move::move_droneGoalConstPtr &goal);
    double getYAW(geometry_msgs::Quaternion quat);
    double distance(geometry_msgs::Pose2D start, geometry_msgs::Pose2D end);

};

move_drone::move_drone(string name) :
    action(nh_, name, boost::bind(&move_droneAction::executeCB, this, _1), false),
    mover_drone(name)
{
    action.start();
    ROS_WARN("_move_drone_:Constructing an object of class MOVE_DRONE");
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(_pose_topic.data(), 5, &move_drone::poseCallback, this);
    goal_pub = nh_.advertise<geometry_msgs::Pose2D>(_goal_topic.data(),5);
}

move_drone::~move_drone()
{
    ROS_WARN("_move_drone_:Deleting an object of class MOVE_DRONE");
}

double move_drone::getYAW(geometry_msgs::Quaternion quat)
{
    double roll, pitch, yaw;
    Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

double move_drone::distance(geometry_msgs::Pose2D start, geometry_msgs::Pose2D end)
{
    double distance;
    double x_diff = start.x - end.x;
    double y_diff = start.y - end.y;
    distance = sqrt( pow(x_diff,2) + pow(y_diff,2) );
    return distance;
}

double move_drone::getYAW(geometry_msgs::Quaternion quat)
{
    double roll, pitch, yaw;
    Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

double move_drone::distance(geometry_msgs::Pose2D start, geometry_msgs::Pose2D end)
{
    double distance;
    double x_diff = start.x - end.x;
    double y_diff = start.y - end.y;
    distance = sqrt( pow(x_diff,2) + pow(y_diff,2) );
    return distance;
}

//Save pose from EKF_POSE/MAVROS_POSE in a PoseStamped variable
void move_drone::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;

    //Save from PoseStamped to Pose2D
    current_pose.x = pose.pose.position.x;
    current_pose.y = pose.pose.position.y;
    current_pose.theta = getYAW(pose.pose.orientation);
}


void move_drone::executeCB(const dr_one_move::move_droneGoalConstPtr &target)
{
    Rate loop_rate(20);
    move_base_msgs::MoveBaseGoal new_target;
    bool success = false;
    ROS_WARN("_move_drone_:Sending Setpoint to /mavros/setpoints for drone to navigate");

    while(ok() && !success)
    {
        if(action.isPreemptRequested())
        {
            ROS_WARN("_move_drone_:Action Preempted");
            if(action.isNewGoalAvailable())
            {
                ROS_WARN("_move_drone_:New Goal Received from Exploration");
                new_target = action.acceptNewGoal();

//                double roll, pitch, yaw;
//                Matrix3x3 m(new_target.target_pose.pose.orientation);
//                m.getRPY(roll,pitch,yaw);
                goal.x = new_target.target_pose.pose.position.x;
                goal.y = new_target.target_pose.pose.position.y;
                goal.theta = getYAW(new_target.target_pose.pose.orientation);
            }
            else
                action.setPreempted();
        }

        goal.x = target->target_pose.pose.position.x;
        goal.y = target->target_pose.pose.position.y;
        goal.theta = getYAW(target->target_pose.pose.orientation);

        goal_pub.publish(goal);
        //Send Feeback
        spinOnce;

        loop_rate.sleep();

        if(distance(current_pose, goal) < 0.20)//If within 20cm of goal
        {
            feedback_.current_pose = current_pose;
            action.publishFeedback(feedback_);
            success = true;
        }
    }//end while
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_drone");
  move_droneAction move_drone("move_drone");
  ros::spin();

  return 0;
}
