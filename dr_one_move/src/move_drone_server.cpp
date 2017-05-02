#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dr_one_move/move_droneAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>

using namespace ros;
using namespace std;
using namespace actionlib;


class move_drone
{

protected:
    NodeHandle nh_;
    SimpleActionServer<dr_one_move::move_droneAction> action;
    string move_drone;
    dr_one_move::move_droneFeedback feedback_;
    dr_one_move::move_droneResult result_;

public:
    move_drone(string name);
    geometry_msgs::PoseStamped current_pose; //Save pose from mavros here
    geometry_msgs::PoseStamped pose_frontier;
    vector<geometry_msgs::Pose2D> goal_array;

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void exCallback(const dr_one_move::move_droneGoalConstPtr &goal);
    Subscriber pose_sub;
    Publisher pose_pub;
    string _position_topic, _goal_topic;

move_drone::move_drone(string name) :
    action(nh_, name, boost::bind(&move_droneAction::executeCB, this, _1), false),
    move_drone(name)
{
    action.start();
    ROS_INFO("_move_drone_:Constructing an object of class MOVE_DRONE");
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(_position_topic.data(), 5, &move_drone::poseCallback, this);
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(_goal_topic.data(),5);
}

//Save pose from MAVROS topic into PoseStamped variable
void move_drone::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose.header = msg->header;
    current_pose.pose = msg->pose;
}

void move_drone::executeCB(const dr_one_move::move_droneGoalConstPtr &goal)
{
    Rate loop_rate(20);
    bool success = true;
    ROS_INFO("Sending Setpoint to /mavros/setpoints for drone to navigate");
    for(int i=0; i<=goal->order;i++)
    {
      if(action.isPreemptRequested()||!ros.ok())
      {
          ROS_INFO("Action Preempted");
          action.setPreempted();
          success = false;
          break;
      }

      
    }


 
     
    

}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_drone");

  move_droneAction move_drone("move_drone");
  ros::spin();

  return 0;
}
