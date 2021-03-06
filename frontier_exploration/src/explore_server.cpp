#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <tf/transform_listener.h>

// #include <move_base_msgs/MoveBaseAction.h>
#include <dr_one_move/move_droneAction.h>

#include <frontier_exploration/geometry_tools.h>

using namespace ros;
using namespace boost;
using namespace actionlib;
using namespace std;

namespace frontier_exploration{

/*
 * Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */

class FrontierExplorationServer
{

public:

    /*
     * @brief Constructor for the server, sets up this node's ActionServer for exploration
     *         and ActionClient to move_base for robot movement.
     *
     * @param "name" Name for SimpleActionServer
     */
    FrontierExplorationServer(string name) :
        tf_listener_(Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, bind(&FrontierExplorationServer::executeCb, this, _1), false),
        move_client_("move_drone",true),
        retry_(5)
    {
        private_nh_.param<double>("frequency", frequency_, 0.0);
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);

        explore_costmap_ros_ = shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));

        as_.registerPreemptCallback(bind(&FrontierExplorationServer::preemptCb, this));
        as_.start();
    }

private:

    NodeHandle nh_;
    NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

    shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    double frequency_, goal_aliasing_;
    bool success_, moving_;
    int retry_;

    mutex move_client_lock_;
    frontier_exploration::ExploreTaskFeedback feedback_;
    // SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    // move_base_msgs::MoveBaseGoal move_client_goal_;

    //Client for to send goals to move_drone actionlib
    SimpleActionClient<dr_one_move::move_droneAction> move_client_;
    dr_one_move::move_droneGoal move_client_goal_;

    /*
     *  @brief Execute callback for actionserver, run after accepting a new goal
     *
     *  @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
    {

        //setup bool check variables
        success_ = false;
        moving_ = false;

        //clear costmap each time callback is executed
        explore_costmap_ros_->resetLayers();

        //create costmap services
        ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");

        //wait for move_base and costmap services
        if(!move_client_.waitForServer() ||
           !updateBoundaryPolygon.waitForExistence() ||
           !getNextFrontier.waitForExistence())
        {
            as_.setAborted();
            return;
        }

        //set region boundary on costmap
        if(ok() && as_.isActive())
        {
            frontier_exploration::UpdateBoundaryPolygon srv;
            srv.request.explore_boundary = goal->explore_boundary;
            if(updateBoundaryPolygon.call(srv))
            {
                ROS_INFO("_explore_server_:Region boundary set");
            }
            else
            {
                ROS_ERROR("_explore_server_:Failed to set region boundary");
                as_.setAborted();
                return;
            }
        }

        //loop until all frontiers are explored
        Rate rate(frequency_);
        while(ok() && as_.isActive())
        {

            frontier_exploration::GetNextFrontier srv;

            //placeholder for next goal to be sent to move base
            geometry_msgs::PoseStamped goal_pose;

            //get current robot pose in frame of exploration boundary
            tf::Stamped<tf::Pose> robot_pose;
            explore_costmap_ros_->getRobotPose(robot_pose);

            //provide current robot pose to the frontier search service request
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);

            //evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::PoseStamped eval_pose = srv.request.start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id)
            {
                tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
            }

            //check if robot is not within exploration boundary and needs to return to center of search area
            if(goal->explore_boundary.polygon.points.size() > 0 &&
               !pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon))
            {

                //check if robot has explored at least one frontier, and promote debug message to warning
                if(success_)
                {
                    ROS_WARN("_explore_server_:Robot left exploration boundary, returning to center");
                }
                else
                {
                    ROS_DEBUG("_explore_server_:Robot not initially in exploration boundary, traveling to center");
                }
                //get current robot position in frame of exploration center
                geometry_msgs::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;

                if(eval_point.header.frame_id != goal->explore_center.header.frame_id)
                {
                    geometry_msgs::PointStamped temp = eval_point;
                    tf_listener_.transformPoint(goal->explore_center.header.frame_id, temp, eval_point);
                }

                //set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(eval_point.point, goal->explore_center.point) );

            }

            //Robot is inside the exploration boundary
            else if(getNextFrontier.call(srv))
            {
                //if in boundary, try to find next frontier to search
                ROS_DEBUG("_explore_server_:Found frontier to explore");
                success_ = true;
                goal_pose = feedback_.next_frontier = srv.response.next_frontier;
                retry_ = 5;

            }
            else
            {
                //if no frontier found, check if search is successful
                ROS_DEBUG("_explore_server_:Couldn't find a frontier");

                //search is succesful
                if(retry_ == 0 && success_)
                {
                    ROS_WARN("_explore_server_:Finished exploring room");
                    as_.setSucceeded();
                    unique_lock<mutex> lock(move_client_lock_);
                    move_client_.cancelGoalsAtAndBeforeTime(Time::now());
                    return;

                }
                else if(retry_ == 0 || !ok())
                { //search is not successful

                    ROS_ERROR("_explore_server_:Failed exploration");
                    as_.setAborted();
                    return;
                }

                ROS_DEBUG("_explore_server_:Retrying...");
                retry_--;
                //try to find frontier again, without moving robot
                continue;
            }
            //if above conditional does not escape this loop step, search has a valid goal_pose

            //check if new goal is close to old goal, hence no need to resend
            if(!moving_ || !pointsNearby(move_client_goal_.target_pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.5))
            {
                ROS_DEBUG("_explore_server_:New exploration goal");
                move_client_goal_.target_pose = goal_pose;
                unique_lock<mutex> lock(move_client_lock_);
                if(as_.isActive())
                {
                    move_client_.sendGoal(move_client_goal_, bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),0,bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
                    moving_ = true;
                }
                lock.unlock();
            }

            //check if continuous goal updating is enabled
            if(frequency_ > 0)
            {
                //sleep for specified frequency and then continue searching
                rate.sleep();
            }
            else
            {
                //wait for movement to finish before continuing
                while(ok() && as_.isActive() && moving_)
                {
                    WallDuration(0.1).sleep();
                }
            }
        }

        //goal should never be active at this point
        ROS_ASSERT(!as_.isActive());

    }


    /*
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */
    void preemptCb()
    {

        unique_lock<mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(Time::now());
        ROS_WARN("_explore_server_:Current exploration task cancelled");

        if(as_.isActive())
        {
            as_.setPreempted();
        }

    }

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(const dr_one_move::move_droneFeedbackConstPtr& feedback)
    {
        feedback_.current_pose = feedback->current_pose;
        as_.publishFeedback(feedback_);
    }

    /*
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     *
     *  @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const SimpleClientGoalState& state, const dr_one_move::move_droneResultConstPtr& result)
    {
        if (state == SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("_explore_server_:Failed to move");
            as_.setAborted();
        }
        else if(state == SimpleClientGoalState::SUCCEEDED)
        {
            moving_ = false;
        }
    }

};

}

int main(int argc, char** argv)
{
    init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(this_node::getName());
    spin();
    return 0;
}
