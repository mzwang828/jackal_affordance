#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/OccupancyGrid.h>

#include "jackal_affordance/ValidateAction.h"

typedef actionlib::SimpleActionClient<jackal_affordance::ValidateAction> AffordanceValidate;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NamoPlanner
{
  public:
    NamoPlanner(ros::NodeHandle n) : nh_(n)
    {
        nh_.getParam("trajectory_topic", trajectory_topic_);
        nh_.getParam("dynamic_map_topic", dynamic_map_topic_);
        nh_.getParam("movebase_cancel_topic", movebase_cancel_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);

        dynamic_map_sub_ = nh_.subscribe(dynamic_map_topic_, 1, &NanoPlanner::map_callback, this);
        cancel_movebase_pub_ = nh_.advertise<actionlib_msgs::GoalID>(movebase_cancel_topic_, 1);
        affordance_detect_client_ = nh.serviceClient<jackal_affordance::AffordanceDetect>("detect_affordance");
    }

    void map_callbakc(const nav_msgs::OccupancyGrid &msg)
    {
        dynamic_map_ = msg;
    }

    // given position on /map frame, return of occupancy of that position on dynmaic_map
    int get_occupancy(float position[2], nav_msgs::OccupancyGrid map)
    {
        unsigned int grid_x = (unsigned int)((position[0] - map.info.origin.position.x) / map.info.resolution);
        unsigned int grid_y = (unsigned int)((position[1] - map.info.origin.position.y) / map.info.resolution);
        int occupancy = map.data[grid_y * map.info.width + grid_x];
        return occupancy;
    }

    // accept goal, navigate while moving obstacles
    bool go_namo(float goal_position[2])
    {
        // send goal
        MoveBaseClient mc("/move_base/move", true);
        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = fixed_frame_;
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose.position.x = goal_position[0];
        move_goal.target_pose.pose.position.y = goal_position[0];
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Sending goal");
        ac.sendGoal(move_goal);

        while(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // 2 case
            // case 1: check occupancy of current planned path
            // case 2: check movability for the obstacle
            
        }


        initial_trajectory_ = ros::topic::waitForMessage(trajectory_topic_);
    }
    // initial_trajectory_ = ros::topic::waitForMessage(trajectory_topic_);

    // actionlib_msgs::GoalID　first_goal;
    //       cancle_pub_.publish(first_goal);
  private:
    ros::NodeHandle nh_;
    ros::Subscriber trajectory_sub_, dynamic_map_sub_;
    ros::Publisher cancel_movebase_pub_;
    ros::ServiceClient affordance_detect_client_;
    AffordanceValidate affordance_validate_client_("/affordance_validate/affordance_validate", true);

    nav_msgs::Path initial_trajectory_;
    nav_msgs::OccupancyGrid dynamic_map_;

    std::string trajectory_topic_, dynamic_map_topic_, movebase_cancel_topic_, fixed_frame_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "namo_planner");
    ros::NodeHandle n("~");

    namo_planner namo_planner(n);
    ROS_INFO("Namo Planner Initialized");

    ros::spin();

    return 0;
}