#include <iostream>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>

#include "jackal_affordance/ValidateAction.h"
#include "jackal_affordance/AffordanceDetect.h"
#include "jackal_affordance/Primitive.h"

typedef actionlib::SimpleActionClient<jackal_affordance::ValidateAction> AffordanceValidate;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NamoPlanner
{
  public:
    NamoPlanner(ros::NodeHandle n) : nh_(n), affordance_validate_client_("/hsr_affordance_validate/affordance_validate/", true)
    {
        nh_.getParam("trajectory_topic", trajectory_topic_);
        nh_.getParam("dynamic_map_topic", dynamic_map_topic_);
        nh_.getParam("movebase_cancel_topic", movebase_cancel_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);

        dynamic_map_sub_ = nh_.subscribe(dynamic_map_topic_, 1, &NamoPlanner::map_callback, this);
        robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &NamoPlanner::pose_callback, this);
        cancel_movebase_pub_ = nh_.advertise<actionlib_msgs::GoalID>(movebase_cancel_topic_, 1);
        affordance_detect_client_ = nh_.serviceClient<jackal_affordance::AffordanceDetect>("detect_affordance");
        initial_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("initial_trajectory", 1);
    }

    void map_callback(const nav_msgs::OccupancyGrid &msg)
    {
        dynamic_map_ = msg;
    }

    void pose_callback(const geometry_msgs::Pose &msg)
    {
        robot_x_ = msg.position.x;
        robot_y_ = msg.position.y;
    }

    // given position on /map frame, return of occupancy of that position on dynmaic_map
    int get_occupancy(double position[2], nav_msgs::OccupancyGrid map)
    {
        int grid_x = (int)((position[0] - map.info.origin.position.x) / map.info.resolution);
        int grid_y = (int)((position[1] - map.info.origin.position.y) / map.info.resolution);
        int index = grid_y * map.info.width + grid_x;
        if ((grid_x >= 0) && (grid_y >= 0) && (index < map.info.width * map.info.height))
        {
            int occupancy = map.data[index];
            return occupancy;
        }
        else
            return 0;
    }

    // accept goal, navigate while moving obstacles
    void go_namo(double goal_position[2])
    {
        // send goal
        MoveBaseClient mc("/move_base/move", true);
        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = fixed_frame_;
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose.position.x = goal_position[0];
        move_goal.target_pose.pose.position.y = goal_position[1];
        while (!mc.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Sending goal");
        mc.sendGoal(move_goal);
        ros::Duration(0.5).sleep();
        boost::shared_ptr<nav_msgs::Path const> sharedTrajectory;
        sharedTrajectory = ros::topic::waitForMessage<nav_msgs::Path>(trajectory_topic_);
        if (sharedTrajectory != NULL)
        {
            initial_trajectory_ = *sharedTrajectory;
        }
        else
            ROS_INFO("No trajectory received");

        int namo_state = 1;
        actionlib_msgs::GoalID empty_goal;
        while ((robot_x_ - goal_position[0] > 0.1) || (robot_y_ - goal_position[1] > 0.1))
        {
            initial_trajectory_pub_.publish(initial_trajectory_);
            // 2 case
            // case 1: check occupancy of current planned path
            // case 2: check movability for the obstacle
            switch (namo_state)
            {
            case 1:
                for (int i = 0; i < initial_trajectory_.poses.size(); i++)
                {
                    double position[2] = {initial_trajectory_.poses[i].pose.position.x, initial_trajectory_.poses[i].pose.position.y};
                    int occupancy = this->get_occupancy(position, dynamic_map_);
                    ROS_INFO("%d Occupancy: %d", i, occupancy);
                    if (occupancy > 50)
                    {
                        cancel_movebase_pub_.publish(empty_goal);
                        namo_state = 2;
                        break;
                    }
                }
                break;

            case 2:
                jackal_affordance::AffordanceDetect detect_srv;
                if (affordance_detect_client_.call(detect_srv))
                {
                    if (detect_srv.response.success)
                    {
                        ROS_INFO("Affordance Detect Successfully, find %lu affordance candidate(s)", detect_srv.response.primitives.size());
                        // TODO: pick right obstacle to validate
                        jackal_affordance::Primitive affordance_to_validate = detect_srv.response.primitives[0];
                        affordance_validate_client_.waitForServer();
                        jackal_affordance::ValidateGoal validate_goal;
                        validate_goal.center = affordance_to_validate.center;
                        validate_goal.normal = affordance_to_validate.normal;
                        affordance_validate_client_.sendGoal(validate_goal);
                        affordance_validate_client_.waitForResult();
                        if ((affordance_validate_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
                        {
                            jackal_affordance::ValidateResult validate_result = *affordance_validate_client_.getResult();
                            if (validate_result.movable)
                            {
                                ROS_INFO("The obstacle is movable, now moving the obstacle to clear the path");
                            }
                            else
                            {
                                ROS_INFO("The obstacle is not movable, now re-planning the path");
                                mc.sendGoal(move_goal);
                                sharedTrajectory = ros::topic::waitForMessage<nav_msgs::Path>(trajectory_topic_);
                                if (sharedTrajectory != NULL)
                                {
                                    initial_trajectory_ = *sharedTrajectory;
                                }
                                else
                                    ROS_INFO("No trajectory received");
                            }
                        }
                        else
                            ROS_INFO("Failed to Validate the Affordance Candidate");
                    }
                    else
                        ROS_INFO("No Affordance Detected");
                }
                break;
            }
        }
        ROS_INFO("Arrival!");
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber trajectory_sub_, dynamic_map_sub_, robot_pose_sub_;
    ros::Publisher cancel_movebase_pub_, initial_trajectory_pub_;
    ros::ServiceClient affordance_detect_client_;
    AffordanceValidate affordance_validate_client_;

    nav_msgs::Path initial_trajectory_;
    nav_msgs::OccupancyGrid dynamic_map_;

    std::string trajectory_topic_, dynamic_map_topic_, movebase_cancel_topic_, fixed_frame_;
    float robot_x_, robot_y_;
};

main(int argc, char *argv[])
{
    if (argc < 3)
        return -1;

    ros::init(argc, argv, "namo_planner");
    ros::NodeHandle n("~");
    NamoPlanner namo_planner(n);
    ROS_INFO("Namo Planner Initialized");
    double goal[2] = {atof(argv[1]), atof(argv[2])};
    namo_planner.go_namo(goal);

    ros::spin();

    return 0;
}