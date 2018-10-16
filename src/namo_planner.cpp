#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <numeric>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>

// arm relative
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <frasier_utilities/gripper.h>
#include <geometry_msgs/Twist.h>

#include "jackal_affordance/ValidateAction.h"
#include "jackal_affordance/AffordanceDetect.h"
#include "jackal_affordance/Primitive.h"

typedef actionlib::SimpleActionClient<jackal_affordance::ValidateAction> AffordanceValidate;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class NamoPlanner
{
  public:
    NamoPlanner(ros::NodeHandle n) : nh_(n), affordance_validate_client_("/hsr_affordance_validate/affordance_validate/", true), gc_(n, true)
    {
        nh_.getParam("trajectory_topic", trajectory_topic_);
        nh_.getParam("dynamic_map_topic", dynamic_map_topic_);
        nh_.getParam("movebase_cancel_topic", movebase_cancel_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);
        nh_.getParam("move_base_topic", move_base_topic_);
        nh_.getParam("/move_base/local_costmap/inflation_radius", inflation_radius_);

        dynamic_map_sub_ = nh_.subscribe(dynamic_map_topic_, 1, &NamoPlanner::map_callback, this);
        robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &NamoPlanner::pose_callback, this);
        cancel_movebase_pub_ = nh_.advertise<actionlib_msgs::GoalID>(movebase_cancel_topic_, 1);
        affordance_detect_client_ = nh_.serviceClient<jackal_affordance::AffordanceDetect>("/hsr_affordance_detect/affordance_detect");
        initial_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("initial_trajectory", 1);
        path_sub_ = nh_.subscribe(trajectory_topic_, 1, &NamoPlanner::path_callback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/hsrb/opt_command_velocity", 10);
    }

    void map_callback(const nav_msgs::OccupancyGrid &msg)
    {
        dynamic_map_ = msg;
    }

    void pose_callback(const geometry_msgs::Pose &msg)
    {
        //ROS_INFO("update robot pose");
        robot_x_ = msg.position.x;
        robot_y_ = msg.position.y;
    }

    void path_callback(const nav_msgs::Path &msg)
    {
        //ROS_INFO("update path");
        initial_trajectory_ = msg;
    }

    bool move_arm(float joint_state[5])
    {
        TrajectoryClient tc("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true);
        tc.waitForServer();
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("arm_lift_joint");
        goal.trajectory.joint_names.push_back("arm_flex_joint");
        goal.trajectory.joint_names.push_back("arm_roll_joint");
        goal.trajectory.joint_names.push_back("wrist_flex_joint");
        goal.trajectory.joint_names.push_back("wrist_roll_joint");

        goal.trajectory.points.resize(1);

        goal.trajectory.points[0].positions.resize(5);
        goal.trajectory.points[0].positions[0] = joint_state[0];
        goal.trajectory.points[0].positions[1] = joint_state[1];
        goal.trajectory.points[0].positions[2] = joint_state[2];
        goal.trajectory.points[0].positions[3] = joint_state[3];
        goal.trajectory.points[0].positions[4] = joint_state[4];
        goal.trajectory.points[0].velocities.resize(5);

        for (size_t i = 0; i < 5; ++i)
        {
            goal.trajectory.points[0].velocities[i] = 0.0;
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(3.0);

        tc.sendGoal(goal);
        tc.waitForResult(ros::Duration(5.0));

        if (tc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            return true;
        else
            return false;
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

    // iterate all detected afffordances to find the one block the way
    int which_primitive(double position[2], std::vector<jackal_affordance::Primitive> primitives)
    {
        for (int i = 0; i < primitives.size(); i++)
        {
            jackal_affordance::Primitive current_primitive = primitives[i];
            if (!current_primitive.is_vertical)
                continue;
            // vector from center of primitive to occupied trajectory position
            std::vector<double> traj_vector = {position[0] - current_primitive.center.x, position[1] - current_primitive.center.y};
            // check for plane
            if (current_primitive.type == 1)
            {
                // unit normal vector of the plane
                double mag = sqrt(current_primitive.normal[0] * current_primitive.normal[0] +
                                  current_primitive.normal[1] * current_primitive.normal[1]);
                std::vector<double> normal = {current_primitive.normal[0] / mag, current_primitive.normal[1] / mag};
                double distance_away = abs(std::inner_product(traj_vector.begin(), traj_vector.end(), normal.begin(), 0));
                double distance_along = sqrt((traj_vector[0] * traj_vector[0] + traj_vector[1] * traj_vector[1]) - distance_away * distance_away);
                double length = sqrt((current_primitive.max.x - current_primitive.min.x) * (current_primitive.max.x - current_primitive.min.x) +
                                     (current_primitive.max.y - current_primitive.min.y) * (current_primitive.max.y - current_primitive.min.y));
                if ((distance_away < inflation_radius_) && (distance_along < (length / 2 + inflation_radius_)))
                {
                    ROS_INFO("current primitive: %f, %f", current_primitive.center.x, current_primitive.center.y);
                    ROS_INFO("distance_away: %f, distance along: %f, length: %f, inflation: %f", distance_away, distance_along, length, inflation_radius_);
                    ROS_INFO("traj_vecot: %f, %f, normal: %f, %f", traj_vector[0], traj_vector[1], normal[0], normal[1]);
                    ROS_INFO("Found obstacle primitive (Plane)");
                    return i;
                }
            }
            // check for cylinder
            else if (current_primitive.type == 3)
            {
                double distance_radius = sqrt((traj_vector[0] * traj_vector[0] + traj_vector[1] * traj_vector[1]));
                if (distance_radius < (current_primitive.coef[6] + inflation_radius_))
                {
                    ROS_INFO("Found obstacle primitive (Cylinder)");
                    return i;
                }
            }
        }
        ROS_INFO("Error: Couldn't find obstacle primitive");
        return 10086;
    }

    // accept goal, navigate while moving obstacles
    void go_namo(double goal_position[2])
    {
        // send goal
        MoveBaseClient mc(move_base_topic_, true);
        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = fixed_frame_;
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose.position.x = goal_position[0];
        move_goal.target_pose.pose.position.y = goal_position[1];
        move_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("move_base goal %f, %f ", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.y);
        while (!mc.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Sending goal");
        mc.sendGoal(move_goal);
        ros::Duration(0.5).sleep();

        ros::spinOnce();

        int namo_state = 1;
        actionlib_msgs::GoalID empty_goal;
        while ((abs(robot_x_ - goal_position[0]) > 0.05) || (abs(robot_y_ - goal_position[1]) > 0.05))
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
                    // get each coordinate of the trajectory and check its occupancy
                    traj_coordinate_[0] = initial_trajectory_.poses[i].pose.position.x;
                    traj_coordinate_[1] = initial_trajectory_.poses[i].pose.position.y;
                    int occupancy = this->get_occupancy(traj_coordinate_, dynamic_map_);
                    if (occupancy > 70)
                    {
                        cancel_movebase_pub_.publish(empty_goal);
                        namo_state = 2;
                        break;
                    }
                }
                break;

            case 2:
                jackal_affordance::AffordanceDetect detect_srv;
                jackal_affordance::ValidateGoal validate_goal;
                ROS_INFO("Trajectory blocked, try to move obstacle");
                if (affordance_detect_client_.call(detect_srv))
                {
                    // check if successful
                    if (detect_srv.response.success)
                    {
                        ROS_INFO("Affordance Detect Successfully, find %lu affordance candidate(s)", detect_srv.response.primitives.size());
                        // find the primitive that blocks the trajectory
                        int obstacle_index = this->which_primitive(traj_coordinate_, detect_srv.response.primitives);
                        ROS_INFO("primitive index %d", obstacle_index);
                        ROS_INFO("Trajectory coordinate %f, %f", traj_coordinate_[0], traj_coordinate_[1]);

                        jackal_affordance::Primitive current_primitive = detect_srv.response.primitives[obstacle_index];
                        // for cylinders
                        if (current_primitive.type == 3)
                        {
                            ROS_INFO("Cylinder Detected");
                            // calcualte directional vector between robot and cylinder
                            float distance_x = current_primitive.center.x - robot_x_;
                            float distance_y = current_primitive.center.y - robot_y_;
                            float mag = sqrt(distance_x * distance_x + distance_y * distance_y);
                            float normal_x = distance_x / mag;
                            float normal_y = distance_y / mag;
                            validate_goal.center = current_primitive.center;
                            validate_goal.normal[0] = normal_x;
                            validate_goal.normal[1] = normal_y;
                            validate_goal.normal[2] = 0;
                            validate_goal.primitive_type = current_primitive.type;
                            validate_goal.validate_type = 2;
                            affordance_validate_client_.sendGoal(validate_goal);
                            affordance_validate_client_.waitForResult();
                            if ((affordance_validate_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
                            {
                                jackal_affordance::ValidateResult validate_result = *affordance_validate_client_.getResult();
                                if (validate_result.result)
                                {
                                    ROS_INFO("The object is liftable. Pick up to continue.");
                                    //TODO: Pick up obstacle
                                    ros::Duration(1).sleep();
                                    float joint_state_back[] = {0.05, 0, -1.57, -1.57, 0};
                                    if (!this->move_arm(joint_state_back))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(move_goal);
                                    ros::Duration(0.5).sleep();
                                }
                                else
                                {
                                    ROS_INFO("The object is not liftable, now re-plan the path.");
                                    // leave obstacle and put arm back
                                    gc_.release();
                                    ros::Duration(1).sleep();
                                    geometry_msgs::Twist tw;
                                    tw.linear.x = -1;
                                    velocity_pub_.publish(tw);
                                    ros::Duration(1).sleep();
                                    tw.linear.x = 0;
                                    velocity_pub_.publish(tw);
                                    ros::Duration(1).sleep();
                                    float joint_state_back[] = {0.05, 0, -1.57, -1.57, 0};
                                    gc_.grab();
                                    ros::Duration(0.5).sleep();
                                    if (!this->move_arm(joint_state_back))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(move_goal);
                                    ros::Duration(0.5).sleep();
                                }
                            }
                            else
                                ROS_INFO("Failed to Validate the Affordance Candidate");
                        }
                        else if (current_primitive.type == 1)
                        {
                            ROS_INFO("Planer detected");
                            validate_goal.center = current_primitive.center;
                            validate_goal.normal = current_primitive.normal;
                            validate_goal.primitive_type = current_primitive.type;
                            validate_goal.validate_type = 1;
                            affordance_validate_client_.sendGoal(validate_goal);
                            affordance_validate_client_.waitForResult();
                            if ((affordance_validate_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
                            {
                                jackal_affordance::ValidateResult validate_result = *affordance_validate_client_.getResult();
                                if (validate_result.result)
                                {
                                    ROS_INFO("The obstacle is movable, now moving the obstacle to clear the path");
                                    geometry_msgs::Twist tw;
                                    tw.linear.x = 1;
                                    velocity_pub_.publish(tw);
                                    ros::Duration(1).sleep();
                                    tw.linear.x = 0;
                                    velocity_pub_.publish(tw);
                                    ros::Duration(0.5).sleep();

                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(move_goal);
                                    ros::Duration(0.5).sleep();
                                }
                                else
                                {
                                    ROS_INFO("The obstacle is not movable, now re-plan the path");
                                    float joint_state_back[] = {0.05, 0, -1.57, -1.57, 0};
                                    if (!this->move_arm(joint_state_back))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }
                                    
                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(move_goal);
                                    ros::Duration(0.5).sleep();
                                }
                            }
                        }
                    }
                    else
                        ROS_INFO("No Affordance Detected");
                }
                break;
            }
            ros::spinOnce();
        }
        ROS_INFO("Arrival!");
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber trajectory_sub_, dynamic_map_sub_, robot_pose_sub_, path_sub_;
    ros::Publisher cancel_movebase_pub_, initial_trajectory_pub_, velocity_pub_;
    ros::ServiceClient affordance_detect_client_;
    AffordanceValidate affordance_validate_client_;

    nav_msgs::Path initial_trajectory_;
    nav_msgs::OccupancyGrid dynamic_map_;

    std::string trajectory_topic_, dynamic_map_topic_, movebase_cancel_topic_, fixed_frame_, move_base_topic_;
    float robot_x_, robot_y_, inflation_radius_;
    double traj_coordinate_[2];

    GripperMSG gc_;
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

    //ros::spin();

    return 0;
}