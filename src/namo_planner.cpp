#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <numeric>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

// arm relative
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
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
    NamoPlanner(ros::NodeHandle n) : nh_(n), affordance_validate_client_("/hsr_affordance_validate/affordance_validate/", true), gc_(n, true), joint_state_init_({0.05, 0, -1.57, -1.57, 0})
    {
        nh_.getParam("trajectory_topic", trajectory_topic_);
        nh_.getParam("local_map_topic", local_map_topic_);
        nh_.getParam("global_map_topic", global_map_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);
        nh_.getParam("move_base_topic", move_base_topic_);
        nh_.getParam("/move_base/local_costmap/inflater/inflation_radius", inflation_radius_);

        std::string move_base_goal_topic = move_base_topic_ + "/goal";
        std::string move_base_cancel_topic = move_base_topic_ + "/cancel";

        local_map_sub_ = nh_.subscribe(local_map_topic_, 1, &NamoPlanner::local_map_callback, this);
        global_map_sub_ = nh_.subscribe(global_map_topic_, 1, &NamoPlanner::global_map_callback, this);
        robot_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &NamoPlanner::pose_callback, this);
        move_base_goal_sub_ = nh_.subscribe(move_base_goal_topic, 1, &NamoPlanner::move_base_goal_callback, this);
        cancel_movebase_pub_ = nh_.advertise<actionlib_msgs::GoalID>(move_base_cancel_topic, 1);
        affordance_detect_client_ = nh_.serviceClient<jackal_affordance::AffordanceDetect>("/hsr_affordance_detect/affordance_detect");
        initial_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("initial_trajectory", 1);
        path_sub_ = nh_.subscribe(trajectory_topic_, 1, &NamoPlanner::path_callback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/hsrb/opt_command_velocity", 10);
        non_movable_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("verified_obstacle_clouds", 10);
    }

    void local_map_callback(const nav_msgs::OccupancyGrid &msg)
    {
        local_map_ = msg;
    }

    void global_map_callback(const nav_msgs::OccupancyGrid &msg)
    {
        global_map_ = msg;
    }

    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
    {
        //ROS_INFO("update robot pose");
        robot_x_ = msg.pose.pose.position.x;
        robot_y_ = msg.pose.pose.position.y;
    }

    void path_callback(const nav_msgs::Path &msg)
    {
        //ROS_INFO("update path");
        initial_trajectory_ = msg;
    }

    void move_base_goal_callback(const move_base_msgs::MoveBaseActionGoal &msg)
    {
        this->go_namo(msg.goal);
    }

    bool move_arm(double joint_state[5])
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

    sensor_msgs::PointCloud2 cloud_transform(sensor_msgs::PointCloud2 cloud_raw, std::string source_frame, std::string target_frame)
    {
        sensor_msgs::PointCloud2 cloud_pub;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_raw, *cloud_raw_pcl);
        tf::TransformListener listener;
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
        tf::StampedTransform transform;
        tf::Transform tf_transform;
        Eigen::Affine3d eigen_transform;
        try
        {
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
            tf_transform.setOrigin(transform.getOrigin());
            tf_transform.setRotation(transform.getRotation());
            tf::transformTFToEigen(transform, eigen_transform);
            pcl::transformPointCloud(*cloud_raw_pcl, *cloud_transformed, eigen_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        pcl::toROSMsg(*cloud_transformed, cloud_pub);
        return cloud_pub;
    }

    // given position on /map frame, return of occupancy of that position on dynmaic_map
    int get_occupancy(double position[2], nav_msgs::OccupancyGrid map)
    {
        int grid_x = (int)((position[0] - map.info.origin.position.x) / map.info.resolution);
        int grid_y = (int)((position[1] - map.info.origin.position.y) / map.info.resolution);
        int index = grid_y * map.info.width + grid_x;

        // ROS_INFO("grid x %d, grid y %d, x %f, y %f, origin x %f, origin y %f", grid_x, grid_y, position[0], position[1], map.info.origin.position.x, map.info.origin.position.y);

        if ((grid_x >= 0) && (grid_y >= 0) && (grid_x < map.info.width) && (grid_y < map.info.height))
        {
            int occupancy = map.data[index];
            return occupancy;
        }
        else
            return 0;
    }

    void move_straight(float speed_x, float speed_y, float time)
    {
        geometry_msgs::Twist tw;
        tw.linear.x = speed_x;
        tw.linear.y = speed_y;
        ros::Time endTime = ros::Time::now() + ros::Duration(time);
        while (ros::Time::now() < endTime)
        {
            velocity_pub_.publish(tw);
        }
        ros::Duration(0.5).sleep();
    }
    // iterate all detected afffordances to find the one block the way
    int which_primitive(double position[2], std::vector<jackal_affordance::Primitive> primitives)
    {
        //TODO: finding the real obstacle when there are offsets
        ROS_INFO("radius: %f", inflation_radius_);
        for (int i = 0; i < primitives.size(); i++)
        {
            jackal_affordance::Primitive current_primitive = primitives[i];
            if (!current_primitive.is_vertical)
                continue;
            // check if primitive is static obstacle on global map
            double primitive_position[2] = {current_primitive.center.x, current_primitive.center.y};
            if (get_occupancy(primitive_position, global_map_) > 50)
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
                // DEBUG: check distance
                ROS_INFO("distance from cylinder center %f, inflation area %f", distance_radius, current_primitive.coef[6] + inflation_radius_);
                if (distance_radius < (current_primitive.coef[6] + inflation_radius_))
                {
                    ROS_INFO("Found obstacle primitive (Cylinder)");
                    return i;
                }
            }
        }
        ROS_INFO("NAMO Error: Couldn't find obstacle primitive");
        return 10086;
    }

    // add pushable object on map to generate geometry information needed for path optimization
    void planmap_generate(jackal_affordance::Primitive object, nav_msgs::OccupancyGrid map)
    {
        /*
        int size_x = map.info.width;
        int size_y = map.info.height;
        cv_bridge::CvImage cv_img_full;
        cv_img_full.header.frame_id = "map_image";
        cv_img_full.encoding = sensor_msgs::image_encodings::MONO8;
        cv::Mat *map_mat = &cv_img_full.image;
        *map_mat = cv::Mat(size_x, size_y, CV_8U);
        const std::vector<int8_t> &map_data(map->data);

        unsigned char *map_mat_data_p = (unsigned char *)map_mat.data;

        //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
        int size_y_rev = size_y - 1;

        for (int y = size_y_rev; y >= 0; --y)
        {

            int idx_map_y = size_x * (size_y - y);
            int idx_img_y = size_x * y;

            for (int x = 0; x < size_x; ++x)
            {

                int idx = idx_img_y + x;

                switch (map_data[idx_map_y + x])
                {
                case -1:
                    map_mat_data_p[idx] = 127;
                    break;

                case 0:
                    map_mat_data_p[idx] = 255;
                    break;

                case 100:
                    map_mat_data_p[idx] = 0;
                    break;
                }
            }
        }
    */
    }

    // accept goal, navigate while moving obstacles
    void go_namo(move_base_msgs::MoveBaseGoal goal)
    {
        // receive the move base goal
        MoveBaseClient mc(move_base_topic_, true);
        // move_base_msgs::MoveBaseGoal move_goal;
        // move_goal.target_pose.header.frame_id = fixed_frame_;
        // move_goal.target_pose.header.stamp = ros::Time::now();
        // move_goal.target_pose.pose.position.x = goal_position[0];
        // move_goal.target_pose.pose.position.y = goal_position[1];
        // move_goal.target_pose.pose.orientation.w = 1.0;
        // ROS_INFO("move_base goal %f, %f ", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.y);
        // while (!mc.waitForServer(ros::Duration(5.0)))
        // {
        //     ROS_INFO("Waiting for the move_base action server to come up");
        // }
        ROS_INFO("NAMO received goal");
        // mc.sendGoal(move_goal);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        int namo_state = 1;
        actionlib_msgs::GoalID empty_goal;

        while ((abs(robot_x_ - goal.target_pose.pose.position.x) > 0.01) || (abs(robot_y_ - goal.target_pose.pose.position.y) > 0.01))
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
                    ros::spinOnce();
                    // get each coordinate of the trajectory and check its occupancy
                    traj_coordinate_[0] = initial_trajectory_.poses[i].pose.position.x;
                    traj_coordinate_[1] = initial_trajectory_.poses[i].pose.position.y;
                    int occupancy = this->get_occupancy(traj_coordinate_, local_map_);
                    if (occupancy > 96)
                    {
                        // ROS_INFO("traj coor, %f, %f, occupancy %d, ", traj_coordinate_[0], traj_coordinate_[1], occupancy);
                        cancel_movebase_pub_.publish(empty_goal);
                        // wait for point cloud to update
                        ros::Duration(1).sleep();
                        namo_state = 2;
                        break;
                    }
                }
                break;

            case 2:
                jackal_affordance::AffordanceDetect detect_srv;
                jackal_affordance::ValidateGoal validate_goal;
                ROS_INFO("Trajectory blocked, now calculating if can clear the path");
                if (affordance_detect_client_.call(detect_srv))
                {
                    // check if successful
                    if (detect_srv.response.success)
                    {
                        ROS_INFO("Affordance Detect Successfully, find %lu affordance candidate(s)", detect_srv.response.primitives.size());
                        // find the primitive that blocks the trajectory
                        int primitive_index = this->which_primitive(traj_coordinate_, detect_srv.response.primitives);
                        // if no obstacle primitive found, restart whole process
                        if (primitive_index == 10086)
                        {
                            ros::Duration(0.5).sleep();
                            ROS_INFO("Restart NAMO...");
                            namo_state = 1;
                            ROS_INFO("Sending goal");
                            mc.sendGoal(goal);
                            ros::Duration(0.5).sleep();
                            break;
                        }
                        jackal_affordance::Primitive current_primitive = detect_srv.response.primitives[primitive_index];
                        // print obstacle info
                        std::cout << "\033[1;32mObstacle index is: \033[0m" << current_primitive.object_index << std::endl;
                        jackal_affordance::Object obstacle = detect_srv.response.objects[current_primitive.object_index];
                        std::cout << "Obstacle center is: " << obstacle.object_center.x << ", " << obstacle.object_center.y << "\n";
                        std::cout << "Polygon: \n";
                        for (int i = 0; i < obstacle.object_polygon.size(); i++){
                            std::cout << "(" << obstacle.object_polygon[i].x << ", " << obstacle.object_polygon[i].y << ")\n";
                        }
                        // for cylinders
                        if (current_primitive.type == 3)
                        {
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
                                if (validate_result.result == 2)
                                {
                                    ROS_INFO("The object is liftable. Pick up to continue.");
                                    //TODO: Pick up obstacle
                                    ros::Duration(1).sleep();
                                    // float joint_state_back[] = {0.15, -1.56, 0.09, -1.35, -0.3};
                                    if (!this->move_arm(joint_state_init_))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    namo_state = 1;
                                    ros::Duration(1).sleep();
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(goal);
                                    ros::Duration(0.5).sleep();
                                }
                                else if (validate_result.result == 1)
                                {
                                    ROS_INFO("The object is pushable.");
                                    //TODO: push;
                                }
                                else
                                {
                                    ROS_INFO("The object is not movable, now replan the path.");
                                    // publish non-movable clouds for mapping purpose
                                    non_movable_point_cloud_ = this->cloud_transform(current_primitive.cloud, "map", "head_rgbd_sensor_link");
                                    non_movable_point_cloud_.header.frame_id = "head_rgbd_sensor_link";
                                    ros::Time endTime = ros::Time::now() + ros::Duration(5);
                                    while (ros::Time::now() < endTime)
                                    {
                                        non_movable_point_cloud_pub_.publish(non_movable_point_cloud_);
                                        // ros::spinOnce();
                                        ros::Duration(0.1).sleep();
                                    }
                                    // leave obstacle and put arm back
                                    gc_.release();
                                    ros::Duration(1).sleep();
                                    this->move_straight(-1, 0, 2);
                                    this->move_straight(0, 0, 1);
                                    // float joint_state_back[] = {0.15, -1.56, 0.09, -1.35, -0.3};
                                    gc_.grab();
                                    if (!this->move_arm(joint_state_init_))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    namo_state = 1;
                                    ros::Duration(1).sleep();
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(goal);
                                    ros::Duration(0.5).sleep();
                                }
                            }
                            else
                                ROS_INFO("Failed to Validate the Affordance Candidate");
                        }
                        // find planer
                        else if (current_primitive.type == 1)
                        {
                            validate_goal.center = current_primitive.center;
                            validate_goal.normal = current_primitive.normal;
                            validate_goal.primitive_type = current_primitive.type;
                            validate_goal.validate_type = 1;
                            affordance_validate_client_.sendGoal(validate_goal);
                            affordance_validate_client_.waitForResult();
                            if ((affordance_validate_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
                            {
                                jackal_affordance::ValidateResult validate_result = *affordance_validate_client_.getResult();
                                if (validate_result.result == 1)
                                {
                                    ROS_INFO("The obstacle is movable, now moving the obstacle to clear the path");
                                    getchar();
                                    
                                    /*
                                    float length = sqrt((current_primitive.max.x - current_primitive.min.x) * (current_primitive.max.x - current_primitive.min.x) +
                                                        (current_primitive.max.y - current_primitive.min.y) * (current_primitive.max.y - current_primitive.min.y));

                                    if (!this->move_arm(joint_state_init_))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    this->move_straight(-1, 0, 1);
                                    this->move_straight(-0.5, 1, 2 * (length + 1));
                                    this->move_straight(1, 0, 3);

                                    double joint_state_straight[] = {0.10, -2.20, 0.04, 0, 1.43};

                                    if (!this->move_arm(joint_state_straight))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    this->move_straight(-0.5, -1, 4 * (length + 1));
                                    this->move_straight(-0.5, 1, 2 * (length + 1));
                                    this->move_straight(0, 0, 1);

                                    if (!this->move_arm(joint_state_init_))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(goal);
                                    ros::Duration(0.5).sleep();

                                    // this->move_straight(1, 0, 2);
                                    // this->move_straight(0, 0, 1);
                                    // ros::Duration(0.5).sleep();
                                    // this->move_straight(-1, 0, 2);
                                    // this->move_straight(0, 0, 1);
                                    // // float joint_state_back[] = {0.15, -1.56, 0.09, -1.35, -0.3};
                                    // if (!this->move_arm(joint_state_init_))
                                    // {
                                    //     ROS_INFO("Failed to move arm back");
                                    //     return;
                                    // }
                                    // namo_state = 1;
                                    // ROS_INFO("Sending goal");
                                    // mc.sendGoal(goal);
                                    // ros::Duration(0.5).sleep();
                                    */
                                }
                                else
                                {
                                    ROS_INFO("The obstacle is not movable, now re-plan the path");
                                    non_movable_point_cloud_ = this->cloud_transform(current_primitive.cloud, "map", "head_rgbd_sensor_link");
                                    non_movable_point_cloud_.header.frame_id = "head_rgbd_sensor_link";
                                    ros::Time endTime = ros::Time::now() + ros::Duration(5);
                                    while (ros::Time::now() < endTime)
                                    {
                                        non_movable_point_cloud_pub_.publish(non_movable_point_cloud_);
                                        // ros::spinOnce();
                                        ros::Duration(0.1).sleep();
                                    }

                                    // float joint_state_back[] = {0.15, -1.56, 0.09, -1.35, -0.3};
                                    if (!this->move_arm(joint_state_init_))
                                    {
                                        ROS_INFO("Failed to move arm back");
                                        return;
                                    }

                                    this->move_straight(-1, 0, 2);
                                    this->move_straight(0, 0, 1);

                                    namo_state = 1;
                                    ROS_INFO("Sending goal");
                                    mc.sendGoal(goal);
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
            // ros::spinOnce();
        }
        ROS_INFO("Arrival!");
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber trajectory_sub_, local_map_sub_, global_map_sub_, robot_pose_sub_, path_sub_, move_base_goal_sub_;
    ros::Publisher cancel_movebase_pub_, initial_trajectory_pub_, velocity_pub_, non_movable_point_cloud_pub_;
    ros::ServiceClient affordance_detect_client_;
    AffordanceValidate affordance_validate_client_;

    nav_msgs::Path initial_trajectory_;
    nav_msgs::OccupancyGrid local_map_, global_map_;

    pcl::PCLPointCloud2 primitive_cloud_, all_cloud_;
    sensor_msgs::PointCloud2 non_movable_point_cloud_;

    std::string trajectory_topic_, local_map_topic_, global_map_topic_, fixed_frame_, move_base_topic_;
    float robot_x_, robot_y_, inflation_radius_;
    double traj_coordinate_[2], joint_state_init_[5];

    Gripper gc_;
};

main(int argc, char *argv[])
{
    // if (argc < 3)
    //     return -1;

    ros::init(argc, argv, "namo_planner");
    ros::NodeHandle n("~");
    NamoPlanner namo_planner(n);
    ROS_INFO("NAMO Planner Initialized");
    // double goal[2] = {atof(argv[1]), atof(argv[2])};
    // namo_planner.go_namo(goal);
    ros::spin();
    return 0;
}