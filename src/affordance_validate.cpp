#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "tf/transform_datatypes.h"

#include "jackal_affordance/ValidateAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class AffordanceValidate
{
  public:
    AffordanceValidate(ros::NodeHandle n, std::string name) : nh_(n), validate_server_(n, name, boost::bind(&AffordanceValidate::executeCB, this, _1), false),
                                            action_name_(name)
    {
        nh_.getParam("info_topic", info_topic_);
        nh_.getParam("point_cloud_topic", point_cloud_topic_);
        nh_.getParam("depth_topic", depth_topic_);
        nh_.getParam("force_sensor_topic", force_sensor_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);
        nh_.getParam("force_max", force_max_);
        nh_.getParam("force_min", force_min_);
        nh_.getParam("velocity_topic", velocity_topic_); 
        nh_.getParam("arm_trajectory_topic", arm_trajectory_topic_); 

        //point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &AffordanceValidate::point_cloud_callback, this);
        //depth_sub_ = nh_.subscribe(depth_topic_, 10, &AffordanceValidate::depth_callback, this);
        // affordance_validate_srv_ = nh_.advertiseService("affordance_validate", &AffordanceValidate::affordance_validate_callback, this);
        hand_force_sub_ = nh_.subscribe(force_sensor_topic_, 10, &AffordanceValidate::force_callback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic_, 10);
        validate_server_.start();
    }

    void force_callback(const geometry_msgs::WrenchStamped &msg)
    {
        force_[0] = msg.wrench.force.x;
        force_[1] = msg.wrench.force.y;
        force_[2] = msg.wrench.force.z; // force along the arm direction

        // ROS_INFO("Force : %f", force_[2]);

        if ((validating_ == 1) && (force_[2] > force_max_))
        {
            geometry_msgs::Twist tw;
            tw.linear.x = 0.0;
            tw.linear.y = 0.0;
            tw.linear.z = 0.0;
            tw.angular.x = 0.0;
            tw.angular.y = 0.0;
            tw.angular.z = 0.0;
            velocity_pub_.publish(tw);
            movable_ = 0;
        }
    }

    bool move_arm(float joint_state[5])
    {
        TrajectoryClient tc(arm_trajectory_topic_, true);
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

    ///////////////////////////////////////////////////////////////////////////////////////
    // service
    ///////////////////////////////////////////////////////////////////////////////////////
    // bool affordance_validate_callback(jackal_affordance::AffordanceValidate::Request &req,
    //                                   jackal_affordance::AffordanceValidate::Response &res)
    // {
    //     // first go to the position of obstacle
    //     validating_ = 1;
    //     MoveBaseClient ac("/move_base/move", true);

    //     move_base_msgs::MoveBaseGoal move_goal;
    //     move_goal.target_pose.header.frame_id = fixed_frame_;
    //     move_goal.target_pose.header.stamp = ros::Time::now();

    //     move_goal.target_pose.pose.position.x = req.center.x - req.normal[0];
    //     move_goal.target_pose.pose.position.y = req.center.y - req.normal[1];
    //     double normal[3] = {req.normal[0], req.normal[1], req.normal[2]};
    //     move_goal.target_pose.pose.orientation = this->calculate_quaternion(normal);

    //     while (!ac.waitForServer(ros::Duration(5.0)))
    //     {
    //         ROS_INFO("Waiting for the move_base action server to come up");
    //     }

    //     ROS_INFO("Sending goal");
    //     ac.sendGoal(move_goal);
    //     ac.waitForResult();

    //     if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         // straight arm
    //         ROS_INFO("Straighting the arm");
    //         float joint_state[] = {0.044, -0.23, -0.00127, -1.241, 0.000};
    //         if (!this->move_arm(joint_state))
    //         {
    //             ROS_INFO("Failed to move arm straight");
    //         }
    //         ros::Duration(0.5).sleep();
    //         float force_at_start = force_[2];
    //         ROS_INFO("Initial Force: %f", force_at_start);
    //         // contact with obstacle to validate affordance
    //         geometry_msgs::Twist tw;
    //         tw.linear.x = 0.3;
    //         while ((force_[2] - force_at_start) < force_min_)
    //         {
    //             velocity_pub_.publish(tw);
    //             ros::spinOnce();
    //             ROS_INFO("Force difference: %f", force_[2]-force_at_start);
    //         }
    //         ROS_INFO("waaaaaaaaaaa");
    //         tw.linear.x = 0.0;
    //         velocity_pub_.publish(tw);
    //         ros::Duration(1).sleep();

    //         tw.linear.x = 0.3;
    //         velocity_pub_.publish(tw);
    //     }

    //     res.movable = movable_;
    //     res.success = 1;
    //     ros::Duration(1).sleep();
    //     float joint_state[] = {0, -0.024, 0.0016, -0.2288, -0.0015};
    //     if (!this->move_arm(joint_state))
    //     {
    //         ROS_INFO("Failed to move arm back");
    //     }
    //     validating_ = 0;
    //     return true;
    // }


    //////////////////////////////////////////////////////////////////////////////
    // actionlib
    //////////////////////////////////////////////////////////////////////////////

    void executeCB(const jackal_affordance::ValidateGoalConstPtr &goal)
    {
        ros::Rate r(1);
        if (validate_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            validate_server_.setPreempted();
            return;
        }

        bool success = true;
        validating_ = 1;
        // first go to the position of obstacle
        MoveBaseClient ac("/move_base/move", true);

        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = fixed_frame_;
        move_goal.target_pose.header.stamp = ros::Time::now();

        move_goal.target_pose.pose.position.x = goal->center.x - goal->normal[0];
        move_goal.target_pose.pose.position.y = goal->center.y - goal->normal[1];
        double normal[3] = {goal->normal[0], goal->normal[1], goal->normal[2]};
        move_goal.target_pose.pose.orientation = this->calculate_quaternion(normal);

        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        ROS_INFO("Sending goal");
        ac.sendGoal(move_goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // straight arm
            ROS_INFO("Straighting the arm");
            float joint_state[] = {0.044, -0.23, -0.00127, -1.241, 0.000};
            if (!this->move_arm(joint_state))
            {
                ROS_INFO("Failed to move arm straight");
            }
            ros::Duration(0.5).sleep();
            float force_at_start = force_[2];
            ROS_INFO("Initial Force: %f", force_at_start);
            // contact with obstacle to validate affordance
            geometry_msgs::Twist tw;
            tw.linear.x = 0.3;
            while ((force_[2] - force_at_start) < force_min_)
            {
                velocity_pub_.publish(tw);
                ROS_INFO("Force difference: %f", force_[2] - force_at_start);
            }
            ROS_INFO("waaaaaaaaaaa");
            tw.linear.x = 0.0;
            velocity_pub_.publish(tw);
            ros::Duration(1).sleep();

            tw.linear.x = 0.3;
            velocity_pub_.publish(tw);
        }

        ros::Duration(1).sleep();
        float joint_state[] = {0, -0.024, 0.0016, -0.2288, -0.0015};
        if (!this->move_arm(joint_state))
        {
            ROS_INFO("Failed to move arm back");
            return;
        }

        result_.movable = movable_;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        validate_server_.setSucceeded(result_);

        validating_ = 0;
    }

    // calculate the quaternion rotation between two vector, up_vector and axis_vector
    geometry_msgs::Quaternion calculate_quaternion(double normal[3])
    {
        tf::Vector3 axis_vector(normal[0], normal[1], 0);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion orientation_quaternion;
        tf::quaternionTFToMsg(q, orientation_quaternion);
        return orientation_quaternion;
    }

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<jackal_affordance::ValidateAction> validate_server_;
    // ros::ServiceServer affordance_validate_srv_;
    ros::Subscriber hand_force_sub_, depth_sub_, point_cloud_sub_;
    ros::Publisher velocity_pub_;

    float force_[3] = {}; //hand force x, y, z

    std::string info_topic_, point_cloud_topic_, depth_topic_, force_sensor_topic_, fixed_frame_, action_name_, arm_trajectory_topic_, velocity_topic_;
    int validating_ = 0, movable_ = 1;
    int force_max_, force_min_;
    jackal_affordance::ValidateFeedback feedback_;
    jackal_affordance::ValidateResult result_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "affordance_validate");
    ros::NodeHandle n("~");

    AffordanceValidate affordance_validate(n, "affordance_validate");
    ROS_INFO("Affordance Validate Service Initialized");

    ros::spin();

    return 0;
}