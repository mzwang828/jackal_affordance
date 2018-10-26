#include <iostream>
#include <math.h>
#include <numeric>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <frasier_utilities/gripper.h>
#include <boost/circular_buffer.hpp>

#include "tf/transform_datatypes.h"
#include "jackal_affordance/ValidateAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class AffordanceValidate
{
  public:
    AffordanceValidate(ros::NodeHandle n, std::string name) : nh_(n), gc_(n, true), validate_server_(n, name, boost::bind(&AffordanceValidate::executeCB, this, _1), false),
                                                              action_name_(name), torque_buffer_x_(200)
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
        hand_force_sub_ = nh_.subscribe(force_sensor_topic_, 10, &AffordanceValidate::force_callback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic_, 10);
        validate_server_.start();
    }

    void force_callback(const geometry_msgs::WrenchStamped &msg)
    {
        force_[0] = msg.wrench.force.x;
        force_[1] = msg.wrench.force.y;
        force_[2] = msg.wrench.force.z; // force along the arm direction

        torque_[0] = msg.wrench.torque.x;
        torque_[1] = msg.wrench.torque.y;
        torque_[2] = msg.wrench.torque.z; // force along the arm direction

        torque_buffer_x_.push_back(torque_[0]);
        // ROS_INFO("Force : %f", force_[2]);
        // if found object is not movable, stop the move immediately
        if ((validating_ == true) && (force_[2] > force_max_))
        {
            geometry_msgs::Twist tw;
            tw.linear.x = 0.0;
            tw.linear.y = 0.0;
            tw.linear.z = 0.0;
            tw.angular.x = 0.0;
            tw.angular.y = 0.0;
            tw.angular.z = 0.0;
            velocity_pub_.publish(tw);
            result_ = false;
            ROS_INFO("too large force %f", force_[2]);
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

    float force_difference(float force1[3], float force2[3])
    {
        return sqrt((force1[0] - force2[0]) * (force1[0] - force2[0]) +
                    (force1[1] - force2[1]) * (force1[1] - force2[1]) +
                    (force1[2] - force2[2]) * (force1[2] - force2[2]));
    }

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

        // first go to the position of obstacle

        // with tmc functions/controllers
        //MoveBaseClient ac("/move_base/move", true);
        // with pure ros move base
        MoveBaseClient ac("/move_base", true);

        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = fixed_frame_;
        move_goal.target_pose.header.stamp = ros::Time::now();
        // normal vector here: for plane, normal vector is normal to the plane; for cylinder, normal vector is the
        // vector poinitng from robot to the center of cylinder.
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
            switch (goal->validate_type)
            {
            case 1:
            {
                //validate pushability
                // TODO: adjust arm position
                validating_ = 1;
                ROS_INFO("Straighting the arm");
                float joint_state_a[] = {0.040, -0.23, -0.00127, -1.241, 0.000};
                if (!this->move_arm(joint_state_a))
                {
                    ROS_INFO("Failed to move arm to right position");
                }
                ros::Duration(0.5).sleep();
                // contact with obstacle to validate affordance
                geometry_msgs::Twist tw;
                tw.linear.x = 0.1;
                float force_at_start = force_[2];
                velocity_pub_.publish(tw);
                ros::Duration(0.3).sleep();
                ROS_INFO("Initial Force: %f", force_at_start);
                while ((force_[2] - force_at_start) < force_min_)
                {
                    velocity_pub_.publish(tw);
                    ROS_INFO("Force difference: %f", force_[2] - force_at_start);
                }
                tw.linear.x = 0.0;
                velocity_pub_.publish(tw);
                ros::Duration(1).sleep();

                tw.linear.x = 0.1;
                ros::Time endTime = ros::Time::now() + ros::Duration(1);
                while (ros::Time::now() < endTime)
                {
                    velocity_pub_.publish(tw);
                }

                tw.linear.x = 0.0;
                velocity_pub_.publish(tw);
                ros::Duration(1).sleep();

                break;
            }
            case 2:
            {
                //validate liftability
                // TODO: adjust arm position
                ROS_INFO("Straighting the arm");
                float joint_state_b[] = {0.06, -2.62, 0.02, 1.06, -0.02};
                if (!this->move_arm(joint_state_b))
                {
                    ROS_INFO("Failed to move arm to right position");
                }
                ros::Duration(0.5).sleep();
                gc_.release();
                // go forward till contact with object
                geometry_msgs::Twist tw;
                tw.linear.x = 0.1;
                float force_at_start[3] = {force_[0], force_[1], force_[2]};
                velocity_pub_.publish(tw);
                ros::Duration(0.3).sleep();
                while ((force_[2] - force_at_start[2]) < force_min_)
                {
                    velocity_pub_.publish(tw);
                    ROS_INFO("Force difference: %f", force_[2] - force_at_start[2]);
                }
                tw.linear.x = 0.0;
                velocity_pub_.publish(tw);
                ros::Duration(1).sleep();
                // pick up object
                gc_.grab();
                // lift arm slightly to check liftibility
                float joint_state_c[] = {0.12, -2.62, 0.02, 1.06, -0.02};
                if (!this->move_arm(joint_state_c))
                {
                    ROS_INFO("Failed to move arm up");
                    result_ = false;
                }
                ros::Duration(6).sleep();
                // TODO: check force/torque to determine liftability
                //float force_x_avg = (std::accumulate(force_buffer_x_.begin(), force_buffer_x_.end(), 0)) / (float)force_buffer_x_.size();
                float torque_x_avg = (std::accumulate(torque_buffer_x_.begin(), torque_buffer_x_.end(), 0.0)) / (float)torque_buffer_x_.size();
                //float weight = force_x_avg-force_at_start[0];
                ROS_INFO("torque %f", torque_x_avg);
                if (torque_x_avg > 2)
                {
                    result_ = false;
                    ROS_INFO("Non-Liftable Object");
                }
                else
                    ROS_INFO("Liftable Object");
                // put arm down
                this->move_arm(joint_state_b);
                /*
                gc_.release();
                ros::Duration(1).sleep();
                */
            }
            }
        }
        else
        {
            ROS_INFO("Failed to move to obstacle position");
            return;
        }
        /*
        // move arm back to original position
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
        ros::Duration(1).sleep();
        if (!this->move_arm(joint_state_back))
        {
            ROS_INFO("Failed to move arm back");
            return;
        }
        */
        action_result_.result = result_;
        ROS_INFO("%s: Action Executed Succeeded", action_name_.c_str());
        // set the action state to succeeded
        validate_server_.setSucceeded(action_result_);

        validating_ = false;

        return;
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
    ros::Subscriber hand_force_sub_, depth_sub_, point_cloud_sub_;
    ros::Publisher velocity_pub_;

    float force_[3] = {}; //hand force x, y, z
    float torque_[3] = {};

    std::string info_topic_, point_cloud_topic_, depth_topic_, force_sensor_topic_, fixed_frame_, action_name_, arm_trajectory_topic_, velocity_topic_;
    bool validating_ = false, result_ = true;
    int force_max_, force_min_;
    jackal_affordance::ValidateFeedback feedback_;
    jackal_affordance::ValidateResult action_result_;
    boost::circular_buffer<float> torque_buffer_x_;

    GripperMSG gc_;
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