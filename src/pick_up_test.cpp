#include <iostream>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>

#include "jackal_affordance/ValidateAction.h"
#include "jackal_affordance/AffordanceDetect.h"
#include "jackal_affordance/Primitive.h"

typedef actionlib::SimpleActionClient<jackal_affordance::ValidateAction> AffordanceValidate;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickUpTest
{
  public:
    PickUpTest(ros::NodeHandle n) : nh_(n), affordance_validate_client_("/hsr_affordance_validate/affordance_validate", true)
    {
        //robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &PickUpTest::pose_callback, this);
        affordance_detect_client_ = nh_.serviceClient<jackal_affordance::AffordanceDetect>("/hsr_affordance_detect/affordance_detect");
    }

    // void pose_callback(const geometry_msgs::Pose &msg)
    // {
    //     ROS_INFO("aaa");
    //     robot_x_ = msg.position.x;
    //     robot_y_ = msg.position.y;
    // }

    void doit()
    {
        jackal_affordance::AffordanceDetect detect_srv;
        jackal_affordance::ValidateGoal validate_goal;
        // call affordance detect
        if (affordance_detect_client_.call(detect_srv))
        {
            // check if successful
            if (detect_srv.response.success)
            {
                ROS_INFO("Affordance Detect Successfully, find %lu affordance candidate(s)", detect_srv.response.primitives.size());
                // iterate for each detected affordance
                for (int i = 0; i < detect_srv.response.primitives.size(); i++)
                {
                    jackal_affordance::Primitive current_primitive = detect_srv.response.primitives[i];
                    // for cylinders
                    if (current_primitive.type == 3)
                    {
                        ROS_INFO("Cylinder Detected");
                        // calcualte directional vector between robot and cylinder
                        // get robot current position
                        geometry_msgs::Pose robot_pose;
                        boost::shared_ptr<geometry_msgs::Pose const> sharedPtr;
                        sharedPtr = ros::topic::waitForMessage<geometry_msgs::Pose>("/robot_pose", ros::Duration(10));
                        if (sharedPtr == NULL)
                            ROS_INFO("No robot pose msg received");
                        else
                            robot_pose = *sharedPtr;
                        float distance_x = current_primitive.center.x - robot_pose.position.x;
                        float distance_y = current_primitive.center.y - robot_pose.position.y;
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
                                ROS_INFO("The object is liftable.");
                            }
                            else
                            {
                                ROS_INFO("The object is not liftable");
                            }
                        }
                        else
                            ROS_INFO("Failed to Validate the Affordance Candidate");
                    }
                    else if (current_primitive.type == 1)
                    {
                        ROS_INFO("Planer detected");
                    }
                }
            }
            else
                ROS_INFO("No Affordance Detected");
        }
        else
            ROS_INFO("Failed call affordance detect.");
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient affordance_detect_client_;
    AffordanceValidate affordance_validate_client_;
    ros::Subscriber robot_pose_sub_;

    float robot_x_, robot_y_;
};

main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_up_test");

    ros::NodeHandle n("~");
    PickUpTest putest(n);
    ROS_INFO("Start Testing");

    putest.doit();

    ros::spin();

    return 0;
}