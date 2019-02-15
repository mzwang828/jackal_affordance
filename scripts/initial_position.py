#!/usr/bin/env python
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize ROS publisher
arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                         trajectory_msgs.msg.JointTrajectory, queue_size=10)
head_pub = rospy.Publisher('/hsrb/head_trajectory_controller/command',
                          trajectory_msgs.msg.JointTrajectory, queue_size=10)
# wait to establish connection between the controller
while arm_pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = (
    rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                       controller_manager_msgs.srv.ListControllers))
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'arm_trajectory_controller' and c.state == 'running':
            running = True

# arm
arm_traj = trajectory_msgs.msg.JointTrajectory()
arm_traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0.28, -2.62, -1.56, -1.57, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Time(3)
arm_traj.points = [p]
arm_pub.publish(arm_traj)

# head
head_traj = trajectory_msgs.msg.JointTrajectory()
head_traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
p.positions = [0.0, -0.56]
p.velocities = [0.0, 0.0]
p.time_from_start = rospy.Time(3)
head_traj.points = [p]
head_pub.publish(head_traj)
