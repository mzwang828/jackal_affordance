#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>

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

class myClass
{
  public:
    myClass(ros::NodeHandle n) : nh_(n), cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pub = nh_.advertise<sensor_msgs::PointCloud2>("/verified_obstacle_clouds", 10);
        sub = nh_.subscribe("/hsr_namo_planner/temp_verified_obstacle_clouds", 10, &myClass::point_cloud_callback, this);
    }

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        cloud_raw_ = *msg;

        cloud_transformed_->clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_raw_, *cloud_raw_pcl);
        std::string source_frame = cloud_raw_.header.frame_id;
        ROS_INFO("OK1");
        tf::TransformListener listener;
        listener.waitForTransform("head_rgbd_sensor_link", source_frame, ros::Time(0), ros::Duration(10.0));
        tf::StampedTransform transform;
        tf::Transform tf_transform;
        Eigen::Affine3d eigen_transform;
        ROS_INFO("OK2");
        try
        {
            listener.lookupTransform("head_rgbd_sensor_link", source_frame, ros::Time(0), transform);
            tf_transform.setOrigin(transform.getOrigin());
            tf_transform.setRotation(transform.getRotation());
            tf::transformTFToEigen(transform, eigen_transform);
            pcl::transformPointCloud(*cloud_raw_pcl, *cloud_transformed_, eigen_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO("OK3");
    }

    void publishData()
    {
        //ROS_INFO("OK4");
        pcl::toROSMsg(*cloud_transformed_, cloud_pub_);
        cloud_pub_.header.frame_id = "head_rgbd_sensor_link";
        pub.publish(cloud_pub_);
    }

  private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 cloud_raw_, cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "verified_cloud_pub");
    ros::NodeHandle n("~");
    myClass myObject(n);
    ros::Rate rate(10);
    while (ros::ok())
    {
        myObject.publishData();
        ros::spinOnce();
        rate.sleep();
    }
}