#include <iostream>
#include <math.h>
#include <random>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "jackal_affordance/Plane.h"
#include "jackal_affordance/AffordanceDetection.h"

typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

class AffordanceDetect
{
  public:
    AffordanceDetect(ros::NodeHandle n) : nh_(n), cloud_transformed_(new pcl::PointCloud<PointT>), cloud_hull_(new pcl::PointCloud<PointT>)
    {
        nh_.getParam("info_topic", info_topic_);
        nh_.getParam("rgb_topic", rgb_topic_);
        nh_.getParam("point_cloud_topic", point_cloud_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);
        nh_.getParam("min_plane_size", min_plane_size_);
        nh_.getParam("min_seg_size", min_seg_size_);
        nh_.getParam("seed_resolution", seed_res_);
        nh_.getParam("voxel_resolution", voxel_res_);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("affordance_visual_marking", 10);
        //info_sub_ = nh_.subscribe(info_topic_, 10, &AffordanceDetect::info_callback, this);
        rgb_sub_ = nh_.subscribe(rgb_topic_, 10, &AffordanceDetect::rgb_callback, this);
        point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &AffordanceDetect::point_cloud_callback, this);
        affordance_detection_srv_ = nh_.advertiseService("affordance_detect", &AffordanceDetect::affordance_detect_callback, this);

        debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 10);
    }

    void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        model1_.fromCameraInfo(msg);
    }

    void rgb_callback(const sensor_msgs::ImageConstPtr &msg)
    {
    }

    bool planar_segmentation(std::vector<jackal_affordance::Plane> &Planes, pcl::PointCloud<PointT>::Ptr cloud_input, char orientation)
    {
        pcl::PointCloud<PointT> plane_clouds;
        plane_clouds.header.frame_id = fixed_frame_;
        jackal_affordance::Plane plane_object_msg;

        pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
        // Create the segmentation object
        // Optional
        seg_.setOptimizeCoefficients(true);
        // Mandatory set plane to be parallel to Z axis within a 10 degrees tolerance
        Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0); //z axis
        seg_.setAxis(axis);
        seg_.setMaxIterations(500); // iteration limits decides segmentation goodness
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setEpsAngle(pcl::deg2rad(10.0f));
        seg_.setDistanceThreshold(0.01);
        if (orientation == 'v')
        {
            seg_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        }
        else
        {
            seg_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        }
        int no_planes = 1;
        while (true)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg_.setInputCloud(cloud_input);
            seg_.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0 and no_planes == 0)
            {
                std::cout << "PCP: no plane found!!!" << std::endl;
                return false;
            }
            else if (inliers->indices.size() < min_plane_size_)
            {
                break;
            }
            extract_.setInputCloud(cloud_input);
            extract_.setNegative(false);
            extract_.setIndices(inliers);
            extract_.filter(*cloud_plane);

            plane_clouds += *cloud_plane;

            chull_.setInputCloud(cloud_plane);
            chull_.setDimension(2);
            chull_.reconstruct(*cloud_hull);
            Eigen::Vector4f center;
            pcl::compute3DCentroid(*cloud_hull, center);

            Eigen::Vector4f min_vals, max_vals;
            pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);

            // Get cloud
            pcl::toROSMsg(*cloud_plane, plane_object_msg.cloud);

            // Construct plane object msg
            pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

            // Get plane center
            plane_object_msg.center.x = center[0];
            plane_object_msg.center.y = center[1];
            plane_object_msg.center.z = center[2];

            // Get plane min and max values
            plane_object_msg.min.x = min_vals[0];
            plane_object_msg.min.y = min_vals[1];
            plane_object_msg.min.z = min_vals[2];

            plane_object_msg.max.x = max_vals[0];
            plane_object_msg.max.y = max_vals[1];
            plane_object_msg.max.z = max_vals[2];

            // Get plane polygon
            for (int i = 0; i < cloud_hull->points.size(); i++)
            {
                geometry_msgs::Point32 p;
                p.x = cloud_hull->points[i].x;
                p.y = cloud_hull->points[i].y;
                p.z = cloud_hull->points[i].z;
                plane_object_msg.polygon.push_back(p);
            }

            // Get plane coefficients
            plane_object_msg.coef[0] = coefficients->values[0];
            plane_object_msg.coef[1] = coefficients->values[1];
            plane_object_msg.coef[2] = coefficients->values[2];
            plane_object_msg.coef[3] = coefficients->values[3];

            // Get plane normal
            float length = sqrt(coefficients->values[0] * coefficients->values[0] +
                                coefficients->values[1] * coefficients->values[1] +
                                coefficients->values[2] * coefficients->values[2]);
            plane_object_msg.normal[0] = coefficients->values[0] / length;
            plane_object_msg.normal[1] = coefficients->values[1] / length;
            plane_object_msg.normal[2] = coefficients->values[2] / length;

            plane_object_msg.size.data = cloud_plane->points.size();
            if (orientation == 'v')
                plane_object_msg.is_vertical = true;
            else
                plane_object_msg.is_vertical = false;

            std::cout << "PCP: " << no_planes << ". plane segmented! # of points: "
                      << inliers->indices.size() << " axis: " << axis << std::endl;
            no_planes++;

            Planes.push_back(plane_object_msg);
            extract_.setNegative(true);
            extract_.filter(*cloud_input);

            ros::Duration(0.2).sleep();
        }

        return true;
    }

    void marker_publish(jackal_affordance::Plane marker_to_be_pub, int id)
    {
        // get cube dimension
        float height = marker_to_be_pub.max.z - marker_to_be_pub.min.z;
        float length = sqrt((marker_to_be_pub.max.x - marker_to_be_pub.min.x) * (marker_to_be_pub.max.x - marker_to_be_pub.min.x) +
                            (marker_to_be_pub.max.y - marker_to_be_pub.min.y) * (marker_to_be_pub.max.y - marker_to_be_pub.min.y));

        //get cube orientation
        geometry_msgs::Quaternion orientation;
        orientation = this->calculate_quaternion(marker_to_be_pub);
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        // create a tf transform to translate the face of cylinder to detected position
        marker.pose.position.x = marker_to_be_pub.center.x;
        marker.pose.position.y = marker_to_be_pub.center.y;
        marker.pose.position.z = marker_to_be_pub.center.z;
        marker.pose.orientation = orientation;
        marker.scale.x = 0.02;
        marker.scale.y = length;
        marker.scale.z = height;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker_pub_.publish(marker);
        //ROS_INFO("position; %f,%f,%f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    }

    // calculate the quaternion rotation between two vector, up_vector and axis_vector
    geometry_msgs::Quaternion calculate_quaternion(jackal_affordance::Plane plane)
    {
        tf::Vector3 axis_vector(plane.normal[0], plane.normal[1], plane.normal[2]);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion cylinder_orientation;
        tf::quaternionTFToMsg(q, cylinder_orientation);
        return cylinder_orientation;
    }

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        cloud_raw_ = *msg;
    }

    bool point_cloud_transform()
    {
        cloud_transformed_->clear();

        pcl::PointCloud<PointT>::Ptr cloud_raw_pcl(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(cloud_raw_, *cloud_raw_pcl);
        std::string source_frame = cloud_raw_.header.frame_id;
        tf::TransformListener listener;
        listener.waitForTransform(fixed_frame_, source_frame, ros::Time(0), ros::Duration(10.0));
        tf::StampedTransform transform;
        tf::Transform tf_transform;
        Eigen::Affine3d eigen_transform;
        try
        {
            listener.lookupTransform(fixed_frame_, source_frame, ros::Time(0), transform);
            tf_transform.setOrigin(transform.getOrigin());
            tf_transform.setRotation(transform.getRotation());
            tf::transformTFToEigen(transform, eigen_transform);
            pcl::transformPointCloud(*cloud_raw_pcl, *cloud_transformed_, eigen_transform);
            std::cout << "Point cloud transformed";
            return true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }

    bool primitive_extract()
    {
        planes_.resize(0);
        // TODO: filter point cloud

        //LCCP Segmentation: SuperVoxel + LCCP
        //STEP1: 超体聚类 set parameters
        //voxel_resolution is the resolution (in meters) of voxels used、
        //seed_resolution is the average size (in meters) of resulting supervoxels
        float voxel_resolution = voxel_res_;
        float seed_resolution = seed_res_;
        float color_importance = 0.0f;
        float spatial_importance = 1.0f;
        float normal_importance = 4.0f;
        bool use_single_cam_transform = false;
        bool use_supervoxel_refinement = false;
        unsigned int k_factor = 0;
        // Preparation of Input: Perform Supervoxel Oversegmentation
        pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
        super.setUseSingleCameraTransform(use_single_cam_transform);
        super.setInputCloud(cloud_transformed_);
        //Set the importance of color for supervoxels.
        super.setColorImportance(color_importance);
        //Set the importance of spatial distance for supervoxels.
        super.setSpatialImportance(spatial_importance);
        //Set the importance of scalar normal product for supervoxels.
        super.setNormalImportance(normal_importance);
        std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
        super.extract(supervoxel_clusters);
        std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);
        // Get the cloud of supervoxel centroid with normals and the
        //colored cloud with supervoxel coloring (this is used for visulization)
        pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);
        // STEP2: LCCPSegmentation
        // Set parameters
        float concavity_tolerance_threshold = 10;
        float smoothness_threshold = 0.1;
        uint32_t min_segment_size = 0;
        bool use_extended_convexity = false;
        bool use_sanity_criterion = false;
        // Perform LCCP Segmentation
        pcl::LCCPSegmentation<PointT> lccp;
        lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
        lccp.setSanityCheck(use_sanity_criterion);
        lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
        lccp.setKFactor(k_factor);
        lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
        lccp.setMinSegmentSize(min_segment_size);
        lccp.segment();
        // Interpolation voxel cloud -> input cloud and relabeling
        pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
        pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
        lccp.relabelCloud(*lccp_labeled_cloud);
        SuperVoxelAdjacencyList sv_adjacency_list;
        lccp.getSVAdjacencyList(sv_adjacency_list); // Needed for visualization
        //retrieve each segmentation and perform pirmitive extraction, also color the segmentations for RViz
        int label_max = 0; //count numbers of segmentations
        for (int i = 0; i < lccp_labeled_cloud->size(); i++)
        {
            if (lccp_labeled_cloud->points[i].label > label_max)
                label_max = lccp_labeled_cloud->points[i].label;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        ColoredCloud2->height = 1;
        ColoredCloud2->width = lccp_labeled_cloud->size();
        ColoredCloud2->resize(lccp_labeled_cloud->size());
        for (int i = 0; i <= label_max; i++)
        {
            const int range_from = 0;
            const int range_to = 255;
            std::random_device rand_dev;
            std::mt19937 generator(rand_dev());
            std::uniform_int_distribution<int> distr(range_from, range_to);

            int color_R = distr(generator);
            int color_G = distr(generator);
            int color_B = distr(generator);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
            for (int j = 0; j < lccp_labeled_cloud->size(); j++)
            {
                if (lccp_labeled_cloud->points[j].label == i)
                {
                    ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
                    ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
                    ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
                    ColoredCloud2->points[j].r = color_R;
                    ColoredCloud2->points[j].g = color_G;
                    ColoredCloud2->points[j].b = color_B;

                    pcl::PointXYZ temp;
                    temp.x = lccp_labeled_cloud->points[j].x;
                    temp.y = lccp_labeled_cloud->points[j].y;
                    temp.z = lccp_labeled_cloud->points[j].z;
                    cloud_temp->push_back(temp);
                }
            }
            ROS_INFO("cloud_temp size: %d", cloud_temp->size());
            if (cloud_temp->size() > min_seg_size_)
                this->planar_segmentation(planes_, cloud_temp, 'v');
            //Find Primitives in Segmentations
            for (int i = 0; i < planes_.size(); i++)
            {
                this->marker_publish(planes_[i], i);
            }
        }

        std::cout << "LCCP: # of input point cloud: " << lccp_labeled_cloud->size() << ", # of segmentations: " << label_max << std::endl;
        pcl::toROSMsg(*ColoredCloud2, lccp_labeled_cloud_);
        lccp_labeled_cloud_.header.frame_id = fixed_frame_;
        debug_cloud_pub_.publish(lccp_labeled_cloud_);

        std::cout << "AD: # of planes found: " << planes_.size() << std::endl;
        return true;
    }

    bool affordance_detect_callback(jackal_affordance::AffordanceDetection::Request &req,
                                    jackal_affordance::AffordanceDetection::Response &res)
    {
        if (!point_cloud_transform())
        {
            std::cout << "AD: couldn't transform point cloud!" << std::endl;
            return false;
        }
        if (!primitive_extract())
        {
            std::cout << "AD: couldn't extract primitive!" << std::endl;
            return false;
        }
        res.success = true;
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_, rgb_sub_, point_cloud_sub_;
    ros::Publisher marker_pub_, debug_cloud_pub_;
    ros::ServiceServer affordance_detection_srv_;
    pcl::PointCloud<PointT>::Ptr cloud_transformed_, cloud_hull_;
    sensor_msgs::PointCloud2 cloud_raw_, lccp_labeled_cloud_;
    image_geometry::PinholeCameraModel model1_;

    pcl::SACSegmentation<PointT> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ConvexHull<PointT> chull_;

    int min_plane_size_, min_seg_size_;
    float seed_res_, voxel_res_;
    std::string info_topic_, rgb_topic_, point_cloud_topic_, fixed_frame_;
    std::vector<jackal_affordance::Plane> planes_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "jackal_affordance_detect");
    ros::NodeHandle n("~");

    AffordanceDetect affordance_detect(n);
    ROS_INFO("Affordance Detection Service Initialized");

    ros::spin();

    return 0;
}
