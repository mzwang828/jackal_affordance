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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "jackal_affordance/Primitive.h"
#include "jackal_affordance/AffordanceDetect.h"

typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

class AffordanceDetect
{
  public:
    AffordanceDetect(ros::NodeHandle n) : nh_(n), cloud_transformed_(new pcl::PointCloud<PointT>), cloud_filtered_(new pcl::PointCloud<PointT>), cloud_hull_(new pcl::PointCloud<PointT>), tree_(new pcl::search::KdTree<PointT>())
    {
        nh_.getParam("info_topic", info_topic_);
        nh_.getParam("rgb_topic", rgb_topic_);
        nh_.getParam("point_cloud_topic", point_cloud_topic_);
        nh_.getParam("fixed_frame", fixed_frame_);
        nh_.getParam("min_primitive_size", min_primitive_size_);
        nh_.getParam("max_plane_area", max_plane_area_);
        nh_.getParam("min_seg_size", min_seg_size_);
        nh_.getParam("seed_resolution", seed_res_);
        nh_.getParam("voxel_resolution", voxel_res_);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("affordance_visual_marking", 10);
        //info_sub_ = nh_.subscribe(info_topic_, 10, &AffordanceDetect::info_callback, this);
        rgb_sub_ = nh_.subscribe(rgb_topic_, 10, &AffordanceDetect::rgb_callback, this);
        point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &AffordanceDetect::point_cloud_callback, this);
        affordance_detection_srv_ = nh_.advertiseService("affordance_detect", &AffordanceDetect::affordance_detect_callback, this);
        lccp_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lccp_cloud", 10);
        primitive_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("primitive_cloud", 10);
    }

    void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        model1_.fromCameraInfo(msg);
    }

    void rgb_callback(const sensor_msgs::ImageConstPtr &msg)
    {
    }

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        cloud_raw_ = *msg;
    }

    bool point_cloud_transform()
    // transform the point cloud to target frame, /map
    {
        cloud_transformed_->clear();
        filter_frame_ = "/base_link";
        pcl::PointCloud<PointT>::Ptr cloud_raw_pcl(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(cloud_raw_, *cloud_raw_pcl);
        std::string source_frame = cloud_raw_.header.frame_id;
        tf::TransformListener listener;

        // transform to base link frame to filter first
        listener.waitForTransform(filter_frame_, source_frame, ros::Time(0), ros::Duration(10.0));
        tf::StampedTransform transform;
        Eigen::Affine3d eigen_transform;
        try
        {
            listener.lookupTransform(filter_frame_, source_frame, ros::Time(0), transform);
            tf::transformTFToEigen(transform, eigen_transform);
            pcl::transformPointCloud(*cloud_raw_pcl, *cloud_filtered_, eigen_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        // filter
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0, 5);
        pass.filter(*cloud_filtered_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, 100);
        pass.filter(*cloud_filtered_);

        cloud_transformed_ = cloud_filtered_;

        return true;

        // //transform to fxied frame for future computation
        // listener.waitForTransform(fixed_frame_, filter_frame, ros::Time(0), ros::Duration(10.0));
        // try
        // {
        //     listener.lookupTransform(fixed_frame_, filter_frame, ros::Time(0), transform);
        //     // listener.lookupTransform(filter_frame, fixed_frame_, ros::Time(0), transform);
        //     tf::transformTFToEigen(transform, eigen_transform);
        //     pcl::transformPointCloud(*cloud_filtered_, *cloud_transformed_, eigen_transform);
        //     std::cout << "Point cloud transformed";
        //     return true;
        // }
        // catch (tf::TransformException ex)
        // {
        //     ROS_ERROR("%s", ex.what());
        //     return false;
        // }
    }

    int maximum(int a, int b)
    {
        int max = (b < a) ? 0 : 1;
        return max;
    }

    bool primitive_segmentation(std::vector<jackal_affordance::Primitive> &primitives, pcl::PointCloud<PointT>::Ptr cloud_input)
    {
        // remove outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_noise;
        sor_noise.setInputCloud(cloud_input);
        sor_noise.setMeanK(50);
        sor_noise.setStddevMulThresh(1.0);
        sor_noise.filter(*cloud_input);
        // find all primitives from given point cloud input
        int initial_size = cloud_input->size();
        pcl::PointCloud<PointT>::Ptr cloud_primitive_raw(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_primitive(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        // Estimate point normals
        ne_.setSearchMethod(tree_);
        ne_.setInputCloud(cloud_input);
        ne_.setKSearch(50);
        ne_.compute(*cloud_normals);
        // Create the segmentation object
        // Optional
        seg_.setOptimizeCoefficients(true);
        seg_.setMaxIterations(1000); // iteration limits decides segmentation goodness
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setDistanceThreshold(0.02);
        seg_.setNormalDistanceWeight(0.1);
        int no_primitives = 0;
        // record primitives' center to prevent overlap, first number used to record if major primitive detected
        float major_center[4] = {0, 0, 0, 0};
        while (true)
        {
            // coefficients & inliers for cylinder, vertical plane and horizontal plane
            jackal_affordance::Primitive primitive_msg;
            pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients), coefficients_plane(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices), inliers_plane(new pcl::PointIndices);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg_.setInputCloud(cloud_input);
            seg_.setInputNormals(cloud_normals);
            // Obtain the plane inliers and coefficients
            seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            seg_.segment(*inliers_plane, *coefficients_plane);
            // Obtain the cylinder inliers and coefficients
            seg_.setModelType(pcl::SACMODEL_CYLINDER);
            seg_.setNormalDistanceWeight(0.1);
            seg_.setMaxIterations(10000);
            seg_.setDistanceThreshold(0.05);
            seg_.setRadiusLimits(0, 0.1);
            // Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
            // seg_.setAxis(axis);
            seg_.segment(*inliers_cylinder, *coefficients_cylinder);

            //ROS_INFO("vertical plane %d, horizontal %d, cylinder %d", inliers_vplane->indices.size(), inliers_hplane ->indices.size(), inliers_cylinder->indices.size());
            ROS_INFO("plane %d, cylinder %d", inliers_plane->indices.size(), inliers_cylinder->indices.size());

            int best = this->maximum(inliers_plane->indices.size(), inliers_cylinder->indices.size());

            //ROS_INFO("best %d", best);

            switch (best)
            {
            //verticle plane
            case 0:
                inliers = inliers_plane;
                coefficients = coefficients_plane;
                break;
            //cylinder
            case 1:
                inliers = inliers_cylinder;
                coefficients = coefficients_cylinder;
                break;
            }

            if (inliers->indices.size() == 0 and no_primitives == 0)
            {
                std::cout << "PS: no primitive found!!!" << std::endl;
                return false;
            }
            else if ((inliers->indices.size() < min_primitive_size_) || (float(inliers->indices.size()) / float((initial_size)) < 0.2))
            {
                ROS_INFO("Inliers doesn't meet requirement: inliers: %d, initial: %d", inliers->indices.size(), initial_size);
                break;
            }
            // generate the primitive msg
            extract_.setInputCloud(cloud_input);
            extract_.setNegative(false);
            extract_.setIndices(inliers);
            extract_.filter(*cloud_primitive_raw);

            // transform primitive points to fixed frame
            tf::TransformListener listener;
            Eigen::Affine3d eigen_transform;
            tf::StampedTransform transform;
            listener.waitForTransform(fixed_frame_, filter_frame_, ros::Time(0), ros::Duration(10.0));
            try
            {
                listener.lookupTransform(fixed_frame_, filter_frame_, ros::Time(0), transform);
                tf::transformTFToEigen(transform, eigen_transform);
                pcl::transformPointCloud(*cloud_primitive_raw, *cloud_primitive, eigen_transform);
                std::cout << "Point cloud transformed \n";
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                return false;
            }

            chull_.setInputCloud(cloud_primitive);
            //chull_.setDimension(2);
            chull_.reconstruct(*cloud_hull);
            Eigen::Vector4f center;
            pcl::compute3DCentroid(*cloud_hull, center);

            Eigen::Vector4f min_vals, max_vals;
            pcl::getMinMax3D(*cloud_primitive, min_vals, max_vals);

            // Get cloud
            pcl::toROSMsg(*cloud_primitive, primitive_msg.cloud);

            pcl_conversions::fromPCL(cloud_primitive->header, primitive_msg.header);

            // Get primitive center
            primitive_msg.center.x = center[0];
            primitive_msg.center.y = center[1];
            primitive_msg.center.z = center[2];

            // compare center distance to avoid overlap TODO:improve overlap detection
            if (major_center[0] == 0)
            {
                major_center[0] = 1;
                major_center[1] = primitive_msg.center.x;
                major_center[2] = primitive_msg.center.y;
                major_center[3] = primitive_msg.center.z;
            }
            else
            {
                float distance = sqrt((primitive_msg.center.x - major_center[1]) * (primitive_msg.center.x - major_center[1]) +
                                      (primitive_msg.center.y - major_center[2]) * (primitive_msg.center.y - major_center[2]) +
                                      (primitive_msg.center.z - major_center[3]) * (primitive_msg.center.z - major_center[3]));
                ROS_INFO("distance %f", distance);
                if (distance < 0.1)
                    break;
            }

            // Get primitive min and max values
            primitive_msg.min.x = min_vals[0];
            primitive_msg.min.y = min_vals[1];
            primitive_msg.min.z = min_vals[2];

            primitive_msg.max.x = max_vals[0];
            primitive_msg.max.y = max_vals[1];
            primitive_msg.max.z = max_vals[2];
            // Get primitive polygon
            for (int i = 0; i < cloud_hull->points.size(); i++)
            {
                geometry_msgs::Point32 p;
                p.x = cloud_hull->points[i].x;
                p.y = cloud_hull->points[i].y;
                p.z = cloud_hull->points[i].z;
                primitive_msg.polygon.push_back(p);
            }

            for (int i = 0; i < coefficients->values.size(); i++)
            {
                primitive_msg.coef.push_back(coefficients->values[i]);
            }
            // Get primitive coefficients
            if (best == 0)
            {
                // plane
                primitive_msg.type = 1;
                float height, length;
                // Get plane normal
                float mag = sqrt(coefficients->values[0] * coefficients->values[0] +
                                 coefficients->values[1] * coefficients->values[1] +
                                 coefficients->values[2] * coefficients->values[2]);
                primitive_msg.normal[0] = coefficients->values[0] / mag;
                primitive_msg.normal[1] = coefficients->values[1] / mag;
                primitive_msg.normal[2] = coefficients->values[2] / mag;
                primitive_msg.size.data = cloud_primitive->points.size();

                if (primitive_msg.normal[2] < 0.5)
                {
                    primitive_msg.is_vertical = true;
                    height = primitive_msg.max.z - primitive_msg.min.z;
                    length = sqrt((primitive_msg.max.x - primitive_msg.min.x) * (primitive_msg.max.x - primitive_msg.min.x) +
                                  (primitive_msg.max.y - primitive_msg.min.y) * (primitive_msg.max.y - primitive_msg.min.y));
                }
                else
                {
                    primitive_msg.is_vertical = false;
                    height = primitive_msg.max.x - primitive_msg.min.x;
                    length = primitive_msg.max.y - primitive_msg.min.y;
                }
                float plane_area = height * length;
                ROS_INFO("Plane_area: %f, height: %f, length: %f", plane_area, height, length);
                if ((plane_area < max_plane_area_) && ((height / length) < 10) && ((length / height) < 10))
                {
                    primitives.push_back(primitive_msg);
                    no_primitives++;
                }
                else
                {
                    ROS_INFO("Plane size doesn't meet requirement.\n");
                }
            }

            else if (best == 1)
            {
                // cylinder
                primitive_msg.type = 3;

                // Get cylinder normal
                primitive_msg.normal[0] = coefficients->values[3];
                primitive_msg.normal[1] = coefficients->values[4];
                primitive_msg.normal[2] = coefficients->values[5];

                ROS_INFO("cylinder normal %f, %f, %f", primitive_msg.normal[0], primitive_msg.normal[1], primitive_msg.normal[2]);
                primitive_msg.size.data = cloud_primitive->points.size();
                // TODO: check horizontal cylinder in addition to vertical cylinder
                primitive_msg.is_vertical = true;
                primitives.push_back(primitive_msg);
                no_primitives++;
            }

            // remve primitive clouds
            extract_.setNegative(true);
            extract_.filter(*cloud_input);
            extract_normals_.setNegative(true);
            extract_normals_.setInputCloud(cloud_normals);
            extract_normals_.setIndices(inliers);
            extract_normals_.filter(*cloud_normals);
            ros::Duration(0.2).sleep();
        }
        std::cout << "PS: " << no_primitives << ". primitives segmented!" << std::endl;
        return true;
    }

    void marker_publish(jackal_affordance::Primitive marker_to_be_pub, int id)
    {
        // get marker dimension
        float height, length;
        if (marker_to_be_pub.is_vertical)
        {
            height = marker_to_be_pub.max.z - marker_to_be_pub.min.z;
            length = sqrt((marker_to_be_pub.max.x - marker_to_be_pub.min.x) * (marker_to_be_pub.max.x - marker_to_be_pub.min.x) +
                          (marker_to_be_pub.max.y - marker_to_be_pub.min.y) * (marker_to_be_pub.max.y - marker_to_be_pub.min.y));
        }
        else
        {
            height = marker_to_be_pub.max.x - marker_to_be_pub.min.x;
            length = marker_to_be_pub.max.y - marker_to_be_pub.min.y;
        }
        //get marker orientation
        geometry_msgs::Quaternion orientation;
        orientation = this->calculate_quaternion(marker_to_be_pub);
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = marker_to_be_pub.type;
        marker.action = visualization_msgs::Marker::ADD;
        // create a tf transform to translate the face of cylinder to detected position
        marker.pose.position.x = marker_to_be_pub.center.x;
        marker.pose.position.y = marker_to_be_pub.center.y;
        marker.pose.position.z = marker_to_be_pub.center.z;
        marker.pose.orientation = orientation;
        if (marker.type == 1)
        {
            marker.scale.x = 0.02;
            marker.scale.y = length;
            marker.scale.z = height;
        }
        else if (marker.type == 3)
        {
            marker.scale.x = marker_to_be_pub.coef[6] * 2;
            marker.scale.y = marker_to_be_pub.coef[6] * 2;
            marker.scale.z = height;
        }
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.65;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker_pub_.publish(marker);
        //ROS_INFO("position; %f,%f,%f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    }

    // calculate the quaternion rotation between two vector, up_vector and axis_vector
    geometry_msgs::Quaternion calculate_quaternion(jackal_affordance::Primitive Primitive)
    {
        tf::Vector3 axis_vector(Primitive.normal[0], Primitive.normal[1], Primitive.normal[2]);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
        if (Primitive.type == 3)
        {
            up_vector.setX(0.0);
            up_vector.setY(0.0);
            up_vector.setZ(1.0);
        }
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion cylinder_orientation;
        tf::quaternionTFToMsg(q, cylinder_orientation);
        return cylinder_orientation;
    }

    bool primitive_extract()
    {
        primitives_.resize(0);
        // remove outlier
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_noise;
        sor_noise.setInputCloud(cloud_transformed_);
        sor_noise.setMeanK(50);
        sor_noise.setStddevMulThresh(1.0);
        sor_noise.filter(*cloud_transformed_);
        // down sampling the point cloud
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(cloud_transformed_);
        sor.setLeafSize(0.005f, 0.005f, 0.005f);
        sor.filter(*cloud_transformed_);
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
        uint32_t min_segment_size = 100;
        bool use_extended_convexity = false;
        bool use_sanity_criterion = true;
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

            ROS_INFO("temp cloud size: %d", cloud_temp->size());

            pcl::toROSMsg(*cloud_temp, lccp_labeled_cloud_);
            lccp_labeled_cloud_.header.frame_id = filter_frame_;
            lccp_cloud_pub_.publish(lccp_labeled_cloud_);

            // getchar();

            //Find Primitives in one of Segmentations
            if (cloud_temp->size() > min_seg_size_)
            {
                this->primitive_segmentation(primitives_, cloud_temp);
            }

            for (int i = 0; i < primitives_.size(); i++)
            {
                this->marker_publish(primitives_[i], i);
                // primitives_[i].cloud.header.frame_id = fixed_frame_;
                // primitive_cloud_pub_.publish(primitives_[i].cloud);
            }
        }

        //pcl::toROSMsg(*ColoredCloud2, lccp_labeled_cloud_);
        //lccp_labeled_cloud_.header.frame_id = fixed_frame_;
        //lccp_cloud_pub_.publish(lccp_labeled_cloud_);
        if (primitives_.size() != 0)
        {
            std::cout << "LCCP: # of input point cloud: " << lccp_labeled_cloud->size() << ", # of segmentations: " << label_max << std::endl;

            std::cout << "AD: # of primitve found: " << primitives_.size() << std::endl;
            return true;
        }
        else
            return false;
    }

    bool affordance_detect_callback(jackal_affordance::AffordanceDetect::Request &req,
                                    jackal_affordance::AffordanceDetect::Response &res)
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
        res.primitives = primitives_;
        res.success = true;
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_, rgb_sub_, point_cloud_sub_;
    ros::Publisher marker_pub_, lccp_cloud_pub_, primitive_cloud_pub_;
    ros::ServiceServer affordance_detection_srv_;
    pcl::PointCloud<PointT>::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_;
    sensor_msgs::PointCloud2 cloud_raw_, lccp_labeled_cloud_;
    image_geometry::PinholeCameraModel model1_;

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ExtractIndices<pcl::Normal> extract_normals_;
    pcl::ConvexHull<PointT> chull_;
    pcl::search::KdTree<PointT>::Ptr tree_;
    pcl::NormalEstimation<PointT, pcl::Normal> ne_;

    int min_primitive_size_, min_seg_size_, max_plane_area_;
    float seed_res_, voxel_res_;
    std::string info_topic_, rgb_topic_, point_cloud_topic_, fixed_frame_, filter_frame_;
    std::vector<jackal_affordance::Primitive> primitives_;
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
