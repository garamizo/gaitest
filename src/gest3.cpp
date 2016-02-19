#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_ros/point_cloud.h"

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <pcl/filters/crop_box.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
ros::Publisher pub, pubCoef;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  // pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::CropBox<PointT> cropFilter;
  pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>);

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud_blob);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.setFilterFieldName("z");
  // sor.setFilterLimits(0.15, 0.4);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud);

  // Build a passthrough filter to remove spurious NaNs
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.15, 0.6);
  // pass.filter (*cloud_filtered);

  // Crop filter
  Eigen::Vector4f minPoint(-0.3, 0, 0.15, 0);
  // minPoint[0]= -0.2;  // define minimum point x
  // minPoint[1]= -0.2;  // define minimum point y
  // minPoint[2]= 0.15;  // define minimum point z
  Eigen::Vector4f maxPoint(0.3, 0.2, 0.4, 0);
  // maxPoint[0]= 0.2;  // define max point x
  // maxPoint[1]= 0;  // define max point y
  // maxPoint[2]= 0.4;  // define max point z 

  cropFilter.setInputCloud (cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.filter (*cloud_cropped); 

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_cropped);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.3);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMinMaxOpeningAngle(60, 110);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_cropped);
  seg.setInputNormals (cloud_normals);
  Eigen::Vector3f upVector(0, 0, 1);
  seg.setAxis(upVector);
  seg.setEpsAngle(1.5708);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_cropped);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  extract.filter (*cloud_plane);

  pub.publish (*cloud_plane);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*coefficients_plane, ros_coefficients);
  pubCoef.publish (ros_coefficients);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("leap_motion/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PCLCloud> ("/output", 1);

  pubCoef = nh.advertise<pcl_msgs::ModelCoefficients> ("/coef", 1);

  // Spin
  ros::spin ();
}