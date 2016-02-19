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

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

ros::Publisher pub, pubCoef;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud_blob);

  // Perform the actual filtering
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.05, 0.05, 0.05);
  sor.setFilterFieldName("z");
  sor.setFilterLimits(0.01, 0.3);
  sor.filter (*cloud_filtered_blob);

  // // Remove outlier X
  // pcl::PCLPointCloud2::Ptr cloud_filtered_blobx (new pcl::PCLPointCloud2);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sorx;
  // sorx.setInputCloud(cloud_filtered_blobz);
  // sorx.setFilterFieldName("x");
  // sorx.setFilterLimits(-1, 1);
  // sorx.filter(*cloud_filtered_blobx);

  // // Remove outlier Y
  // pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sory;
  // sory.setInputCloud(cloud_filtered_blobx);
  // sory.setFilterFieldName("y");
  // sory.setFilterLimits(-1, 1);
  // sory.filter(*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  Eigen::Vector3f upVector(0, 0, 1);
  seg.setAxis(upVector);
  seg.setEpsAngle(1.5708);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);

  // Publish inliers
  // sensor_msgs::PointCloud2 inlierpc;
  // pcl_conversions::fromPCL(cloud_p, inlierpc);
  pub.publish (*cloud_p);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  pubCoef.publish (ros_coefficients);

    // ==========================================

  // // Convert to ROS data type
  // sensor_msgs::PointCloud2 downpc;
  // pcl_conversions::fromPCL(cloud_filtered, downpc);

  // // Publish the data
  // pub.publish (downpc);


  // pcl::ModelCoefficients coefficients;
  // pcl::PointIndices inliers;

  // //.makeShared()
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // seg.setInputCloud (&cloudPtr);
  // seg.setOptimizeCoefficients (true); // Optional
  // seg.setModelType (pcl::SACMODEL_PLANE); // Mandatory
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.1);

  // seg.segment (inliers, coefficients);

  // if (inliers.indices.size () == 0)
  // {
  //   PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  //   return;
  // }

  // std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
  //                                     << coefficients.values[1] << " "
  //                                     << coefficients.values[2] << " " 
  //                                     << coefficients.values[3] << std::endl;

  // // Publish the model coefficients
  // pcl_msgs::ModelCoefficients ros_coefficients;
  // pcl_conversions::fromPCL(coefficients, ros_coefficients);
  // pubCoef.publish (ros_coefficients);
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