#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//include filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
//include SAC
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//include others
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <vector> 

class ObjectCluster{

private:
  ros::Subscriber pc_sub;
  ros::NodeHandle nh;
  ros::Publisher pc_pub;

public:
  explicit ObjectCluster(ros::NodeHandle& n):
    nh(n) {
    pc_sub = nh.subscribe("/hokuyo3d/hokuyo_cloud2",1,
			  &ObjectCluster::Clustering, this);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud2", 1);
    ROS_INFO("starting clustering.");
  }
  
  void Clustering(const sensor_msgs::PointCloud2ConstPtr& input){
    /** convert sensor_msgs to pcl */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::fromROSMsg(*input, *cloud);

    /** remove invalid points*/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    /** perform downsampling for performance */
    pcl::VoxelGrid<pcl::PointXYZ> sor_down;
    sor_down.setInputCloud(cloud->makeShared());
    sor_down.setLeafSize(0.02, 0.02, 0.02);
    sor_down.filter(*cloud);

    /** remove outliers by statical filtering */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
    sor_stat.setInputCloud(cloud->makeShared());
    sor_stat.setMeanK(50);
    sor_stat.setStddevMulThresh(1.0);
    sor_stat.setNegative(false);
    sor_stat.filter(*cloud);
    
    //include SAC
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.3 * nr_points)
      {
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	  {
	    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	    break;
	  }
	
	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Get the points associated with the planar surface
	extract.filter (*cloud_plane);
	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud);
      }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_list;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	  cloud_cluster->points.push_back (cloud->points[*pit]); //*
	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	cloud_cluster_list.push_back(cloud_cluster);
	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      }
    for(size_t i = 0; i < cloud_cluster_list.size(); i++){
      sensor_msgs::PointCloud2 filter_cloud;
      pcl::toROSMsg(*cloud, filter_cloud);
      pc_pub.publish(filter_cloud);
    }
  }
};



