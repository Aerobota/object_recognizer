#include "ObjectCluster.h"

class LIDARObjectCluster : virtual private ObjectClusterInterface{

private:
  ros::Subscriber pc_sub;
  ros::NodeHandle nh;
  ros::Publisher pc_pub;

public:
  explicit LIDARObjectCluster(ros::NodeHandle& n):
    nh(n) {
    pc_sub = nh.subscribe("/hokuyo3d/hokuyo_cloud2",1,
			  &LIDARObjectCluster::Clustering, this);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud2", 1);
    //ROS_INFO("starting clustering.");
  }
  
  void Clustering(const sensor_msgs::PointCloud2::Ptr &input){
    /** convert sensor_msgs to pcl */
    sensor_msgs::PointCloud2::Ptr input_cloud = input;
    input_cloud->fields[3].name = "intensity";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    /** remove invalid points*/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    /** perform downsampling for performance */
    /**
    pcl::VoxelGrid<pcl::PointXYZI> sor_down;
    sor_down.setInputCloud(cloud->makeShared());
    sor_down.setLeafSize(0.02, 0.02, 0.02);
    sor_down.filter(*cloud);
    **/

    /** remove outliers by statical filtering */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_stat;
    sor_stat.setInputCloud(cloud->makeShared());
    sor_stat.setMeanK(50);
    sor_stat.setStddevMulThresh(1.0);
    sor_stat.setNegative(false);
    sor_stat.filter(*cloud);
    
    //include SAC
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.7 * nr_points)
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
	pcl::ExtractIndices<pcl::PointXYZI> extract;
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
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::cout << cluster_indices.size() << std::endl;

    int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_cluster_list;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
	  cloud_cluster->points.push_back(cloud->points[*pit]); //*
	  std::cout << cloud->points[*pit];
	}
	std::cout << std::endl;
	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	cloud_cluster_list.push_back(*cloud_cluster);
	std::cout << "PointCloud representing the Cluster:" << cloud_cluster->points.size() << " %d  data points." << std::endl;

      }
    pcl::PointCloud<pcl::PointXYZI> cluster_sum;
    ROS_INFO("Cluster size is %d", cloud_cluster_list.size());
    for(size_t i = 0; i < cloud_cluster_list.size(); i++){
      cluster_sum += cloud_cluster_list[i];
    }
    
    sensor_msgs::PointCloud2 filter_cloud;
    pcl::toROSMsg(cluster_sum, filter_cloud);
    filter_cloud.header = input->header;
    pc_pub.publish(filter_cloud);
  }
};



