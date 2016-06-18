#include "ObjectCluster.h"

#define X 0
#define Y 1
#define Z 2

class RGBDObjectCluster : virtual private ObjectClusterInterface{

private:
  ros::Subscriber pc_sub;
  ros::NodeHandle nh;
  ros::Publisher pc_pub;
  ros::Publisher vis_pub;

public:
  explicit RGBDObjectCluster(ros::NodeHandle& n):
    nh(n) {
    pc_sub = nh.subscribe("/camera/depth/points",1,
			  &RGBDObjectCluster::Clustering, this);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
    //ROS_INFO("starting clustering.");
  }
  
  void Clustering(const sensor_msgs::PointCloud2::Ptr &input){
    /** convert sensor_msgs to pcl */
    sensor_msgs::PointCloud2::Ptr input_cloud = input;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    /** remove invalid points*/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    
    /** perform downsampling for performance */
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_down;
    sor_down.setInputCloud(cloud->makeShared());
    sor_down.setLeafSize(0.01, 0.01, 0.01);
    sor_down.filter(*cloud);

    /** remove outliers by statical filtering */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_stat;
    sor_stat.setInputCloud(cloud->makeShared());
    sor_stat.setMeanK(50);
    sor_stat.setStddevMulThresh(1.0);
    sor_stat.setNegative(false);
    sor_stat.filter(*cloud);
    
    //include SAC
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());

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
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
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
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > cloud_cluster_list;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
	  cloud_cluster->points.push_back(cloud->points[*pit]); //*
	}
	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	cloud_cluster_list.push_back(*cloud_cluster);

      }
    pcl::PointCloud<pcl::PointXYZRGB> cluster_max;
    //ROS_INFO("Cluster size is %d", cloud_cluster_list.size());
    double size_max_pc = 0.0;
    double size_max[3] = {0.0,0.0,0.0};
    double max_pos [3] = {0.0,0.0,0.0};
    double prob = 0.0;
    double pos[3] = {0.0, 0.0, 0.0};
    double lpos[3] = {0.0, 0.0, 0.0};
    double volume_max = 0.0;

    for(size_t i = 0; i < cloud_cluster_list.size(); i++){
      /** get max/min */
      pcl::PointXYZRGB min_point, max_point; 
      pcl::getMinMax3D(cloud_cluster_list[i], min_point, max_point);

      Eigen::Vector4f centroid;
      compute3DCentroid(cloud_cluster_list[i], centroid);
      
      /** get size */
      double x = (max_point.x - min_point.x);
      double y = (max_point.y - min_point.y);
      double z = (max_point.z - min_point.z);

      /** update position */
      lpos[X] = pos[X];
      lpos[Y] = pos[Y];
      lpos[Z] = pos[Z];
      pos[X] = centroid.x();
      pos[Y] = centroid.y();
      pos[Z] = centroid.z();

      if(CentroidRule(pos) * TangentRule(y/x) * VolumeRule(x*y*z) * HeightRule(y) > prob){
	prob = CentroidRule(pos) * TangentRule(y/x) * VolumeRule(x*y*z) * HeightRule(y);
	/** remember size and position */
	size_max_pc = x * y * z;
	size_max[X] = x;
	size_max[Y] = y;
	size_max[Z] = z;
	max_pos[X] = pos[X];
	max_pos[Y] = pos[Y];
	max_pos[Z] = pos[Z];
	cluster_max = cloud_cluster_list[i];
      }
    }

    ROS_INFO("human size : %f, %f, %f", size_max[X], size_max[Y], size_max[Z]);
    ROS_INFO("human possibility : %f", prob * 100.0);
    ROS_INFO("position : %f, %f, %f", max_pos[X], max_pos[Y], max_pos[Z]);
    ROS_INFO("volume : %f", size_max[X] * size_max[Y] * size_max[Z]);

    displayBoundingBox(cluster_max);
    sensor_msgs::PointCloud2 filter_cloud;
    pcl::toROSMsg(cluster_max, filter_cloud);
    filter_cloud.header.frame_id = "realsense_frame";
    pc_pub.publish(filter_cloud);
  }

  double VolumeRule(double volume){
    if(volume > 0.7){
      if(1.4 - volume < 0)
	return 0.0;
    }
    else if(volume < 0.08)
      return 0.0;
  }
  
  double WidthRule(double width){
    if(width >= 0.2 && 1.8 >= width)
      return 1.0;
    else if(width < 0.2)
      return width / 0.2;
    else
      return 1.8/width;
  }

  double HeightRule(double height){
    double real_height = height + 0.4;
    if(real_height >= 0.9 && real_height <= 2.5)
      return 1.0;
    
    else if(real_height > 2.2)
      return 0.0;

    else
      return  real_height / 0.9;
  }


  double DepthRule(double z){
    if(z >= 0.2 && 0.7 >= z)
      return 1.0;
    else if(z < 0.2)
      return z / 0.2;
    else
      return 0.7 / z;
  }

  double TangentRule(double tan){
    if(tan >= 1.5 && 7.0 >= tan){
      return 1.0;
    }
    else if(tan < 1.5)
      return tan/1.5;
    else
      return 0;
  }

  double KalmanFilter(double lpos[], double pos[]){
    double prob[3] = {0.0, 0.0, 0.0};
    
    double distance[3] = {0.0, 0.0, 0.0};

    distance[X] = abs(pos[X] - lpos[X]);
    distance[Y] = abs(pos[Y] - lpos[Y]);
    distance[Z] = abs(pos[Z] - lpos[Z]);

    for(int i = 0; i < 3; i++){
      if(distance[i] > 0.2)
	prob[i] = 0.0;
      else
	prob[i] = 1.0;
    }

    return prob[X] * prob[Y] * prob[Z];
  }



  double CentroidRule(double centroid[]){
    if(centroid[Y] > 0)
      return 1.0;
    else
      return 0.0;
  }

  void displayBoundingBox(pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster){
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    pcl::PointXYZRGB min_point, max_point; 

    pcl::getMinMax3D(cloud_cluster, min_point, max_point);
    geometry_msgs::Point pt1;
    pt1.x = max_point.x;
    pt1.y = min_point.y;
    pt1.z = max_point.z;
    geometry_msgs::Point pt2;
    pt2.x = min_point.x;
    pt2.y = min_point.y; 
    pt2.z = max_point.z; 
    geometry_msgs::Point pt3;
    pt3.x = max_point.x; 
    pt3.y = min_point.y; 
    pt3.z = min_point.z;

    marker.header.frame_id = "realsense_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.scale.x = (max_point.x - min_point.x);
    marker.scale.y = (max_point.y - min_point.y);
    marker.scale.z = (max_point.z - min_point.z);
    //ROS_INFO("scale is %f, %f, %f",marker.scale.x,marker.scale.y,marker.scale.z);
    marker.pose.position.x = (min_point.x + max_point.x)/2;
    marker.pose.position.y = (min_point.y + max_point.y)/2;
    marker.pose.position.z = (min_point.z + max_point.z)/2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    vis_pub.publish(marker);
  }


};



