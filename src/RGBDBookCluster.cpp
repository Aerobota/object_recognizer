#include "ObjectCluster.h"

#define X 0
#define Y 1
#define Z 2

class RGBDBookCluster : virtual private ObjectClusterInterface{

private:
  ros::Subscriber pc_sub;
  ros::NodeHandle nh;
  ros::Publisher pc_pub;
  ros::Publisher vis_pub;
  ros::Publisher cluster_pub;  
  //ros::Publisher image_pub;
  //ros::Subscriber img_sub;
  //mutable cv_bridge::CvImagePtr bridge;
  //cv_bridge::CvImagePtr out_bridge;
  //cv::Rect* roi;
  object_recognizer::Cluster cluster;
  int img_x;
  int img_y;
  int img_w;
  int img_h;



public:
  explicit RGBDBookCluster(ros::NodeHandle& n):
    nh(n), img_x(0), img_y(0), img_w(0), img_h(0) {
    pc_sub = nh.subscribe("/camera/depth/points",5,
			  &RGBDBookCluster::Clustering, this);
    cluster_pub = nh.advertise<object_recognizer::Cluster>("/cluster", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker",10);
  }

  void Clustering(const sensor_msgs::PointCloud2::Ptr &input){
    /** convert sensor_msgs to pcl */
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


    double scale_min = 0.02;
    double scale_max = 1.00;
    double threshold = 0.8;
    double segradius = 0.40;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized ())
      {
	tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
      }
    else
      {
	tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
      }

    tree->setInputCloud(cloud);

    if(scale_min >= scale_max){
      ROS_ERROR("large scale must be > small scale!");
      exit(-1);
    }

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);

    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    std::cout << "Calculating normals for scale..." << scale_min << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale_min);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    std::cout << "Calculating normals for scale..." << scale_max << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale_max);
    ne.compute (*normals_large_scale);


    pcl::PointCloud<pcl::PointNormal>::Ptr 
      doncloud(new pcl::PointCloud<pcl::PointNormal>);
    
    pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointNormal>
      (*cloud, *doncloud);
    
    std::cout << "calculating DoN." << std::endl;
    pcl::DifferenceOfNormalsEstimation
      <pcl::PointXYZRGB,pcl::PointNormal,pcl::PointNormal> don;

    don.setInputCloud(cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if(!don.initCompute()){
      ROS_ERROR("Could not initialize DoN Features.");
      exit(0);
    }
    
    don.computeFeature(*doncloud);
    
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond
      (new pcl::ConditionOr<pcl::PointNormal> ()
       );
    range_cond->addComparison(
			      pcl::FieldComparison
			      <pcl::PointNormal>::ConstPtr(
							   new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)
							   )
			      );
    pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
    condrem.setInputCloud(doncloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered
      (new pcl::PointCloud<pcl::PointNormal>);
    condrem.filter(*doncloud_filtered);
    doncloud = doncloud_filtered;
    
    std::cout << "Filtered Pointcloud: " << doncloud->points.size() <<
      " data points." << std::endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr 
      segtree(new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud(doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000000);
    ec.setSearchMethod(segtree);
    ec.setInputCloud(doncloud);
    ec.extract(cluster_indices);

    int j = 0;
    for(std::vector<pcl::PointIndices>::const_iterator it =
	  cluster_indices.begin(); it != cluster_indices.end(); ++it, j++){
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don
	(new pcl::PointCloud<pcl::PointNormal>);
      for(std::vector<int>::const_iterator pit = it->indices.begin();
	  pit != it->indices.end(); ++pit){
	cloud_cluster_don->points.push_back(doncloud->points[*pit]);
      }
      cloud_cluster_don->width = int(cloud_cluster_don->points.size());
      cloud_cluster_don->height = 1;
      cloud_cluster_don->is_dense = true;
      /** position */
      double pos[3] = {0.0, 0.0, 0.0};
      double lpos[3] = {0.0, 0.0, 0.0};
      
      object_recognizer::Prob prob;
      object_recognizer::Info info;
      object_recognizer::Object object;
      
      /** convert pointnormal from XYZRGB */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_rgbd
	(new pcl::PointCloud<pcl::PointXYZRGB>);
      cloud_cluster_rgbd->resize(cloud_cluster_don->size());
      for(size_t i = 0; i < cloud_cluster_don->points.size(); ++i){
	const pcl::PointNormal &rgbd_pt = cloud_cluster_don->points[i];
	uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
	uint8_t r = (rgb >> 16) & 0x0000ff;
	uint8_t g = (rgb >> 8)  & 0x0000ff;
	uint8_t b = (rgb)       & 0x0000ff;

	pcl::PointXYZRGB pt(r, g, b);
	pt.x = rgbd_pt.x;
	pt.y = rgbd_pt.y;
	pt.z = rgbd_pt.z;
	cloud_cluster_rgbd->push_back(pt);
      }

      pcl::PointXYZRGB min_point, max_point;
      pcl::getMinMax3D(*cloud_cluster_rgbd, min_point, max_point);
      Eigen::Vector4f centroid;
      compute3DCentroid(*cloud_cluster_rgbd, centroid);
      double x = (max_point.x - min_point.x);
      double y = (max_point.y - min_point.y);
      double z = (max_point.z - min_point.z);
 
      pos[X] = centroid.x();
      pos[Y] = centroid.y();
      pos[Z] = centroid.z();


      prob.all = AreaRule(x * y);
      prob.all = 1;
      prob.centroid = CentroidRule(pos);
      prob.tangent = TangentRule(y/x);
      prob.volume = VolumeRule(x*y*z);
      prob.height = HeightRule(y);
      
      info.volume = x * y * z;
      info.centroid.x = pos[X];
      info.centroid.y = pos[Y];
      info.centroid.z = pos[Z];
           
      sensor_msgs::PointCloud2 cluster_cloud;
      sensor_msgs::Image image;

      pcl::PointXYZRGB cl_min_point, cl_max_point; 
      pcl::getMinMax3D(*cloud, cl_min_point, cl_max_point);
      Eigen::Vector4f cl_centroid;
      compute3DCentroid(*cloud, cl_centroid);


      float cl_w = cl_max_point.x - cl_min_point.x;
      float cl_h = cl_max_point.y - cl_min_point.y;
      int img_x = int((min_point.x - cl_min_point.x)/cl_w * 640);
      int img_y = int((min_point.y - cl_min_point.y)/cl_h * 480);
      int img_w = int(x/cl_w * 640);
      int img_h = int(y/cl_h * 480);

      //roi = new cv::Rect(img_x, img_y, img_w, img_h);

      pcl::toROSMsg(*cloud_cluster_rgbd, cluster_cloud);
      cluster_cloud.is_dense = true;
      cluster_cloud.header.frame_id = "realsense_frame";

      cluster.candidates.push_back(cluster_cloud);
      cluster.width.push_back(img_w);
      cluster.height.push_back(img_h);
      cluster.x.push_back(img_x);
      cluster.y.push_back(img_y);

      cluster.prob.push_back(prob);
      cluster.info.push_back(info);
    }

    ROS_INFO("number of candidates are : %d", cluster.candidates.size());
    displayBoundingBox(cluster);
    displayCluster(cluster);
    cluster_pub.publish(cluster);
  }
  
  double VolumeRule(double volume){
    if(0.01 >= volume ){
      return 1.0;
    }
    else 
      return 0.0;
  }

  double AreaRule(double area){
    if(0.001 <=area && area <= 0.10){
      return 1.0;
    }
    else
      return 0;
  }

  
  double WidthRule(double width){
    if(width >= 0.05 && 0.4 >= width)
      return 1.0;
    else if(width < 0.05)
      return width;
    else if(1.0 - width < 0)
      return 0.0;
    else
      return (1.0 - width);
  }

  double HeightRule(double height){
    if(height >= 0.05 && height <= 0.4)
      return 1.0;
    else if(height > 0.4)
      if(1.0 - height < 0)
	return 0.0;
      else
	return 1.0 - height;
    else
      return  height;
  }


  double DepthRule(double z){
    if(z >= 0.05 && 0.3 >= z)
      return 1.0;
    else if(z < 0.05)
      return z;
    else if (0.5 - z < 0)
      return 0;
    else
      return (0.5 - z);
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

  double ZTangentRule(double tan){
    if(tan <= 1.0)
      return 1;
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
    if(centroid[Z] < 1.5)
      return 1.0;
    else
      return 0.0;
  }

  void displayCluster(object_recognizer::Cluster cluster){
    pcl::PointCloud<pcl::PointXYZRGB> cluster_sum;
    pcl::PointCloud<pcl::PointXYZRGB> cluster_elem;
    for(size_t i = 0; i < cluster.candidates.size(); i++){
      pcl::fromROSMsg(cluster.candidates[i], cluster_elem);
      cluster_sum += cluster_elem;
    }
    sensor_msgs::PointCloud2 cluster_cloud;
    pcl::toROSMsg(cluster_sum, cluster_cloud);
    cluster_cloud.header.frame_id = "realsense_frame";
    pc_pub.publish(cluster_cloud);
  }

  void displayBoundingBox(object_recognizer::Cluster cluster){
    visualization_msgs::MarkerArray marker_array;

    for(size_t i = 0; i < cluster.candidates.size(); i++){
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.frame_id = "realsense_frame";
      marker.header.stamp = ros::Time();

      pcl::PointXYZRGB min_point, max_point; 

      pcl::PointCloud<pcl::PointXYZRGB> cloud;

      pcl::fromROSMsg(cluster.candidates[i], cloud);


      pcl::getMinMax3D(cloud, min_point, max_point);
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
      marker.color.r = cluster.prob[i].all;
      marker.color.g = 0.0;
      marker.color.b = 1.0 - cluster.prob[i].all;
      marker.scale.x = (max_point.x - min_point.x);
      marker.scale.y = (max_point.y - min_point.y);
      marker.scale.z = (max_point.z - min_point.z);
      marker.pose.position.x = (min_point.x + max_point.x)/2;
      marker.pose.position.y = (min_point.y + max_point.y)/2;
      marker.pose.position.z = (min_point.z + max_point.z)/2;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker_array.markers.push_back(marker); 
    }
    for(size_t i = 0; i < marker_array.markers.size(); i++){
      marker_array.markers[i].id = i;
    }

    vis_pub.publish(marker_array);
  }


};



