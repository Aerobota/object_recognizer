#ifndef OBJECTCLUSTER_H
#define OBJECTCLUSTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
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
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include <vector> 
#include <cmath>
#include "ObjectClusterInterface.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <object_recognizer/Prob.h>
#include <object_recognizer/Info.h>
#include <object_recognizer/Cluster.h>

#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/range_image/range_image.h>
#include <typeinfo>

#ifndef INFINITY
#define INFINITY HUGE_VAL
#endif

#ifndef NAN
#define NAN 0xffc00000
#endif

#endif // OBJECTCLUSTER_H
