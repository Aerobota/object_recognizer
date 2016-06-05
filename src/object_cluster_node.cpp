#include "LIDARObjectCluster.cpp"
#include "RGBDObjectCluster.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "object_cluster");
  
  ros::NodeHandle nh("~");
  RGBDObjectCluster* objectcluster = new RGBDObjectCluster(nh);
  ros::spin();
}
