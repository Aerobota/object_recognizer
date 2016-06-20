#include "RGBDNonHumanCluster.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "object_cluster");
  
  ros::NodeHandle nh("~");
  RGBDNonHumanCluster* objectcluster = new RGBDNonHumanCluster(nh);
  ros::spin();
}
