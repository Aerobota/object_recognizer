#include "RGBDBookCluster.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "object_cluster");
  
  ros::NodeHandle nh("~");
  RGBDBookCluster* bookcluster = new RGBDBookCluster(nh);
  ros::spin();
}
