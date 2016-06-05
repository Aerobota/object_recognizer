#include "ObjectCluster.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "object_cluster");
  
  ros::NodeHandle nh("~");
  ObjectCluster* objectcluster = new ObjectCluster(nh);
  ros::spin();
}
