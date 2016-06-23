#include "RGBDBookCluster.cpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "object_cluster");

  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh("~");
  RGBDBookCluster* bookcluster = new RGBDBookCluster(nh);
  //ros::spin();
  spinner.start();
  ros::waitForShutdown();
}
