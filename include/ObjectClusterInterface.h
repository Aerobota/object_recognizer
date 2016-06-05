#ifndef OBJECTCLUSTERINTERFACE_H
#define OBJECTCLUSTERINTERFACE_H

class ObjectClusterInterface{
public:
  virtual void Clustering(const sensor_msgs::PointCloud2::Ptr &input) = 0;
  virtual ~ObjectClusterInterface(){}
};

#endif // OBJECTCLUSTERINTERFACE_H
