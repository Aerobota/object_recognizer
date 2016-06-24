#!/usr/bin/env python

import rospy
import numpy as np
from object_recognizer.msg import Cluster, Prob, Info
import scipy.cluster.hierarchy as hcluster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

class HumanDetector:
    def __init__(self):
        self.cluster_sub = rospy.Subscriber("/cluster", Cluster, self.callback)
        self.marker_pub = rospy.Publisher("/location_marker", Marker)
        self.pos_pub = rospy.Publisher("/human_location", Vector3)
        self.pos  = Vector3()


    def callback(self, cluster):
        human_cluster = np.zeros((len(cluster.candidates),3))
        human_cluster = map(lambda info :np.array(
            [info.centroid.x,
             info.centroid.y,
             info.centroid.z]
        ), cluster.info)
        human_prob = map(lambda prob : prob.all, cluster.prob) 
        thresh = 1.0
        clusters = hcluster.fclusterdata(human_cluster,
                                         thresh,
                                         criterion="distance")
        cluster_point = np.asarray([0.0 for i in range(len(cluster.candidates))])

        for i in range(len(clusters)):
            if human_prob[i] > 0.85:
                cluster_point[clusters[i]-1] += float(len(clusters))
            if human_prob[i] > 0.7:
                cluster_point[clusters[i]-1] += human_prob[i]

        centroid  = np.zeros((1,3))
        num = 0
    
        for i in range(len(clusters)):
            if clusters[i]-1 == np.argmax(cluster_point):
                centroid += human_cluster[i]
                num += 1
                if human_prob[i] > 0.5:
                    print human_prob[i]
        if num > 0:
            centroid =  centroid / num
            marker = Marker()
            marker.header.frame_id = "realsense_frame"
            marker.action = marker.ADD
            marker.type = marker.SPHERE
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.pose.position.x = centroid[0][0]
            marker.pose.position.y = centroid[0][1]
            marker.pose.position.z = centroid[0][2]
            self.marker_pub.publish(marker)
            self.pos.x = centroid[0][0]
            self.pos.y = centroid[0][1]
            self.pos.z = centroid[0][2]
            self.pos_pub.publish(pos)

        else:
            rospy.loginfo("no cluster found")
            self.pos.z = 0.0
            self.pos_pub.publish(pos)
        

if __name__ == "__main__":
    rospy.init_node("human_detector",anonymous=True)
    humandetector = HumanDetector()
    rospy.spin()
