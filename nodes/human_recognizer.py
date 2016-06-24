#!/usr/bin/env python

import tf
import math
import rospy
import numpy as np
from object_recognizer.msg import Cluster, Prob, Info, Person
import scipy.cluster.hierarchy as hcluster
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3

class HumanDetector:
    def __init__(self):
        self.cluster_sub = rospy.Subscriber("/cluster", Cluster, self.callback)
        self.marker_pub = rospy.Publisher("/location_markers", MarkerArray)
        self.pos_pub = rospy.Publisher("/human_location", Person)
        self.pos  = Vector3()
        self.name = ["Kousaka_Honoka", "Yazawa_Niko", "Nishino_Maki", "Ayase_Eri","Minami_Kotori",  "Sonoda_Umi", "Toujo_Nozomi", "Hoshizora_Rin", "Koizumi_hanayo"]

    def callback(self, cluster):
        human_cluster = np.zeros((len(cluster.candidates),3))
        human_cluster = map(lambda info :np.array(
            [info.centroid.x,
             info.centroid.y,
             info.centroid.z]
        ), cluster.info)
        human_prob = map(lambda prob : prob.all, cluster.prob) 
        thresh = 0.4
        if len(cluster.candidates) > 0:
            try:
                clusters = hcluster.fclusterdata(human_cluster,
                                                 thresh,
                                                 criterion="distance")

            except ValueError:
                self.pos.x = 0.0
                self.pos.y = 0.0
                self.pos.z = 0.0
                self.pos_pub.publish(self.pos)
                
            cluster_point = np.asarray([0.0 for i in range(len(cluster.candidates))])

            candidate_cluster = []

            for i in range(len(clusters)):
                if human_prob[i] > 0.85:
                    cluster_point[clusters[i]-1] += human_prob[i]
                    candidate_cluster.append(clusters[i])

            
            centroids = [np.zeros((1,3)) for i in range(len(candidate_cluster))]
            nums = [0 for i in range(len(candidate_cluster))]

            for i in range(len(candidate_cluster)):
                for j in range(len(clusters)):
                    if clusters[j] == candidate_cluster[i]:
                        centroids[i] += human_cluster[j]
                        nums[i] += 1

                        
            if len(nums) > 0:
                rospy.loginfo("human candidates : %d", len(nums))
                centroids = [centroids[i]/nums[i] for i in range(len(candidate_cluster))]
                markerArray = MarkerArray()
                people      = Person()

                for i in range(len(centroids)):
                    marker = Marker()
                    marker.header.frame_id = "realsense_frame"
                    marker.action = marker.ADD
                    marker.type = marker.MESH_RESOURCE
                    marker.mesh_resource = "package://object_recognizer/meshes/lovelive/"+self.name[i]+".dae"
                    marker.mesh_use_embedded_materials = True
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.scale.x = 0.08
                    marker.scale.y = 0.08
                    marker.scale.z = 0.08
                    marker.pose.position.x = centroids[i][0][0]
                    marker.pose.position.y = centroids[i][0][1] + 0.8
                    marker.pose.position.z = centroids[i][0][2]
                    q = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
                    marker.pose.orientation.x = q[0]
                    marker.pose.orientation.y = q[1]
                    marker.pose.orientation.z = q[2]
                    marker.pose.orientation.w = q[3]

                    markerArray.markers.append(marker)
                    centroid = Vector3()
                    centroid.x = centroids[i][0][0]
                    centroid.y = centroids[i][0][1]
                    centroid.z = centroids[i][0][2]
                    people.centroids.append(centroid)
                
                for i in range(len(markerArray.markers)):
                    markerArray.markers[i].id = i
                    markerArray.markers[i].lifetime = rospy.Duration(1.0)

                self.marker_pub.publish(markerArray)
                self.pos_pub.publish(people)

            else:
                rospy.loginfo("no cluster found")
                people      = Person()
                centroid    = Vector3()
                centroid.x  = 0.0
                centroid.y  = 0.0
                centroid.z  = 0.0
                people.centroids.append(centroid)
                self.pos_pub.publish(people)

        else:
            people      = Person()
            centroid    = Vector3()
            centroid.x  = 0.0
            centroid.y  = 0.0
            centroid.z  = 0.0
            people.centroids.append(centroid)
            self.pos_pub.publish(people)

if __name__ == "__main__":
    rospy.init_node("human_detector",anonymous=True)
    humandetector = HumanDetector()
    rospy.spin()
