#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from object_recognizer.msg import Person
from visualization_msgs.msg import MarkerArray, Marker
import tf
import math

pos_pub = rospy.Publisher("/human_location_filtered", Person)
marker_pub = rospy.Publisher("/location_markers", MarkerArray)

class HumanFilter:
    def __init__(self):
        self.last_human_number = 0
        self.human_number = 0
        self.isFirst = True
        self.last_human = Person()
        self.name = ["Kousaka_Honoka", "Yazawa_Niko", "Nishino_Maki","Ayase_Eri","Minami_Kotori","Sonoda_Umi","Toujo_Nozomi","Hoshizora_Rin","Koizumi_hanayo"]

    def FilterHuman(self, data):
        self.human_number = len(data.centroids)
        human_number_output = 0
        if self.isFirst:
            self.last_human_number = self.human_number
            self.last_human = data
            self.isFirst = False
            human_number_output = self.last_human_number
            markerArray = MarkerArray()
            for i in range(human_number_output):
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
                marker.pose.position.x = data.centroids[i].x
                marker.pose.position.y = data.centroids[i].y + 0.8
                marker.pose.position.z = data.centroids[i].z
                q = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
                marker.pose.orientation.x = q[0]
                marker.pose.orientation.y = q[1]
                marker.pose.orientation.z = q[2]
                marker.pose.orientation.w = q[3]
                markerArray.markers.append(marker)

            for i in range(len(markerArray.markers)):
                markerArray.markers[i].id = i
                markerArray.markers[i].lifetime = rospy.Duration(1.0)

            marker_pub.publish(markerArray)
            pos_pub.publish(self.last_human)



        elif self.human_number > self.last_human_number:
            human_number_output = self.last_human_number
            self.last_human_number = self.human_number
            self.last_human = data

        else:
            norm = [100, 100, 100]
            min_norm = [100 for i in range(len(data.centroids))]
            min_index = [-1 for i in range(len(data.centroids))]
            for i in range(self.last_human_number):
                for j in range(self.human_number):
                    if i is j:
                        continue
                    else:
                        norm[0] = abs(self.last_human.centroids[i].x - data.centroids[j].x)
                        norm[1] = abs(self.last_human.centroids[i].y - data.centroids[j].y)
                        norm[2] = abs(self.last_human.centroids[i].z - data.centroids[j].z)
                        if i > len(data.centroids) -1:
                            break
                            
                        if norm[0] + norm[1] + norm[2] < min_norm[i]:
                            min_norm[i] = norm[0] + norm[1] + norm[2]
                            min_index[i] = j

            human_number_output = self.human_number
            print self.last_human_number, self.human_number
            self.last_human_number = self.human_number

            for i in range(human_number_output):
               self.last_human.centroids[i] = data.centroids[min_index[i]]
               
            markerArray = MarkerArray()
            for i in range(human_number_output):
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
                marker.pose.position.x = self.last_human.centroids[i].x
                marker.pose.position.y = self.last_human.centroids[i].y + 0.8
                marker.pose.position.z = self.last_human.centroids[i].z
                q = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
                marker.pose.orientation.x = q[0]
                marker.pose.orientation.y = q[1]
                marker.pose.orientation.z = q[2]
                marker.pose.orientation.w = q[3]
                markerArray.markers.append(marker)

            for i in range(len(markerArray.markers)):
                markerArray.markers[i].id = i
                markerArray.markers[i].lifetime = rospy.Duration(1.0)
                
            marker_pub.publish(markerArray)
            pos_pub.publish(self.last_human)

if __name__ == "__main__":
    rospy.init_node("human_filter")
    human_filter = HumanFilter()
    filter_sub = rospy.Subscriber("/human_location", Person, human_filter.FilterHuman)
    rospy.spin()
