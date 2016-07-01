#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import caffe
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from object_recognizer.msg import Cluster, Info
from visualization_msgs.msg import MarkerArray, Marker
import os
import math
from subprocess import call

camera_fov = [77, 43]
camera_res = [640, 480]
obj_pub = rospy.Publisher("/objects", Cluster)
marker_pub = rospy.Publisher("/classification_markers", MarkerArray)

def normalize(arr):
    for i in range(3):
        minval = arr[...,i].min()
        maxval = arr[...,i].max()
        if minval != maxval:
            arr[...,i] -= minval
            arr[...,i] *= (255.0 / (maxval - minval))

    return arr


class ObjectRecognizer:
    def __init__(self):
        os.chdir("/home/"+os.getenv('USER')+"/catkin_ws/src/object_recognizer")
        self.net = caffe.Classifier(
            "cfg/deploy.prototxt",
            "cfg/snapshot_iter_2640.caffemodel"
        )
        self.labels = {0 : "7up", 1 : "banana", 2 :  "cereal",
                       3 : "potato", 4 : "apple", 5 : "beer",
                       6 : "coke", 7 : "milk", 8 : "pringles",
                       9 : "sprite", 10 : "apple_juice", 11 : "can_coffee",
                       12 : "fanta", 13 : "orange", 14 : "red_bull"}
        self.bridge = CvBridge()
        self.img = []
        self.step_time = 0

    def imgCb(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    def Recognizer(self, data):
        call(["import","-w","root","hello.jpg"])
        call(["convert","hello.jpg","-compress","Zip","demuranet-Try-20160701-"+str(self.step_time)+".pdf"])
        self.step_time += 1
        for i in range(len(data.info)):
            if self.img is not []:
                image_deg = [math.degrees(math.atan2(data.info[i].centroid.x, data.info[i].centroid.z)),math.degrees(math.atan2(data.info[i].centroid.y, data.info[i].centroid.z))]            
                img_x = image_deg[0] / float(camera_fov[0]) * camera_res[0]
                img_y = image_deg[1] / float(camera_fov[1]) * camera_res[1]
                img_w = data.width[i] /data.info[i].centroid.z * camera_res[0]
                img_h = data.height[i] / data.info[i].centroid.z * camera_res[1]
                self.img = normalize(self.img.astype(np.float32))
                print self.img
                image = self.img[img_y:img_y+img_h, img_x:img_x+img_w]

                #print image
                try:
                    score = self.net.predict([image], oversample=False)
                    data.info[i].type = self.labels[np.argmax(score)]
                except:
                    data.info[i].type = "undef"
                    
                rospy.loginfo("object classification : %s", data.info[i].type) 

                markerArray = MarkerArray()
                for i in range(len(data.info)):
                    marker = Marker()
                    marker.header.frame_id = "realsense_frame"
                    marker.action = marker.ADD
                    marker.type = marker.TEXT_VIEW_FACING
                    marker.text = str(data.info[i].type)
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.scale.x = 0.25
                    marker.scale.y = 0.25
                    marker.scale.z = 0.25
                    marker.pose.position.x = data.info[i].centroid.x
                    marker.pose.position.y = data.info[i].centroid.y
                    marker.pose.position.z = data.info[i].centroid.z
                    if data.info[i].type != "potato" and data.info[i].type != "banana" and data.info[i].type != "undef" and data.info[i].type != [] and data.info[i].type != "[]":
                        markerArray.markers.append(marker)

                for i in range(len(markerArray.markers)):
                    markerArray.markers[i].id = i
                    markerArray.markers[i].lifetime = rospy.Duration(1.0)

                marker_pub.publish(markerArray)
                obj_pub.publish(data)


            else:
                rospy.loginfo("waiting for receiving image")


if __name__ == "__main__":
    rospy.init_node("obj_recog")
    obj_recog = ObjectRecognizer()
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, obj_recog.imgCb)
    per_sub = rospy.Subscriber("/cluster", Cluster, obj_recog.Recognizer)
    rospy.spin() 
    
    
