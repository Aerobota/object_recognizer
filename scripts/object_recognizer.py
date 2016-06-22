#!/usr/bin/env python
import rospy
from object_recognizer.msg import Cluster, Prob, Info
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ObjectRecognizer:
    def __init__(self):
        self.cluster_sub = rospy.Subscriber("/cluster", Cluster, self.Recognize)
        self.last_textures = 0
        self.textures = 0
        self.bridge = CvBridge()
        self.isFirst = True


    def Recognizer(self, cluster):
        pass


            


if __name__ == "__main__":
    rospy.init_node("object_recognizer")
    obj_recog = ObjectRecognizer()
    rospy.spin()
    
