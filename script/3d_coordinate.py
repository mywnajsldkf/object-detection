#!/usr/bin/env python

import rospy
from math import pow, atan2, sqrt
from tf.transformations import *

# darknet_ros
import time
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import tf
from enum import Enum

class ur3_tomatoByYoloJsk():
    def __init__(self):
        self.detected = {}
        self.detected_names = rospy.get_param('/darknet_ros/yolo_model/detection_classes/names')
        self.object_pose_sub = rospy.Subscriber('/cluster_decomposer/centroid_pose_array',PoseArray,self.collectJsk)
        self.listener = tf.TransformListener()
        self.trans = [0.0, 0.0, 0.0]

        self.object_name_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.collect)

        self.tomatoboundingBox = BoundingBox()

    def collectJsk(self,msg):
        for i,pose in enumerate(msg.poses):
            if pose != Pose():
                try:
                    pos = pose.position
                    val = [pos.x, pos.y, pos.z]
                    key = self.detected_names[i]
                    print('Found a {} at {}'.format(key, val))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn('there is no tf')

    def collect(self,msg):
        for i, boundingBox in enumerate(msg.bounding_boxes):
            print("BoundingBox {} {} ".format(i, boundingBox))
            if boundingBox.Class == "tomato":
                self.tomatoboundingBox = boundingBox
                self.last_yolodetect_time = rospy.get_rostime()

if __name__ == '__main__':
    rospy.init_node('detection_collector')
    ur3_tomatoByYoloJsk()
    rospy.spin()
