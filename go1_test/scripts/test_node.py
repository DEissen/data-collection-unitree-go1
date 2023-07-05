#!/usr/bin/env python3
import sys
import numpy as np
import cv2
import time

import rospy
from sensor_msgs.msg import PointCloud2, Range, PointField
import sensor_msgs.point_cloud2 as pc2

range_received = False
pointCloud_received = False

def sub_callback_range(sub_msg: Range):
    range = sub_msg.range

    # rospy.loginfo(sub_msg)
    rospy.loginfo("Current range is: ", range)

    rospy.loginfo("Messages received. Node \"test_node\" will be closed.")
    sys.exit(0)

def sub_callback_PointCloud(sub_msg: PointCloud2):
    cloud = pc2.read_points(sub_msg)
    i = 0
    timestamp = time.strftime("%H-%M-%S")
    with open(f'PointCloud_{timestamp}.txt', 'w') as f:
        for data in cloud:
            f.write(str(data) + "\n")
        

if __name__ == "__main__":
    # init node and log start
    rospy.init_node("test_node")
    rospy.loginfo("Node \"test_node\" started")

    #camera1_range_sub = rospy.Subscriber("/camera1/range_visual_face", Range, callback=sub_callback_range)
    camera1_pointCloud_sub = rospy.Subscriber("/camera1/point_cloud_face", PointCloud2, callback=sub_callback_PointCloud)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
            
        rate.sleep()
