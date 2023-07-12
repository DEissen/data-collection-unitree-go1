#!/usr/bin/env python3
import sys
import numpy as np
import cv2
import time

import rospy
from sensor_msgs.msg import PointCloud2, Range, PointField
import sensor_msgs.point_cloud2 as pc2

def sub_callback_PointCloud(sub_msg: PointCloud2):
    cloud = pc2.read_points(sub_msg)
    timestamp = time.strftime("%H_%M_%S")

    data_arr = []

    # get data from point cloud generator
    for data in cloud:
        data_arr.append(data)

    # save array in pcd format by converting it to a numpy array first
    data_arr = np.asarray(data_arr)
    length = np.shape(data_arr)[0]
    rospy.loginfo(f"length {length}, from message {sub_msg.width}") # is equal!
    pcd_file_header = f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {length}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {length}\nDATA ascii"
    np.savetxt(f'PointCloud_{timestamp}_{sub_msg.header.seq}.pcd', data_arr,
               delimiter=" ", fmt="%1.5f", comments="", header=pcd_file_header)
    
    rospy.loginfo(f"Recieved and stored point cloud {sub_msg.header.seq}")


if __name__ == "__main__":
    # init node and log start
    rospy.init_node("test_node")
    rospy.loginfo("Node \"test_node\" started")

    camera1_pointCloud_sub = rospy.Subscriber(
        "/camera1/point_cloud_face", PointCloud2, callback=sub_callback_PointCloud)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()
