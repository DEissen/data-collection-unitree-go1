#!/usr/bin/env python3
import sys
import numpy as np
import time
import os
from os import path

import rospy
from sensor_msgs.msg import PointCloud2, Range, PointField
import sensor_msgs.point_cloud2 as pc2


class Camera_PointCloud_Subscriber:
    def __init__(self, camera_id, add_debug_infos):
        self.camera_id = camera_id
        self.add_debug_infos = add_debug_infos

        # create folder for results (if it does not exist yet)
        measurement_timestamp = time.strftime("%H_%M_%S")
        self.data_dir = f"./measurement{measurement_timestamp}/cam{self.camera_id}_pointClouds"
        os.makedirs(self.data_dir, exist_ok=True)

        # create callback
        topic_name = f"/camera{self.camera_id}/point_cloud_face"
        self.sub = rospy.Subscriber(
            topic_name, PointCloud2, callback=self.sub_callback)
        rospy.loginfo(f"Created PointCloud Subscriber for topic {topic_name}")

    def sub_callback(self, sub_msg: PointCloud2):
        timestamp = time.strftime("%H_%M_%S")
        data_arr = []
        recieved_cloud = pc2.read_points(sub_msg)

        # get data from point cloud generator
        for data in recieved_cloud:
            data_arr.append(data)

        # save array in pcd format by converting it to a numpy array first
        data_arr = np.asarray(data_arr)

        # optionally add points to indicate directions by adding the following four points to the point cloud:
        # 0 0 0 4.2108e+01      => reference point in origin in darkblue
        # 0.1 0 0 4.2108e+01    => reference point at x = 0.1 in darkblue
        # 0 0.1 0 4.2108e+024   => reference point at y = 0.1 in lightblue
        # 0 0 0.1 0             => reference point at z = 0.1 in black
        if self.add_debug_infos:
            darkblue = 4.2108e+01
            lightblue = 4.2108e+024
            black = 0
            data_arr = np.append(data_arr, [[0, 0, 0, darkblue], [0.1, 0, 0, darkblue], [
                0, 0.1, 0, lightblue], [0, 0, 0.1, black]], axis=0)

        # prepare pcd header
        length = np.shape(data_arr)[0]
        pcd_file_header = f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {length}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {length}\nDATA ascii"
        
        np.savetxt(os.path.join(self.data_dir, f'cam{self.camera_id}_{timestamp}_{sub_msg.header.seq}.pcd'), data_arr,
                   delimiter=" ", fmt=["%.10f", "%.10f", "%.10f", "%.10e"], comments="", header=pcd_file_header)

        rospy.loginfo(f"Recieved and stored point cloud {sub_msg.header.seq}")

def get_all_pointClouds():
    """
        Function to get all data from all cameras in parallel.
        NOTE: This function contains an infinite loop!
    """
    # init node and log start
    rospy.init_node("pointCloud_Subscriber_Node")
    rospy.loginfo("Node \"pointCloud_Subscriber_Node\" started")

    camera1_pointCloud_sub = Camera_PointCloud_Subscriber(1, True)
    camera2_pointCloud_sub = Camera_PointCloud_Subscriber(2, True)
    camera3_pointCloud_sub = Camera_PointCloud_Subscriber(3, True)
    camera4_pointCloud_sub = Camera_PointCloud_Subscriber(4, True)
    camera5_pointCloud_sub = Camera_PointCloud_Subscriber(5, True)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()

def get_single_pointCloud_by_ID(camera_ID):
    """
        Function to get PointCloud from specific camera.
        NOTE: This function contains an infinite loop!
    """
    # init node and log start
    rospy.init_node("pointCloud_Subscriber_Node")
    rospy.loginfo("Node \"pointCloud_Subscriber_Node\" started")

    camera_pointCloud_sub = Camera_PointCloud_Subscriber(camera_ID, True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    # get_all_pointClouds()
    get_single_pointCloud_by_ID(1)
