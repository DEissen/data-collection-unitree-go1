#!/usr/bin/env python3
import sys

import rospy
from sensor_msgs.msg import PointCloud2, Range, PointField

range_received = False
pointCloud_received = False

def sub_callback_range(sub_msg: Range):
    if not range_received:
        range = sub_msg.range

        rospy.loginfo(sub_msg)
        rospy.loginfo("Current range is: ", range)

        range_received = True

def sub_callback_PointCloud(sub_msg: PointCloud2):
    if not pointCloud_received:
        data = sub_msg.data
        data_length = len(data)

        rospy.loginfo(sub_msg)
        rospy.loginfo("Length of data is: ", data_length)

        pointCloud_received = True

if __name__ == "__main__":
    # init node and log start
    rospy.init_node("test_node")
    rospy.loginfo("Node \"test_node\" started")

    camera1_range_sub = rospy.Subscriber("/camera1/range_visual_face", Range, callback=sub_callback_range)
    camera1_pointCloud_sub = rospy.Subscriber("/camera1/point_cloud_face", PointCloud2, callback=sub_callback_PointCloud)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        
        if range_received and pointCloud_received:
            rospy.loginfo("Messages received. Node \"test_node\" will be closed.")
            sys.exit(0)
        
        rate.sleep()
