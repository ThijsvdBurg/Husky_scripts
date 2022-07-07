#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
# import nav_msgs
# import geometry_msgs

def callback(camera_info1, camera_info2):
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', camera_info1.header)
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', camera_info2.header)

info1_sub = message_filters.Subscriber('/zed_node/left/camera_info_throttle', CameraInfo)
info2_sub = message_filters.Subscriber('/zed_node/right/camera_info_throttle', CameraInfo)
# object_pose_sub = message_filters.Subscriber('/', CameraInfo)
rospy.init_node('synchronizer',anonymous=True)

ts = message_filters.TimeSynchronizer([info1_sub, info2_sub], 10)
ts.registerCallback(callback)
rospy.spin()