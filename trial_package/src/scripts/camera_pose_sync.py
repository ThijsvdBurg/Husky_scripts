#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import nav_msgs.msg
# import geometry_msgs

def callback(camera_info1, camera_info2):
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', camera_info1.header)
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', camera_info2.header)

info1_sub = message_filters.Subscriber('/zed_node/left/camera_info_throttle', CameraInfo)
info2_sub = message_filters.Subscriber('/zed_node/right/camera_info_throttle', CameraInfo)
#robot_sub = message_filters.Subscriber('/Bebop1/position_velocity_orientation_estimation', nav_msgs.msg.Odometry)
#object_sub = message_filters.Subscriber('/Bebop2/position_velocity_orientation_estimation', nav_msgs.msg.Odometry)
# object_pose_sub = message_filters.Subscriber('/', CameraInfo)
rospy.init_node('camera_synchronizer_node',anonymous=True)

# ts = message_filters.ApproximateTimeSynchronizer([info1_sub, info2_sub, object_pose_sub], queue_size=10, slop=10)

ts = message_filters.ApproximateTimeSynchronizer([info1_sub, info2_sub], queue_size=10, slop=0.01)
#ts = message_filters.TimeSynchronizer([info1_sub, info2_sub],queue_size=1)
#ts = message_filters.TimeSynchronizer([object_sub, robot_sub],queue_size=10)
#ts = message_filters.ApproximateTimeSynchronizer([object_sub, robot_sub],queue_size=10,slop=1)

#, object_pose_sub], 10)
ts.registerCallback(callback)
rospy.spin()
