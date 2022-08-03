#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import nav_msgs.msg
# import geometry_msgs

# When topics are synced, they should be put in a new bag file together
def callback(leftcam,rightcam,robotpos,objectpos):
  # rospy.loginfo('Match found')
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', leftcam.header)
  rospy.loginfo(rospy.get_caller_id() + ' I heard %s', robotpos.header)
  

left_sub = message_filters.Subscriber('/zed_node/left/camera_info_throttle', CameraInfo)
right_sub = message_filters.Subscriber('/zed_node/right/camera_info_throttle', CameraInfo)
robot_sub = message_filters.Subscriber('/Husky/Pose', nav_msgs.msg.Odometry)
object_sub = message_filters.Subscriber('/Box/Pose', nav_msgs.msg.Odometry)
rospy.init_node('camera_synchronizer_node',anonymous=True)

# ts = message_filters.ApproximateTimeSynchronizer([info1_sub, info2_sub, object_pose_sub], queue_size=10, slop=10)
ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub, robot_sub, object_sub], queue_size=10, slop=0.01)
#ts = message_filters.TimeSynchronizer([info1_sub, info2_sub],queue_size=1)
#ts = message_filters.TimeSynchronizer([object_sub, robot_sub],queue_size=10)
#ts = message_filters.ApproximateTimeSynchronizer([object_sub, robot_sub],queue_size=10,slop=1)

#, object_pose_sub], 10)
ts.registerCallback(callback)
rospy.spin()
