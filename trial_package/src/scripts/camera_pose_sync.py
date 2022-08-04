#!/usr/bin/env python

import rospy
import argparse
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import nav_msgs.msg
# from nav_msgs.msg import Odometry
from std_msgs.msg import String
# from geometry_msgs.msg import PoseWithCovariance

# import geometry_msgs

parser = argparse.ArgumentParser(description="Sync rosbag topics and rewrite the synced messages into a new rosbag")
parser.add_argument("--slop", type=float, default=0.05, help="Directory containing rosbags.")
args = parser.parse_args()

leftcampub = rospy.Publisher('/sync/left/camera_info', CameraInfo, queue_size=1000)
rightcampub = rospy.Publisher('/sync/right/camera_info', CameraInfo, queue_size=1000)
robotpub = rospy.Publisher('/sync/robot/pose', nav_msgs.msg.Odometry, queue_size=1000)
objectpub = rospy.Publisher('/sync/object/pose', nav_msgs.msg.Odometry, queue_size=1000)

# When topics are synced, they should be put in a new bag file together
def callback(leftcam,rightcam,robotpos,objectpos):
  # rospy.loginfo('Match found')
  # last_data=objectpos.pose
  # rospy.loginfo(rospy.get_caller_id() + ' I heard %s', leftcam.header)
  # rospy.loginfo(rospy.get_caller_id() + ' I heard %s', robotpos.header)
  # rospy.loginfo(rospy.get_caller_id() + ' I heard %s', robotpos.pose)
  # pub.publish(last_data)
  leftcampub.publish(leftcam)
  rightcampub.publish(rightcam)
  robotpub.publish(robotpos)
  objectpub.publish(objectpos)

  print('last data published on /status topic')

left_sub = message_filters.Subscriber('/zed_node/left/camera_info_throttle', CameraInfo)
right_sub = message_filters.Subscriber('/zed_node/right/camera_info_throttle', CameraInfo)
robot_sub = message_filters.Subscriber('/Husky/Pose', nav_msgs.msg.Odometry)
object_sub = message_filters.Subscriber('/Box/Pose', nav_msgs.msg.Odometry)
rospy.init_node('camera_synchronizer_node',anonymous=True)

ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub, robot_sub, object_sub], queue_size=10, slop=args.slop)
#ts = message_filters.TimeSynchronizer([info1_sub, info2_sub],queue_size=1)

ts.registerCallback(callback)
rospy.spin()
