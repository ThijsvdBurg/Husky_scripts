#!/usr/bin/env python

import rospy
import argparse
import message_filters
import yaml
import json
from sensor_msgs.msg import Image, CameraInfo
import nav_msgs.msg
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from rospy_message_converter import json_message_converter

parser = argparse.ArgumentParser(description="Sync rosbag topics and rewrite the synced messages into a new rosbag")
parser.add_argument("--slop", type=float, default=0.05, help="Directory containing rosbags.")
args = parser.parse_args()

leftImpub = rospy.Publisher('/sync/left/image_rect', Image, queue_size=1000)
leftInfopub = rospy.Publisher('/sync/left/camera_info', CameraInfo, queue_size=1000)
rightImpub = rospy.Publisher('/sync/right/image_rect', Image, queue_size=1000)
rightInfopub = rospy.Publisher('/sync/right/camera_info', CameraInfo, queue_size=1000)
robotpub = rospy.Publisher('/sync/robot/pose', nav_msgs.msg.Odometry, queue_size=1000)
objectpub = rospy.Publisher('/sync/object/pose', nav_msgs.msg.Odometry, queue_size=1000)


def msg2json(msg):
  ''' Convert a ROS message to JSON format'''
  print('msg type is: ', type(msg) )
  print('type after str(msg) is : ', type( str(msg) ) )
  y = yaml.load(str(msg), Loader=yaml.FullLoader)
  print('type after yaml load is: ', type(y) )
  return json.dumps(y,indent=4)


# When topics are synced, they should be put in a new bag file together
def callback(leftcaminfo,leftcamim,rightcaminfo,rightcamim,robotpos,objectpos):
  # rospy.loginfo(rospy.get_caller_id() + ' I heard %s', leftcam.header)
  leftInfopub.publish(leftcaminfo)
  leftImpub.publish(leftcamim)
  rightInfopub.publish(rightcaminfo)
  rightImpub.publish(rightcamim)
  robotpub.publish(robotpos)
  objectpub.publish(objectpos)
  message1=Float64(leftcaminfo.K)
  #print(type(message1))
  #print(dir(leftcaminfo.header.serialize))
  msg2=leftcaminfo.header.seq
  msg3=msg2json(leftcaminfo.header)
  print(type(msg3),msg3)
  #message2=Int32(leftcaminfo.header.seq)
  #json_str = json_message_converter.convert_ros_message_to_json(message1)
  # print(message1, message2)
  # print('last data published')

leftinfo_sub = message_filters.Subscriber('/zed_node/left/camera_info_throttle', CameraInfo)
leftimage_sub = message_filters.Subscriber('/zed_node/left/image_rect_color_throttle', Image)
rightinfo_sub = message_filters.Subscriber('/zed_node/right/camera_info_throttle', CameraInfo)
rightimage_sub = message_filters.Subscriber('/zed_node/right/image_rect_color_throttle', Image)
robot_sub = message_filters.Subscriber('/Husky/Pose', nav_msgs.msg.Odometry)
object_sub = message_filters.Subscriber('/Box/Pose', nav_msgs.msg.Odometry)
rospy.init_node('camera_synchronizer_node',anonymous=True)

ts = message_filters.ApproximateTimeSynchronizer([leftinfo_sub, leftimage_sub, rightinfo_sub, rightimage_sub, robot_sub, object_sub], queue_size=30, slop=args.slop)
#ts = message_filters.TimeSynchronizer([info1_sub, info2_sub],queue_size=1)

ts.registerCallback(callback)
rospy.spin()
