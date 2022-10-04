#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2021 Delft University of Technology

"""Extract images from a rosbag, rosnode implementation
"""

import os
#import argparse
import cv2
import rosbag
import rospy
import sys
from cv_bridge import CvBridge

def main():
  """Extract an image sequence from a rosbag.
  """
  # rospy.init_node()
  rospy.init_node('bag_to_image_node')
  source_dir  =  rospy.get_param('~src')      # path to rosbag
  target_dir  =  rospy.get_param('~tgt')      # path to resulting images
  left_topic  =  rospy.get_param('~lefttop')  # topic name for left image stream
  right_topic =  rospy.get_param('~righttop') # topic name for right image stream
  # parser.add_argument("--bag_file", help="Input ROS bag.")
  # parser.add_argument("--output_dir", help="Output directory.")
  # parser.add_argument("--image_topic", help="Image topic.")
  # parser.add_argument("--intrinsics_topic", help="Topic containing CameraInfo.")

  # args = parser.parse_args()
  bridge = CvBridge()
  
  # print("Extract images from %s on topic %s into %s" % (src,left_topic, target_dir))
  
  bag = rosbag.Bag(source_dir, "r")
  print("Extract images from {} on topic {} into {}".format(source_dir,left_topic,target_dir))
  if not os.access(target_dir,os.W_OK)==True:
    print('target dir not writable, making the directory?')
    if input("Do You Want To Continue? [y/n]") == "y":
      os.mkdir(target_dir)
      print("created output directory")
    else:
      print('aborting')
      sys.exit()
  
  count = 0

  for topic, msg, t in bag.read_messages(topics=[left_topic]):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imwrite(os.path.join(target_dir, "%06i.png" % count), cv_image)
    print("Wrote image %i" % count)
    count+=1

  print("Extract images from {} on topic {} into {}".format(source_dir,right_topic,target_dir))
  count = 0

  for topic, msg, t in bag.read_messages(topics=[right_topic]):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imwrite(os.path.join(target_dir, "right","%06i.png" % count), cv_image)
    print("Wrote right image %i" % count)
    count+=1

  bag.close()

  return

if __name__ == '__main__':
  main()
