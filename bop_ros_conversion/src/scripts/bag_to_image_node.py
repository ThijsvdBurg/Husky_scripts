#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2021 Delft University of Technology

"""Extract images from a rosbag, rosnode implementation
"""

import os
from turtle import right
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
  source_dir       =  rospy.get_param('bagpath')          # path to rosbag
  scenes_directory =  rospy.get_param('dataset_path')     # path to resulting images
  scene_num        =  rospy.get_param('sequence_number')  # sequence/experiment number
  split_type       =  rospy.get_param('split_type')       # export to train, val or test directory
  left_topic       =  rospy.get_param('~lefttop')         # topic name for left image stream
  right_topic      =  rospy.get_param('~righttop')        # topic name for right image stream
  left_cam         =  rospy.get_param('~leftcamtop')      # topic name for left image stream
  right_cam        =  rospy.get_param('~rightcamtop')     # topic name for right image stream
  # parser.add_argument("--bag_file", help="Input ROS bag.")
  # parser.add_argument("--output_dir", help="Output directory.")
  # parser.add_argument("--image_topic", help="Image topic.")
  # parser.add_argument("--intrinsics_topic", help="Topic containing CameraInfo.")

  # args = parser.parse_args()
  bridge = CvBridge()

  # print("Extract images from %s on topic %s into %s" % (src,left_topic, scenes_path))

  bag = rosbag.Bag(source_dir, "r")

  scenes_path       = os.path.join(scenes_directory, split_type, f"{scene_num:06}", "rgb"      )
  scenes_path_right = os.path.join(scenes_directory, split_type, f"{scene_num:06}", "rgb_right")

  print("Extract images from {} on topic {} into {}".format(source_dir,left_topic,scenes_path))

  if not os.path.exists(scenes_path):
  #if not os.access(scenes_path,os.W_OK)==True:
    print('target dir not existing, creating the directory')
    os.makedirs(scenes_path)
    os.makedirs(scenes_path_right)
    print("created {}".format(scenes_path))

    left_info = {}
    right_info = {}
    countl = 0
    countr = 0
    countcamleft = 0
    countcamright = 0

    for topic, msg, t in bag.read_messages(topics=[left_topic, right_topic, left_cam, right_cam]):
      cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
      if topic == left_topic:
        cv2.imwrite(os.path.join(scenes_path,      "%06i.png" % countl), cv_image)
        countl+=1
        print("Wrote left image %i" % countl)
      elif topic == right_topic:
        cv2.imwrite(os.path.join(scenes_path_right,"%06i.png" % countr), cv_image)
        countr+=1
        print("Wrote right image %i" % countr)
      elif topic == left_cam:
        # cv2.imwrite(os.path.join(scenes_path_right,"%06i.png" % countr), cv_image)
        countcamleft+=1
        
        # print("Wrote right image
      else:
        right_info[countcamright] = extract_cam_info(msg)
        countcamright+=1

  else:
    print('path exists, assuming that images exist inside, aborting')
    sys.exit()

  bag.close()

  return

if __name__ == '__main__':
  main()
