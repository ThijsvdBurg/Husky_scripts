#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2022 Delft University of Technology

"""Get first timestamp from a rosbag.
"""

import os
import argparse
import rosbag
import sys

def main():
  """Extract an image sequence from a rosbag.
  """
  parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")

  parser.add_argument("--bag_file", help="Input ROS bag.")
  #parser.add_argument("--output_dir", help="Output directory.")
  parser.add_argument("--image_topic", help="Image topic.")
  # parser.add_argument("--intrinsics_topic", help="Topic containing CameraInfo.")

  args = parser.parse_args()

  print("Extract images from %s on topic %s into %s" % (args.bag_file,args.image_topic, args.output_dir))

  bag = rosbag.Bag(args.bag_file, "r")
  
  if not os.access(args.output_dir,os.W_OK)==True:
    print('target directory not available/writable, requiring user input...')
  if input("Do You Want me to create the directory? [y/n]") == "y":
    os.mkdir(args.output_dir)
    print("created output directory")
  else:
    print('aborting')
    sys.exit()
  
  count = 0
  for topic, msg, t in bag.read_messages(topics=[args.topic]):
    if topic.header.seq == 1:
      first_timestamp = topic.header.stamp
    count+=1
    if count == 1:
      break
    bag.close()

  return first_timestamp

if __name__ == '__main__':
  main()
