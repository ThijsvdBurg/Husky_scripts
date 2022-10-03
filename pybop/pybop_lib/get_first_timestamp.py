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
  parser.add_argument("--topic", help="topic.")
  # parser.add_argument("--intrinsics_topic", help="Topic containing CameraInfo.")

  args = parser.parse_args()

  bag = rosbag.Bag(args.bag_file, "r")

  count = 0
  for topic, msg, t in bag.read_messages(topics=[args.topic]):
    if msg.header.seq == 1:
      first_timestamp = msg.header.stamp
    count+=1
    if count == 1:
      break
    bag.close()

  print(first_timestamp)

if __name__ == '__main__':
  main()
