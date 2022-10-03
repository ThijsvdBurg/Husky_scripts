#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2022 Delft University of Technology

"""Get first timestamp from a rosbag.
"""

# import os
# import argparse
import rosbag
# import sys

def get_Timestamp(bagpath,topic_name):
  """Extract an image sequence from a rosbag.
  """

  bag = rosbag.Bag(bagpath, "r")

  count = 0
  for topic, msg, t in bag.read_messages(topics=[topic_name]):
    if msg.header.seq == 1:
      first_timestamp = msg.header.stamp
    count+=1
    if count == 1:
      break
    bag.close()

  return first_timestamp  

if __name__ == '__main__':
  main()
