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


def assertStamp(current_stamp, first_stamp, bound):
  upperb = first_stamp+bound
  lowerb = first_stamp-bound
  print('first timestamp in bag is:', first_stamp, ' and the current stamp is: ', current_stamp)
  if current_stamp < lowerb:
    print('The current stamp is lower than expected, try sleepbf:=2.0')
  if current_stamp > upperb:
    print('The current stamp is higher than expected, try sleepbf:=1.4')
  assert current_stamp > lowerb and current_stamp < upperb
