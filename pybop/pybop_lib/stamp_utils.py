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

def getFirstStamp(bagpath,topic_name):
  """Extract the first header stamp and the number of messages in a topic.
  """

  bag = rosbag.Bag(bagpath, "r")

  count = 0
  for topic, msg, t in bag.read_messages(topics=[topic_name]):
    if msg.header.seq == 1:
      first_timestamp = msg.header.stamp
    count+=1
    #print('')
  bag.close()
  return first_timestamp, count


def assertStamp(current_stamp, first_stamp, bound):
  upperb = first_stamp+bound
  lowerb = first_stamp-bound
  print('first timestamp in bag is:', first_stamp, ' and the current stamp is: ', current_stamp)
  print('lowerbound and upperbound are:', lowerb, upperb)
  if current_stamp < lowerb:
    print('The current stamp is lower than expected, \n\n try increasing sleepbf, default is 1.7 \n\n')
  if current_stamp > upperb:
    print('The current stamp is higher than expected, \n\n\ try decreasing sleepbf, default is 1.7 \n\n')
  assert current_stamp > lowerb and current_stamp < upperb
