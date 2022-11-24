#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2022 Delft University of Technology

"""tools for debugging code easily
"""

# import os
# import argparse
import rosbag
# import sys

def printdebug(name,object):
  #print(name,' dir:\n',dir(object))
  print(name,' type:\n',type(object))
  print(name,' value:\n',object,'\n\n')

def printMap(map):
  listed = list(map)
  print(listed)
  print(dir(listed))

def printMaxMin(depth_mm):
  print('depth mm max:',depth_mm.max())
  print('depth mm min:',depth_mm.min())
