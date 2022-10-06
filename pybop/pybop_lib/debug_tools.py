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

def printDirTypeValue(object):
  print('dir:',dir(object))
  print('type:',type(object))
  print('value:',object)

def printMap(map):
  listed = list(map)
  print(listed)
  print(dir(listed))
