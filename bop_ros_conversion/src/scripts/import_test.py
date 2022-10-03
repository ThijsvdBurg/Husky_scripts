#!/usr/bin/env python

#from quat_to_euler.py import angleConv
#import quat_to
#import quat_to_euler.py
# import euler_to_quaternion
import math
from geometry_msgs.msg import TransformStamped
#from bop_toolkit_lib import inout
from pybop_lib import inout
#import pybop_lib

#print(dir(quat_to_euler))

#i = 0

#t = TransformStamped()
#cam_angle = 23.31
# camera optitrack to opencv convention (converted from deg to radians)
#camroll = (-90-cam_angle) / 360 * 2 * math.pi
#campitch=    0            / 360 * 2 * math.pi
#camyaw  =  -90            / 360 * 2 * math.pi
# TODO werkt nog niet, op de een of andere stomme manier
#quats = quat_to_euler.euler_to_quaternion(camroll, campitch, camyaw)

#t.transform.rotation.x = quats[0]
#t.transform.rotation.y = quats[1]
#t.transform.rotation.z = quats[2]
#t.transform.rotation.w = quats[3]

#print(t)

print(dir(inout))
