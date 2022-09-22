#!/usr/bin/env python
import math
import numpy as np
def euler_to_quaternion(r,p,y):
    import numpy as np # Scientific computing library for Python
    """
    Convert an Euler angle to a quaternion.
    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
    qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
    qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
    qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)

    return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def main():

  x = -0.004428309854120016
  y = 0.20557308197021484
  z = 0.0028261321131139994
  w = 0.978627622127533
  print('values x y z and w from input: \n', x ,'\n', y ,'\n', z ,'\n', w)
  roll,pitch,yaw = euler_from_quaternion(x,y,z,w)
  print('roll: ', roll,'\npith', pitch ,'\nyaw', yaw)
  rolld = 360* roll / ( 2*math.pi )
  pitchd = 360* pitch / ( 2*math.pi )
  yawd = 360* yaw / ( 2*math.pi )
  print('roll (deg): ', rolld,'\npitch (deg)', pitchd ,'\nyaw(deg)', yawd)
  # camera roll pitch yaw to quats

  cam_angle = 23.3



  # camera optitrack to opencv convention
  #camroll = (-90-cam_angle) / 360 * 2 * math.pi
  #campitch= (0) / 360 * 2 * math.pi
  #camyaw  = -90 / 360 * 2 * math.pi

  # box yaw to face the camera
  camroll = 0 / 360 * 2 * math.pi
  campitch= 0 / 360 * 2 * math.pi
  camyaw  = 180 / 360 * 2 * math.pi

  quats = euler_to_quaternion(camroll, campitch, camyaw)
  print(quats)

if __name__ == '__main__':
  main()
