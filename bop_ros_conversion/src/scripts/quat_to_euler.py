#!/usr/bin/env python
import math

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
  roll,pitch,yaw = euler_from_quaternion(x,y,z,w)
  print('roll: ', roll,'\npith', pitch ,'\nyaw', yaw)
  rolld = 360* roll / ( 2*math.pi )
  pitchd = 360* pitch / ( 2*math.pi )
  yawd = 360* yaw / ( 2*math.pi )
  print('roll (deg): ', rolld,'\npitch (deg)', pitchd ,'\nyaw(deg)', yawd)


if __name__ == '__main__':
  main()
