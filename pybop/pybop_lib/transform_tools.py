#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2022 Delft University of Technology

"""tools for tf conversions
"""

# import os
# import argparse
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import math


def transform_init():
  """Init an empty C{geometry_msgs/Transform} 

  @return:
    - tf: empty TransformStamped
  """
  tf = TransformStamped()
  tf.transform.translation.x = 0
  tf.transform.translation.y = 0
  tf.transform.translation.z = 0
  tf.transform.rotation.x    = 0
  tf.transform.rotation.y    = 0
  tf.transform.rotation.z    = 0
  tf.transform.rotation.w    = 1.0
  return tf

def transform_to_pq(msg):
  """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
  q = np.array([msg.rotation.x, msg.rotation.y,
                msg.rotation.z, msg.rotation.w])
  return p,q

def transformStampedTopq(msg):
  """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  return transform_to_pq(msg.transform)

def pose_to_pq(msg):
  """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  p = np.array([msg.position.x, msg.position.y, msg.position.z])
  q = np.array([msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w])
  return p,q

def Odometry_to_pq(msg):
  """Convert a C{nav_msgs/Odometry} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  return pose_to_pq(msg.pose.pose)

def Pose_to_Rt(msg):
  """Convert a C{geometry_msgs/Pose} into full 4x4 rotation and translation matrix

  @param msg: ROS message to be converted
  @return:
    - Rt: full 4x4 rotation and translation matrix
  """
  p,q = pose_to_pq(msg)
  return pqToRotationMatrix(p,q)

def PoseStamped_to_pq(msg):
  """Convert a C{geometry_msgs/PoseStamped} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  return pose_to_pq(msg.pose)

def PoseStamped_Odometry_to_Rt(msg):
  """Convert a C{geometry_msgs/PoseStamped} or C{nav_msgs/Odometry} into full 4x4 rotation and translation matrix

  @param msg: ROS message to be converted
  @return:
    - Rt: full 4x4 rotation and translation matrix
  """
  p,q = pose_to_pq(msg.pose)
  return pqToRotationMatrix(p,q)

def PoseStamped_Odometry_to_Rt(msg):
  """Convert a C{geometry_msgs/PoseStamped} or C{nav_msgs/Odometry} into full 4x4 rotation and translation matrix

  @param msg: ROS message to be converted
  @return:
    - Rt: full 4x4 rotation and translation matrix
  """
  p,q = pose_to_pq(msg.pose)
  return pqToRotationMatrix(p,q)

def quatToRotationMatrix(quat_np):
  """Convert quaternion array to 4x4 transformation matrix
  @param quat_np: quaternion to be converted (order=[x,y,z,w])
  @return rotM: transformation matrix 4x4
  """
  rotM = tf.transformations.quaternion_matrix(quat_np)
  return rotM

def pqToRotationMatrix(p,q):
  """Convert translation and quaternion vector numpy array to 4x4 rotation matrix
  @param 
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  @return Rt: full 4x4 rotation and translation matrix
  """
  Rt = quatToRotationMatrix(q)
  Rt[0,3] = p[0]
  Rt[1,3] = p[1]
  Rt[2,3] = p[2]
  return Rt

def isRotationMatrix(R):
  Rt = np.transpose(R)
  shouldBeI = np.dot(Rt,R)
  I = np.identity(3, dtype= R.dtype)
  n = np.linalg.norm(I - shouldBeI)
  return n < 1e-6

def rotationMatrixToEuler(R):
  print(R)
  R = R[0:3,0:3]

  assert(isRotationMatrix(R))
  sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
  singular = sy < 1e-6
  if  not singular :
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], sy)
    z = math.atan2(R[1,0], R[0,0])
  else :
    x = math.atan2(-R[1,2], R[1,1])
    y = math.atan2(-R[2,0], sy)
    z = 0
  return np.array([x, y, z])

def rotationMatrixToEulerDeg(R):
  eulerRad = rotationMatrixToEuler(R)
  roll  = eulerRad[0] * 360 / math.pi
  pitch = eulerRad[1] * 360 / math.pi
  yaw   = eulerRad[2] * 360 / math.pi

  return np.array([roll, pitch, yaw])

def cam3DToUV(intrins, Xc, Yc, Zc):
  '''
  @param:
    - intrins: the camera intrinsics
    - Xc, Yc, Zc: the 3D world coordinates, expressed wrt the camera frame
  @return u,v: pipxel coordinate locations corresponding to the world coordinates
  '''
  camvec = np.array([Xc,Yc,Zc])
  uvw = np.dot( intrins, camvec )
  u = uvw[0] / uvw[2]
  v = uvw[1] / uvw[2]
  print('u: ',u)
  print('v: ',v)
  return u, v

def TF_to_azi_alti(t_vec):
  '''
  @param:
    - t_vec: translation from camera frame, camera coordinate is opencv convention (Positive Z forward)
    - Xc, Yc, Zc: the 3D world coordinates, expressed wrt the camera frame
  @return azimuth, altitude angles of the point described by the translation vector
  '''

  # Compute azimuth and altitude angles

  dist = np.linalg.norm
