#!/usr/bin/env python

import numpy as np


# TODO:
#    - methods with i/o
#    - load json 
def msg_to_info(K, u, v, depth_scale):
  #print(type(K))
  #print(K)
  fx, fy, cx, cy = K[0], K[4], K[2], K[5]
  #im_size = (u, v)

  cam = {
    'im_size': (u,v),
    'K': np.array([[fx,   0.0,   cx],
                   [0.0,   fy,   cy],
                   [0.0,  0.0,   1.0]])
        }
  cam['depth_scale'] = float(depth_scale)

  return cam

def extractCamInfo(msg):
  K = msg.K
  u = msg.height
  v = msg.width
  depth_scale = 1.0
  cam_info = msg_to_info(K,u,v,depth_scale)

  scene_camera = {
        'cam_K': cam_info['K'],
        'depth_scale': cam_info['depth_scale']
      }
  return scene_camera
