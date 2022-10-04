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

# #scene_camera[im_id] = {
#         'cam_K': dp_camera['K'],
#         'depth_scale': dp_camera['depth_scale'],
#         'view_level': int(views_level[view_id])
#       }

# def gt_list(tf, rotation, translation, obj_id):
#     cam_info = [{
#                 'cam_K': k_matrix,
#                 'cam_R_m2c': rotation,
#                 'cam_t_m2c': translation,
#                 'obj_id': int(obj_id)
#                  }]
#     return scene_gt

def extractCamInfo(msg):
  K = msg.K
  u = msg.height
  v = msg.width
  depth_scale = 1.0
  cam_info = msg_to_info(K,u,v,depth_scale)
  return cam_info
