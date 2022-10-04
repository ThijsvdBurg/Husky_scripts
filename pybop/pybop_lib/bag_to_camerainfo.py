#!/usr/bin/env python

import numpy as np


# TODO:
#    - methods with i/o
c = load_json(path)

cam = {
    'im_size': (c['width'], c['height']),
    'K': np.array([[fx,   0.0,   cx],
                   [0.0,   fy,   cy],
                   [0.0,  0.0,   1.0]])
  }

  if 'depth_scale' in c.keys():
    cam['depth_scale'] = float(c['depth_scale'])

  return cam

scene_camera[im_id] = {
        'cam_K': dp_camera['K'],
        'depth_scale': dp_camera['depth_scale'],
        'view_level': int(views_level[view_id])
      }

def gt_list(tf, rotation, translation, obj_id):
    cam_info = [{
                'cam_K': k_matrix,
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
    return scene_gt

def extractCamInfo(msg):
  K = msg.K
  