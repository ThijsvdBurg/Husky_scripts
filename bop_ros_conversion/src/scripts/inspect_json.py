#!/usr/bin/env python
import os
import numpy as np
#import bop_toolkit_lib
import json
import yaml
from bop_toolkit_lib import inout


def load_json(path, keys_to_int=False):
  """Loads content of a JSON file.
  :param path: Path to the JSON file.
  :return: Content of the loaded JSON file.
  """
  # Keys to integers.
  def convert_keys_to_int(x):
    return {int(k) if k.lstrip('-').isdigit() else k: v for k, v in x.items()}

  with open(path, 'r') as f:
    if keys_to_int:
      #content_json = json.load(f, object_hook=lambda x: convert_keys_to_int(x))
      #content = yaml.safe_load(f, object_hook=lambda x: convert_keys_to_int(x))
      content = json.load(f, object_hook=lambda x: convert_keys_to_int(x))
    else:
      content_json = json.load(f)
      content = yaml.safe_load(f)
  #print(content_json)
  #print(content)
  return content

def inspect_json(scene_gt):
    print('type scene_gt is: \n',type(scene_gt))
    imgnum = 0
    for im_id, im_gt in scene_gt.items():
        print('im_id is: \n',im_id)
        for gt in im_gt:
            if 'cam_R_m2c' in gt.keys():
                print('before converting to nparray:\n',gt['cam_R_m2c'],'\n',type(gt['cam_R_m2c']))
                gt['cam_R_m2c'] = np.array(gt['cam_R_m2c'], float).reshape((3, 3))
                #print('After converting to nparray:\n',gt['cam_R_m2c'], '\n',type(gt['cam_R_m2c']))
            if 'cam_t_m2c' in gt.keys():
                print('before converting to nparray:\n',gt['cam_t_m2c'],'\n',type(gt['cam_t_m2c']))
                gt['cam_t_m2c'] = np.array(gt['cam_t_m2c'], float).reshape((3, 1))
                #print('After converting to nparray:\n',gt['cam_t_m2c'], '\n',type(gt['cam_t_m2c']))
        #if imgnum>1:
        #    break
        imgnum+=1
    return scene_gt


def main():
    #path = '/home/pmvanderburg/noetic-husky/data_acquisition/bop_datasets/tudl/train_real/000001/scene_gt.json'
    #path = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/000001/scene_gt.json'
    path = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/000002/scene_gt.json'
    scene_gt = load_json(path,keys_to_int=True)
    #print(scene_gt.keys())
    scene_gt_new=inspect_json(scene_gt)

if __name__ == '__main__':
    main()



