#!/usr/bin/env python
import os
import json

import argparse
import math
import cv2
import numpy as np

from bop_toolkit_lib import inout
from bop_toolkit_lib import config
from bop_toolkit_lib import dataset_params

p = {
  # See dataset_params.py for options.
  'dataset': 'tudl',

  # Dataset split. Options: 'train', 'val', 'test'.
  'dataset_split': 'test',

  # Dataset split type. None = default. See dataset_params.py for options.
  'dataset_split_type': None,

  # Whether to save visualizations of visibility masks.
  'vis_visibility_masks': False,

  # Tolerance used in the visibility test [mm].
  'delta': 15,

  # Type of the renderer.
  'renderer_type': 'vispy',  # Options: 'vispy', 'cpp', 'python'.

  # Folder containing the BOP datasets.
  'datasets_path': config.datasets_path,

  # Path template for output images with object masks.
  'vis_mask_visib_tpath': os.path.join(
    config.output_path, 'vis_gt_visib_delta={delta}',
    'vis_gt_visib_delta={delta}', '{dataset}', '{split}', '{scene_id:06d}',
    '{im_id:06d}_{gt_id:06d}.jpg'),
}
################################################################################

# Load dataset parameters.
dp_split = dataset_params.get_split_params(
  p['datasets_path'], p['dataset'], p['dataset_split'], p['dataset_split_type'])

#print(dp_split)

scene_ids = dataset_params.get_present_scene_ids(dp_split)
print(scene_ids)
for scene_id in scene_ids:
  print(scene_id)

  if scene_id-1 != last_id:
    print('image id number was skipped :)')
  scene_gt = inout.load_scene_gt(
    dp_split['scene_gt_tpath'].format(scene_id=scene_id))

  scene_gt_info = {}
  im_ids = sorted(scene_gt.keys())
  last_id = scene_id

