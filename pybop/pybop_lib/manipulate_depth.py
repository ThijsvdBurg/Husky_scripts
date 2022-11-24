
# author: pmvanderburg

import os
# import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from bop_toolkit_lib import inout
from bop_toolkit_lib import misc
from bop_toolkit_lib import visualization
from pybop_lib.debug_tools import printdebug


def vis_depth(
      rgb=None, depth=None, vis_rgb_path=None,
      vis_depth_diff_path=None, vis_depth_diff_path_debug=None, vis_rgb_resolve_visib=False):
  """Visualizes 3D object models in specified poses in a single image.

  Two visualizations are created:
  1. An RGB visualization (if vis_rgb_path is not None).
  2. A Depth-difference visualization (if vis_depth_diff_path is not None).

  :param poses: List of dictionaries, each with info about one pose:
    - 'obj_id': Object ID.
    - 'R': 3x3 ndarray with a rotation matrix.
    - 't': 3x1 ndarray with a translation vector.
    - 'text_info': Info to write at the object (see write_text_on_image).
  :param K: 3x3 ndarray with an intrinsic camera matrix.
  :param renderer: Instance of the Renderer class (see renderer.py).
  :param rgb: ndarray with the RGB input image.
  :param depth: ndarray with the depth input image.
  :param vis_rgb_path: Path to the output RGB visualization.
  :param vis_depth_diff_path: Path to the output depth-difference visualization.
  :param vis_rgb_resolve_visib: Whether to resolve visibility of the objects
    (i.e. only the closest object is visualized at each pixel).
  """

  # Calculate the depth difference at pixels where both depth maps are valid.
  depth_mask = 255*visualization.depth_for_vis(depth)
  depth_mask[depth_mask < 0.0000000000001] = 70
  #ren_rgb_f[ren_rgb_f > 255] = 255
  depth_n = depth.astype(np.float)
  print('max depthn',depth_n.max())
  print('min depthn',depth_n.min())
  print('max depthmask',depth_mask.max())
  print('min depthmask',depth_mask.min())
  #valid_mask = (depth > 0) * (ren_depth > 0)

  # DEBUG / introspection
  inout.save_im(vis_depth_diff_path_debug, depth_mask.astype(np.uint8)) #depth_diff_vis)
