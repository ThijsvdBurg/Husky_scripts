
import numpy as np
import rosbag

from pybop_lib import bag_to_camerainfo

def main():
  #bagpath = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/2022-09-27-07-23-59_rw3.bag'
  bagpath = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220705/20220705_exp_000100_synced.bag'
  #bagpath = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220705/20220705_exp_000102_synced.bag'
  bag = rosbag.Bag(bagpath, "r")
  left_info = {}
  right_info = {}
  left_cam =  '/sync/left/camera_info'
  right_cam ='/sync/right/camera_info'
  countcamleft =0
  countcamright=0
  for topic, msg, t in bag.read_messages(topics=[left_cam, right_cam]):
    if topic == left_cam:
      left_info[countcamleft] = bag_to_camerainfo.extractCamInfo(msg)
      countcamleft+=1
    else:
      right_info[countcamright] = bag_to_camerainfo.extractCamInfo(msg)
      countcamright+=1

  #print()

if __name__ == "__main__":
  main()
