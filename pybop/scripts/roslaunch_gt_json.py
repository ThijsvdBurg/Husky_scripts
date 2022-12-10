#!/usr/bin/env python3
import os
import sys
#import roslaunch
#import rospy
#from rosbag.bag import Bag
#import yaml
#import json

#rospy.init_node("rospylauncher")

sync_delay  = os.environ['sync_delay']

sequence_number = '{:06d}'.format(int(os.environ['exp_nr']))
bagdir = os.environ['bagpath']
bagpath = os.path.join(bagdir,'20220705_exp_{}_synced_delay_{}.bag'.format(sequence_number,sync_delay))
split_type = os.environ['split_type']
print('bag_path',bagpath)
#info_dict   = yaml.safe_load(Bag(bagpath,'r')._get_yaml_info())
#bagduration = info_dict['duration']
#print('bag_path',bagduration)

json_broken_path = os.path.join(os.environ['BOP_PATH'],'husky',split_type,'broken_missing.txt')
#broken_missing_json = []

sequence_number = str('{:06}'.format(int(sequence_number))) #+500000)

target_gt_dir = os.path.join(os.environ['BOP_PATH'],'husky',split_type,sequence_number )

with open(json_broken_path, 'a') as broken_missing_list:
  if os.path.exists(target_gt_dir):
    print('path exists for', target_gt_dir)
    gt_path = os.path.join(target_gt_dir,'scene_gt.json')
    if os.path.exists(gt_path):
      print('filesize of gt json of ', bagpath, ' is ', os.path.getsize(gt_path))
      if (os.path.getsize(gt_path) < 1):
        print('scene_gt not converted properly')
        msg = '{} is 0 kB\n'.format(sequence_number)
        broken_missing_list.write(msg)
    else:
      print('scene_gt does not exist yet')
      msg = '{} does not exist\n'.format(sequence_number)
      broken_missing_list.write(msg)
  else:
    print('path ',target_gt_dir, ' does not exist :(')
  broken_missing_list.close()
