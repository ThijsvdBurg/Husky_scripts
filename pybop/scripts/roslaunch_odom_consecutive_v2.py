#!/usr/bin/env python3
import os
import sys
import roslaunch
import rospy
from rosbag.bag import Bag
import yaml
import json

rospy.init_node("rospylauncher")

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

sleepbf     = os.environ['sleepbf']
syncdelay   = os.environ['sync_delay']
bagdelay    = os.environ['bagdelay']
dummynr     = os.environ['dummynr']
splittype   = 'train_tmp_{}'.format(syncdelay)
sequence_number = '{:06d}'.format(int(os.environ['exp_nr']))

bagdir = os.environ['bagpath']
bagpath = os.path.join(bagdir,'20220705_exp_{}_synced_delay_{}.bag'.format(sequence_number,syncdelay))
print('bag_path',bagpath)
info_dict   = yaml.safe_load(Bag(bagpath,'r')._get_yaml_info())
bagduration = info_dict['duration']
print('bag duration',bagduration)


if not (info_dict['topics'][0]['messages'] == info_dict['topics'][1]['messages'] and info_dict['topics'][0]['messages'] == info_dict['topics'][2]['messages'] and info_dict['topics'][0]['messages'] == info_dict['topics'][3]['messages'] and info_dict['topics'][0]['messages'] == info_dict['topics'][4]['messages']):
  print('number of messages in topics are not the same')
  exit()

tgt_dir = os.path.join(os.environ['BOP_PATH'],'husky',splittype,sequence_number)

if os.path.exists(tgt_dir):
  print('tgt dir ',tgt_dir,' exists , checking whether a scene_gt.json exists and is correct...')
  gt_path = os.path.join(tgt_dir,'scene_gt.json')
  if os.path.exists(gt_path):
    print('scene_gt.json ',gt_path,' exists , checking whether it is larger than 0kB...')
    if os.path.getsize(gt_path) < 1:
      print('scene_gt not successfully generated, overwriting...')
    else:
      print('scene_gt already exists and has filesize > 1 byte, exiting...')
      #print('Output dir already exists, exiting...')
      exit()
  else:
    print('scene_gt does not exist yet, creating...')

launch_file = "/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bop_ros_conversion/launch/odom_to_tf2.launch"

bagdir_arg            = 'bagdir:={}'.format(bagdir)
sequence_number_arg   = 'sequence_number:={}'.format(sequence_number)
sleepbf_arg           = 'sleepbf:={}'.format(sleepbf)
bagdelay_arg          = 'bagdelay:={}'.format(bagdelay)
dummynr_arg           = 'dummynr:={}'.format(dummynr)
syncdelay_arg         = 'sync_delay:={}'.format(syncdelay)
splittype_arg         = 'split_type:={}'.format(splittype)

cli_args = [launch_file,sequence_number_arg,sleepbf_arg,bagdir_arg, bagdelay_arg, dummynr_arg, syncdelay_arg,splittype_arg]
#print('cli_args',cli_args)
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
         uuid,
         roslaunch_file,
    )

launch.start()


rospy.sleep(bagduration+6.5) #-5.5) #+5.5
# xx seconds later, 5.5 works well, sometimes too short
launch.shutdown()
