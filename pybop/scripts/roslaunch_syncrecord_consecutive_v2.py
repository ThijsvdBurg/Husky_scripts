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

sync_delay      = os.environ['sync_delay']
syncslop        = os.environ['sync_slop']
sequence_number = '{:06d}'.format(int(os.environ['exp_nr']))

bagdir = os.environ['path_to_bags']
bagpath_in = os.path.join(bagdir,'20220705_exp_{}_edit_delay_{}.bag'.format(sequence_number,sync_delay))
bagpath_out = os.path.join(bagdir,'20220705_exp_{}_synced_delay_{}.bag'.format(sequence_number,sync_delay))

info_dict   = yaml.safe_load(Bag(bagpath_in,'r')._get_yaml_info())

bagduration = info_dict['duration']
print('bag_path',bagduration)


# assertion whether the outbag is existing and correctly made
if os.path.exists(bagpath_out):
  print('Output bag already exists, , checking whether it was correctly recorded...')
  info_dict_outbag = yaml.safe_load(Bag(bagpath_out,'r')._get_yaml_info())

  if not (info_dict_outbag['topics'][0]['messages'] == info_dict_outbag['topics'][1]['messages'] and info_dict_outbag['topics'][0]['messages'] == info_dict_outbag['topics'][2]['messages'] and  info_dict_outbag['topics'][0]['messages'] == info_dict_outbag['topics'][3]['messages'] and  info_dict_outbag['topics'][0]['messages'] == info_dict_outbag['topics'][4]['messages']):
    print('Output bag was not created properly, overwriting...')
    #continue
  else:
    print('Output bag already in place and correct, exiting...')
    exit()

launch_file = "/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bop_ros_conversion/launch/record_synced_bags.launch"

bagdir_arg            = 'bagdir:={}'.format(bagdir)
sequence_number_arg   = 'sequence_number:={}'.format(sequence_number)
syncslop_arg          = 'sync_slop:={}'.format(syncslop)
# bagdelay_arg = 'bagdelay:={:06d}'.format(bagdelay)


cli_args = [launch_file,sequence_number_arg,bagdir_arg,syncslop_arg]
print('cli_args',cli_args)
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
         uuid,
         roslaunch_file,
    )

launch.start()

rospy.sleep(bagduration+2)
# xx seconds later
launch.shutdown()
