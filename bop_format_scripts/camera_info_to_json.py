#!/usr/bin/env python

from rospy_message_converter import json_message_converter
from std_msgs.msg import String
import ros_msg


# code from Julian and Andras
def parse_ROS_msg(self, ros_msg, ros_time):

        # if the frame is static
        IS_STATIC = self.name == 'tf_static'
        if IS_STATIC:
            for tf_msg in ros_msg.transforms:
                ROS_TRANSFORMER_HANDLER.add_ROS_message_tf_static(tf_msg)
        else:
            for tf_msg in ros_msg.transforms:
                ROS_TRANSFORMER_HANDLER.add_ROS_message_tf(tf_msg)
        # load from config, or use default values
        base_frame_id = self.config['base_frame_id']
        target_frame_ids = self.config['target_frame_ids']
        # All past static tfs should remain valid once they have been received.
        # To ensure that the TF Transformer will take the most recent timestamp for
        # both the target and base frames, we readd the static TF messages with
        # the updated timestamp
        ROS_TRANSFORMER_HANDLER.update_static_tfs(ros_time)
        # determine tranformations
        transforms = ROS_TRANSFORMER_HANDLER.get_transformation_matrices(base_frame_id, target_frame_ids)
        # construct data
        data = {
            '_timestamp_ns': ros_time.to_nsec(),
            'header': {
                'seq': self.msg_count,
                'stamp' : {
                    'secs': ros_time.secs,
                    'nsecs': ros_time.nsecs,
                },
                'frame_id': base_frame_id,
            },
            'transforms': transforms,
        }
        self.msg_count += 1
        # FIXME: use util_add_ros_timestamp or set timestamp directly above?
        #util_update_header_with_config(data, self.config)
        util_add_ros_timestamp_to_data_dict(data, ros_msg, ros_time)
        gen_msg = self.MsgClass(self.msg_type, base_path=self.ds_dir)
        gen_msg.set_data( data )
        return gen_msg


