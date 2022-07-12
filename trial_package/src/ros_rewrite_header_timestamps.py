import rosbag
import argparse

with rosbag.Bag('output_rewrite_stamp_with_t.bag', 'w') as outbag:
    inbag=rosbag.Bag('/home/pmvanderburg/noetic-husky/data_acquisition/data_acq_20220622/rostest/20220705_sequence_108.bag','r')
    target_topic='/Husky/Pose'
    for topic, msg, t in inbag.read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/Bebop1/position_velocity_orientation_estimation" and msg.pose:
            #print(msg.header.stamp.nsecs)
            time_sec =t.secs
            time_nano=t.nsecs
            msg.header.stamp.secs = time_sec
            outbag.write(target_topic,msg,t)
            #outbag.write(target_topic,msg,msg.header.stamp)
        #else:
        #    print('sacre bleu...')
