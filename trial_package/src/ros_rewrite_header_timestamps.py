import rosbag
import argparse

with rosbag.Bag('output.bag', 'w') as outbag:
    inbag=rosbag.Bag('/home/pmvanderburg/noetic-husky/data_acquisition/data_acq_20220622/rostest/20220622_sequence_6.bag','r')
    for topic, msg, t in inbag.read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/Bebop1/pose" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
