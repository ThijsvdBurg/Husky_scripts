import rosbag
import argparse

num_msgs=500
inbag=rosbag.Bag('/home/pmvanderburg/noetic-husky/data_acquisition/data_acq_20220622/rostest/20220622_sequence_6.bag','r')
with rosbag.Bag('output.bag', 'w') as outbag:
    for topic, msg, t in inbag.read_messages():
        while num_msgs:
            outbag.write(topic, msg, t)
            num_msgs -= 1
