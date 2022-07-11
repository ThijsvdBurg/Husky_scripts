import rosbag
import argparse

with rosbag.Bag('output1.bag', 'w') as outbag:
    inbag=rosbag.Bag('/home/pmvanderburg/noetic-husky/data_acquisition/data_acq_20220622/rostest/20220705_sequence_108.bag','r')
    for topic, msg, t in inbag.read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/Bebop1/position_velocity_orientation_estimation" and msg.pose:
        #   print(msg.odometry)
            print(msg.header)

            print(t)
           # outbag.write(topic,msg,msg.Odometry[0].header.stamp)
        #else:
        #    print('sacre bleu...')
