import rosbag
import argparse

num_msgs=10
# inbag=rosbag.Bag('/home/pmvanderburg/noetic-husky/data_acquisition/data_acq_20220622/rostest/20220622_sequence_6.bag','r')
inbag=rosbag.Bag('/home/pmvanderburg/output_msg_header_stamp.bag','r')
with rosbag.Bag('output_trial.bag', 'w') as outbag:
    for topic, msg, t in inbag.read_messages():
        while num_msgs:
            print('time t is: ',t, '\n and message is:\n',msg)
            #outbag.write(topic, msg, t)
            num_msgs -= 1
