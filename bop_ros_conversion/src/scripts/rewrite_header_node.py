#!/usr/bin/env python
from urllib import robotparser
import rosbag
import rospy
import os

def main():
  """ROS node to rewrite bagfile header stamps to create a dummy bagfile to trick tf lookup
  """
  rospy.init_node('rewrite_header')
  source_dir =     rospy.get_param('bagpath')  # path to rosbag
  target_dir =     rospy.get_param('~tgt')  # path to resulting dummybag
  crop       =     rospy.get_param('~crop') # bool if to crop the resulting dummybag (to save disk space)
  crop_max   =     10*6                     # 10 messages for 6 topics

  print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))


  outbagpath=os.path.join(target_dir)
  # print("Outbagpath is: \n",outbagpath)
  if os.path.exists(outbagpath):
      print("\nOutbagpath exists, skipping... \n")
  else:
    # check if source bagfile exists first before creating a target bagfile object
    filepath = os.path.join(source_dir)
    if not os.path.exists(filepath):
      print("\nInbagpath does not exist, skipping...")
    else:
      with rosbag.Bag(outbagpath,'w') as outbag:
        #with rosbag.Bag(os.path.join(target_dir,"%s_exp_edit.bag" % args.date), 'w') as outbag:
        # print("Inbagpath",filepath)
        inbag=rosbag.Bag(filepath)
        i = 0
        # walk through messages
        for topic, msg, t in inbag.read_messages():
          #if topic == topic1:
          if i == crop_max+1:
            break
          #print('before',msg.header.stamp.secs)
          #msg.header.stamp.secs=msg.header.stamp.secs-3  #will subtract 1 from orig timestamp to create dummy values for tf
          msg.header.stamp.secs=msg.header.stamp.secs-10  #will subtract 1 from orig timestamp to create dummy values for tf
          #print('after',msg.header.stamp.secs)
          #msg.header.stamp=t  #will keep the stamp of the computer's clock which was used to record the bagfile on
            # the modification below will keep the msg.header timestamps, comment whichever suits your needs
            # t=msg.header.stamp
          outbag.write(topic,msg,t)
          if crop:
            i+=1
          #uncomment next section if you want to rewrite the second topic
          #elif topic == topic2:
          #    #print(msg.header.stamp.nsecs)
          #    # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
          #    msg.header.stamp  = t
          #    outbag.write(object_topic,msg,t)
          #else: # to make all other topics join the new rosbag
          #    outbag.write(topic,msg,t)



if __name__ == "__main__":
  main()
