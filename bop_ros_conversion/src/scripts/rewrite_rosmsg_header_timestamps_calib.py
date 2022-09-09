from urllib import robotparser
import rosbag
import argparse
import os

def main():
    """Rewrite bagfile header stamps to syncronize the zed camera with optitrack data
    """

    parser = argparse.ArgumentParser(description="Rewrite header stamps and put them in new bag file")
    parser.add_argument("--source_dir", help="Directory containing rosbags.")
    parser.add_argument("--target_dir", help="Output directory.")

    args = parser.parse_args()
    source_dir = args.source_dir
    target_dir = args.target_dir


    print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))


    # iterature through the start to the end rosbag, dependent on the argument "start" and "end"
    for i in range(1,2):

        outbagpath=os.path.join(target_dir)
        # print("Outbagpath is: \n",outbagpath)
        if os.path.exists(outbagpath):
            print("\nOutbagpath exists, skipping... \n")
        else:
            # check if sourge bagfile exists first before creating a target bagfile object
            filepath = os.path.join(source_dir)
            if not os.path.exists(filepath):
                print("\nInbagpath does not exist, skipping...")
            else:
                with rosbag.Bag(outbagpath,'w') as outbag:
                    #with rosbag.Bag(os.path.join(target_dir,"%s_exp_edit.bag" % args.date), 'w') as outbag:
                    # print("Inbagpath",filepath)
                    inbag=rosbag.Bag(filepath)

                    # walk through messages
                    for topic, msg, t in inbag.read_messages():
                        #if topic == topic1:
                            #print(msg.header.stamp.nsecs)
                            # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                        msg.header.stamp=t  #will keep the stamp of the computer's clock which was used to record the bagfile on
                            # the modification below will keep the msg.header timestamps, comment whichever suits your needs
                            # t=msg.header.stamp
                        outbag.write(topic,msg,t)
                        #uncomment next section if you want to rewrite the second topic
                        #elif topic == topic2:
                        #    #print(msg.header.stamp.nsecs)
                        #    # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                        #    msg.header.stamp  = t
                        #    outbag.write(object_topic,msg,t)
                        #else: # to make all other topics join the new rosbag
                        #    outbag.write(topic,msg,t)


        # +=i

if __name__ == "__main__":
	main()
