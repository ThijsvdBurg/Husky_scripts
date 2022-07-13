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
    parser.add_argument("--start", type=int, help="Start index number.")
    parser.add_argument("--end", type=int, help="End index number.")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--suffix", help="Filename suffix (edit, crop or empty).")

    args = parser.parse_args()
    start=args.start
    end=args.end
    source_dir = args.source_dir
    target_dir = args.target_dir

    husky_topic='/Husky/Pose'
    object_topic='/Box/Pose'

    if args.suffix==None:
        suffix=""
    else:
        suffix="_%s" % args.suffix

    print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))

    for i in range(start,end+1):
        print(i)

        outbagpath=os.path.join(target_dir,"%s_sequence_%01i_edit.bag" % (args.date,i))
        # print("Outbagpath is: \n",outbagpath)
        if os.path.exists(outbagpath):
            print("\nOutbagpath exists, skipping %s_sequence_%01i_edit.bag \n" % (args.date,i))
        else:
            # check if sourge bagfile exists first before creating a target bagfile object
            filepath = os.path.join(source_dir,"%s_sequence_%01i%s.bag" % (args.date,i,suffix))
            if not os.path.exists(filepath):
                print("\nInbagpath does not exist, skipping %s_sequence_%01i%s.bag \n" % (args.date,i,suffix))
            else:
                with rosbag.Bag(outbagpath,'w') as outbag:
                    #with rosbag.Bag(os.path.join(target_dir,"%s_sequence_edit.bag" % args.date), 'w') as outbag:
                    # print("Inbagpath",filepath)
                    inbag=rosbag.Bag(filepath)


                    for topic, msg, t in inbag.read_messages():
                        # This also replaces tf timestamps under the assumption
                        # that all transforms in the message share the same timestamp
                        if topic == husky_topic and msg.pose:
                            #print(msg.header.stamp.nsecs)
                            time_sec =t.secs
                            time_nano=t.nsecs
                            # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                            msg.header.stamp.secs  = time_sec
                            msg.header.stamp.nsecs = time_nano
                            outbag.write(husky_topic,msg,t)
                        elif topic == object_topic and msg.pose:
                            #print(msg.header.stamp.nsecs)
                            time_sec =t.secs
                            time_nano=t.nsecs
                            # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                            msg.header.stamp.secs  = time_sec
                            msg.header.stamp.nsecs = time_nano
                            outbag.write(object_topic,msg,t)
                        else:
                        # topic!=husky_topic and topic!=object_topic: # and topic!="/zed_node/left/image_rect_color_throttle" and topic!="/zed_node/right/image_rect_color_throttle":
                            outbag.write(topic,msg,t)

        # with rosbag.Bag(os.path.join(target_dir,"%s_sequence_%01i_edit.bag" % (args.date,i)), 'w') as outbag: 
        #with rosbag.Bag(os.path.join(target_dir,"%s_sequence_edit.bag" % args.date), 'w') as outbag:

                # if topic== "/Bebop2/position_velocity_orientation_estimation" and msg.pose:
                #     #print(msg.header.stamp.nsecs)
                #     time_sec =t.secs
                #     time_nano=t.nsecs
                #     # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                #     msg.header.stamp.secs  = time_sec
                #     msg.header.stamp.nsecs = time_nano
                #     # Write to topic of object, write the (now altered) message at time t
                #     outbag.write(object_topic,msg,t)

                # if topic== "/zed_node/left/camera_info_throttle":
                #     print('/zed_node msg is: \n',msg)
                #     print("i is :\n",i)

        # +=i

if __name__ == "__main__":
	main()
