import rosbag
import bagpy
import argparse
import os

def main():
    """Crop rosbag to first xxx msgs
    """
    parser = argparse.ArgumentParser(description="Crop rosbags")
    parser.add_argument("--source_dir", help="Directory containing rosbags.")
    parser.add_argument("--target_dir", help="Output directory.")
    parser.add_argument("--start", type=int, help="Start index number.")
    parser.add_argument("--end", type=int, help="End index number.")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--num_msgs", type=int, help="Number of msgs to retain")


    args = parser.parse_args()
    # make args normal variables for increased simplicity when writing and reading the code
    start=args.start
    end=args.end
    src_dir = args.source_dir
    tgt_dir = args.target_dir
    #numsgs=args.num_msgs

    print("Extract data from the directory %s into the directory %s" %(src_dir,tgt_dir))

    for i in range(start,end+1):
        # finding/selecting a rosbag for cropping
        filepath = os.path.join(src_dir,"%s_sequence_%01i.bag" % (args.date,i))
        outbagpath = os.path.join(tgt_dir,"%s_sequence_%01i_crop.bag" % (args.date,i))
        if not os.path.exists(filepath):
            print("\nInbagpath does not exist, skipping %s_sequence_%01i.bag" % (args.date,i))
        else:
            if os.path.exists(outbagpath):
                print("\nOutbagpath \n%s \nalready exists \nSkipping %s_sequence_%01i.bag" % (outbagpath,args.date,i))
            else:
                with rosbag.Bag(outbagpath, 'w') as outbag:
	                print('Bagfile to be cropped:\n', filepath)

	                # make rosbag class and print table of topics
	                inbag=rosbag.Bag(filepath)
	                b=bagpy.bagreader(filepath)
	                print(b.topic_table)

	                # cycle through the bag, message for message
                        # reset numsgs
                    numsgs = args.num_msgs
                    for topic, msg, t in inbag.read_messages():
                        if numsgs:
                        # if topic=="/joy_teleop/cmd_vel" or topic=="/zed_node/left/image_rect_color_throttle" or topic=="/zed_node/right/image_rect_color_throttle":
                        # if topic=="/joy_teleop/cmd_vel":
                        # print("is joy teleop")
                        # print(t)
                        # numsgs+=1
                        if topic!="/joy_teleop/cmd_vel" and topic!="/husky_velocity_controller/cmd_vel": # and topic!="/zed_node/left/image_rect_color_throttle" and topic!="/zed_node/right/image_rect_color_throttle":
                            #print("Is not joy teleop or zed camera image")
                            #print(t)
                            outbag.write(topic,msg,t)
                            #print(topic)
                            #print(numsgs)
                            numsgs-=1
        i+=1
        print("finished for loop iteration, i has increased to: ",i)

if __name__ == "__main__":
	main()
