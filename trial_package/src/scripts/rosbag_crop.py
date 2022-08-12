import rosbag
import bagpy
import argparse
import os

def main():
    """Crop rosbag to first xxx msgs
       Usage:
       python <path_to_this_file> --source_dir <dir containing rosbags> --target_dir <dir to place new cropped rosbags> \
       --start 1 --end 10 --date 20220705 --num_msgs 300
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

    print("Extract data from the directory %s into the directory %s" %(src_dir,tgt_dir))

    max_num = 999
    width=len(str(max_num))
    for i in range(start,end+1):

        print("for loop iteration, i is now: ",i)

        # finding/selecting a rosbag for cropping
        infilename = "%s_sequence_{i:0{width}}.bag".format(i=i,width=width) % args.date
        outfilename = "%s_sequence_{i:0{width}}_crop.bag".format(i=i,width=width) % args.date
        filepath = os.path.join(src_dir,infilename)
        outbagpath = os.path.join(tgt_dir,outfilename)
        #check if the source bagfile exists
        if not os.path.exists(filepath):
            print("\nInbagpath does not exist, skipping ", infilename)

        # Only if it exists, we will try to create a target bagfile
        else:
            # But if target bagfile already exists, we will skip it
            if os.path.exists(outbagpath):
                print("\nOutbagpath \n%s \nalready exists \nSkipping %s_sequence_%01i.bag" % (outbagpath,args.date,i))

            # If it doesn't exist, continue
            else:
                # with rosbag write and path to target bagfile
                with rosbag.Bag(outbagpath, 'w') as outbag:
                    print('Bagfile to be cropped:\n', filepath)

                    # define source rosbag and create rosbag class.
                    # Also print table of topics
                    inbag=rosbag.Bag(filepath)
                    b=bagpy.bagreader(filepath)
                    print(b.topic_table)

                    # cycle through the bag, message for message
                    # reset numsgs
                    numsgs = args.num_msgs
                    for topic, msg, t in inbag.read_messages():
                        if numsgs:
                            if topic!="/joy_teleop/cmd_vel" and topic!="/husky_velocity_controller/cmd_vel": # and topic!="/zed_node/left/image_rect_color_throttle" and topic!="/zed_node/right/image_rect_color_throttle":
                                #print("Is not joy teleop or zed camera image")
                                #print(t)
                                outbag.write(topic,msg,t)
                                #print(topic)
                                #print(numsgs)
                                numsgs-=1

        i+=1

if __name__ == "__main__":
	main()
