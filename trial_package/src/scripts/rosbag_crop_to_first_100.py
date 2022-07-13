import rosbag
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
    parser.add_argument("--num_msgs", help="Number of msgs to retain")


    args = parser.parse_args()
    # make args normal variables for increased simplicity when writing and reading the code
    start=args.start
    end=args.end
    src_dir = args.source_dir
    tgt_dir = args.target_dir
    numsgs=args.num_msgs

    print("Extract data from the directory %s into the directory %s" %(src_dir,tgt_dir))

    for i in range(start,end+1):
        print(i)
        with rosbag.Bag(os.path.join(tgt_dir,"%s_sequence_%01i_crop.bag" % (args.date,i)), 'w') as outbag:

            # finding/selecting a rosbag for cropping
            filepath = os.path.join(src_dir,"%s_sequence_%01i.bag" % (args.date,i))
            print('Bagfile to be cropped:\n', filepath)
            # make rosbag class
            inbag=rosbag.Bag(filepath)

            # cycle through the bag, message for message
            for topic, msg, t in inbag.read_messages():

                print('/zed_node msg is: \n',msg)
                print("i is :\n",i)


        # +=i

if __name__ == "__main__":
	main()
