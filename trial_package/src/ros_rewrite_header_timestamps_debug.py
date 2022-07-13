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

    args = parser.parse_args()
    start=args.start
    end=args.end
    source_dir = args.source_dir
    target_dir = args.target_dir

    print("Extract data from the directory %s into the directory %s" %(source_dir,target_dir))

    for i in range(start,end+1):
        print(i)
        with rosbag.Bag(os.path.join(target_dir,"%s_sequence_%01i_edit.bag" % (args.date,i)), 'w') as outbag: 
        #with rosbag.Bag(os.path.join(target_dir,"%s_sequence_edit.bag" % args.date), 'w') as outbag:

            filepath = os.path.join(source_dir,"%s_sequence_%01i.bag" % (args.date,i))
            print(filepath)
            inbag=rosbag.Bag(filepath)



            for topic, msg, t in inbag.read_messages():
                print('/zed_node msg is: \n',msg)
                print("i is :\n",i)

        # +=i

if __name__ == "__main__":
	main()
