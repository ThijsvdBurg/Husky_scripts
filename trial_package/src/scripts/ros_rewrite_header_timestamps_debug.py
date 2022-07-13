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
    if args.suffix==None:
        suffix=""
    else:
        suffix="_%s" % args.suffix

    print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))

    for i in range(start,end+1):
        # print(i)
        outbagpath=os.path.join(target_dir,"%s_sequence_%01i_edit.bag" % (args.date,i))
        # print("Outbagpath is: \n",outbagpath)
        if os.path.exists(outbagpath):
            print("\nOutbagpath exists, skipping %s_sequence_%01i_edit.bag \n" % (args.date,i))
        else:
            with rosbag.Bag(outbagpath,'w') as outbag:
            #with rosbag.Bag(os.path.join(target_dir,"%s_sequence_edit.bag" % args.date), 'w') as outbag:

                filepath = os.path.join(source_dir,"%s_sequence_%01i%s.bag" % (args.date,i,suffix))
                # print("Inbagpath",filepath)
                if not os.path.exists(filepath):
                    print("\nInbagpath does not exist, skipping %s_sequence_%01i%s.bag \n" % (args.date,i,suffix))
                else:
                    inbag=rosbag.Bag(filepath)
                    for topic, msg, t in inbag.read_messages():
                        print("time t is", t)
                        #print("topic is", topic)
                        #print("i is :\n",i)

        # +=i

if __name__ == "__main__":
	main()
