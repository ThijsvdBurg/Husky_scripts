from urllib import robotparser
import rosbag
import argparse
import os
import geometry_msgs.msg
import nav_msgs.msg
import tf2_msgs.msg

def main():
    """Rewrite bagfile header stamps to syncronize the zed camera with optitrack data
    """

    parser = argparse.ArgumentParser(description="Rewrite header stamps and put them in new bag file")
    parser.add_argument("--source_dir", help="Directory containing rosbags.")
    parser.add_argument("--bagfile", help="Directory containing rosbags.")

    parser.add_argument("--target_dir", help="Output directory.")
    parser.add_argument("--start", type=int, help="Start index number.")
    parser.add_argument("--end", type=int, help="End index number.")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--suffix", help="Filename suffix (edit, crop or empty).")
    parser.add_argument("--link_name_robot", help="link name for robot, like opti, base or zed")
    parser.add_argument("--link_name_object", help="link name for object, like opti, box_top or box_centre")
    #parser.add_argument("--tf_id_robot", help="tf id name for robot, like opti, box_top or box_centre")

    args = parser.parse_args()
    start=args.start
    end=args.end
    source_dir = args.source_dir
    target_dir = args.target_dir

    topic1 = '/Husky/Pose'
    topic2 = '/MMbox/Pose'
    max_k = args.end

    # new (more logical) names for topics, instead of bebop1 and bebop2
    husky_pose_topic='/mocap_node/Husky/%s_link/pose' % args.link_name_robot
    husky_odom_topic='/mocap_node/Husky/%s_link/Odom'  % args.link_name_robot
    object_pose_topic='/mocap_node/MMbox/%s_link/pose' % args.link_name_object
    object_odom_topic='/mocap_node/MMbox/%s_link/Odom' % args.link_name_object
    child_frame_id_robot='%s_link' % args.link_name_robot
    child_frame_id_object='%s_link' % args.link_name_object

    # prevent filename suffix to be filled to "None" when no suffix arg is supplied
    if args.suffix==None:
        suffix=""
    else:
        suffix="_%s" % args.suffix


    print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))

    # iterature through the start to the end rosbag, dependent on the argument "start" and "end"
    #for i in range(start,end+1):

    #infilename="%s_exp_%06i%s.bag" % (args.date,i,suffix)
    #filepath = os.path.join(source_dir,infilename)
    filepath = args.bagfile
    # check if sourge bagfile exists first before creating a target bagfile object
    if not os.path.exists(filepath):
        print("\nInbagpath does not exist, skipping %s \n" % (filepath))
    else:
        # print("Inbagpath",filepath)
        inbag=rosbag.Bag(filepath)
        print(inbag)
        # walk through messages
        k = 0
        for topic, msg, t in inbag.read_messages():
            if topic == '/mocap_node/MMbox/Odom' or topic == '/mocap_node/MMbox/pose' or topic == '/tf':
                print('time: ',t)
                print('topic: ',topic)
                print('msg: \n',msg)
                if k == max_k:
                    break
                k+=1

        # +=i

if __name__ == "__main__":
	main()
