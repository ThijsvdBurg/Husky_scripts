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
    parser.add_argument("--link_name_robot", help="link name for robot, like opti, base or zed")
    parser.add_argument("--link_name_object", help="link name for object, like opti, box_top or box_centre")
    parser.add_argument("--tf_id_robot", help="tf id name for robot, like opti, box_top or box_centre")

    args = parser.parse_args()
    start=args.start
    end=args.end
    source_dir = args.source_dir
    target_dir = args.target_dir

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
    for i in range(start,end+1):

        outbagpath=os.path.join(target_dir,"%s_sequence_%06i_edit.bag" % (date,i)) # %06i for 6 digits
        if os.path.exists(outbagpath):
            print("\nOutbagpath exists, skipping %s_sequence_%06i_edit.bag \n" % (args.date,i))
        else:
            # check if sourge bagfile exists first before creating a target bagfile object
            filepath = os.path.join(source_dir,"%s_sequence_%06i%s.bag" % (args.date,i,suffix))
            if not os.path.exists(filepath):
                print("\nInbagpath does not exist, skipping %s_sequence_%06i%s.bag \n" % (args.date,i,suffix))
            else:
                with rosbag.Bag(outbagpath,'w') as outbag:
                    #with rosbag.Bag(os.path.join(target_dir,"%s_sequence_edit.bag" % args.date), 'w') as outbag:
                    # print("Inbagpath",filepath)
                    inbag=rosbag.Bag(filepath)
                    # walk through messages
                    for topic, msg, t in inbag.read_messages():
                        if topic == "/mocap_node/Husky/Odom": # and msg.pose:
                            outbag.write(husky_topic,msg,t) # t is the original timestamp, so we keep that
                        elif topic == "/mocap_node/Husky/pose": # and msg.pose:
                            outbag.write(husky_topic,msg,t) # t is the original timestamp, so we keep that
                        if topic == "/mocap_node/Husky/Odom": # and msg.pose:
                            outbag.write(husky_topic,msg,t) # t is the original timestamp, so we keep that
                        elif topic == "/mocap_node/Husky/pose": # and msg.pose:
                            outbag.write(husky_topic,msg,t) # t is the original timestamp, so we keep that
                        # elif topic == "/Bebop2/position_velocity_orientation_estimation" and msg.pose:
                        #    outbag.write(object_topic,msg,t)
                        else: # for /tf
                            if msg.child_frame_id == "Husky/base_link"
                                msg.child_frame_id = "Husky/%s_link" % child_frame_id_robot
                            else:
                                msg.child_frame_id = "Husky/%s_link" % child_frame_id_object
                            outbag.write(topic,msg,t)


        # +=i

if __name__ == "__main__":
	main()
