from urllib import robotparser
import rosbag
import argparse
import os
import rospy

def main():
    """Rewrite bagfile header stamps to syncronize the zed camera with optitrack data
    """

    parser = argparse.ArgumentParser(description="Rewrite header stamps and put them in new bag file")
    parser.add_argument("--source_dir", help="Directory containing rosbags.")
    parser.add_argument("--target_dir", help="Output directory.")
    parser.add_argument("--start", type=int, help="Start index number.")
    parser.add_argument("--end", type=int, help="End index number.")
    parser.add_argument("--sync_delay", type=int, default='0', help="Optitrack time delay in sec*e-6 which has to be compensated.")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--suffix", help="Filename suffix (edit, crop or empty).")


    args = parser.parse_args()
    start=args.start
    end=args.end
    source_dir = args.source_dir
    target_dir = args.target_dir

    #topics:      /Bebop1/position_velocity_orientation_estimation   1215 msgs    : nav_msgs/Odometry
    #             /Bebop2/position_velocity_orientation_estimation   1215 msgs    : nav_msgs/Odometry
    #             /husky_velocity_controller/cmd_vel                 1549 msgs    : geometry_msgs/Twist
    #             /joy_teleop/cmd_vel                                1553 msgs    : geometry_msgs/Twist
    #             /zed_node/left/camera_info_throttle                  57 msgs    : sensor_msgs/CameraInfo
    #             /zed_node/left/image_rect_color_throttle             56 msgs    : sensor_msgs/Image
    #             /zed_node/right/camera_info_throttle                 56 msgs    : sensor_msgs/CameraInfo
    #             /zed_node/right/image_rect_color_throttle            56 msgs    : sensor_msgs/Image

    # the topic 1 and 2 are the expected incoming topic names
    topic1   =		'/Bebop1/position_velocity_orientation_estimation'
    topic2   =		'/Bebop2/position_velocity_orientation_estimation'
    topic3   =          '/zed_node'
    topic3_1 =          '/zed_node/left/camera_info_throttle'
    topic3_2 =          '/zed_node/left/image_rect_color_throttle'
    topic3_3 =          '/zed_node/right/camera_info_throttle'
    topic3_4 =          '/zed_node/right/image_rect_color_throttle'
    topic4   =          '/husky_velocity_controller/cmd_vel'
    topic5   =          '/joy_teleop/cmd_vel'
    #topic1 =		'/tfstamped'
    #topic2 =		'/Bebop1/position_velocity_orientation_estimation'

    # new (more logical) names for topics, instead of bebop1 and bebop2
    topic1_new   =          '/sync/Husky/pose'
    topic2_new   =	    '/sync/MMbox/pose'
    topic3_1_new =	    '/sync/left/camera_info'
    topic3_2_new =	    '/sync/left/image_rect'
    topic3_3_new =          '/sync/right/camera_info'
    topic3_4_new =          '/sync/right/image_rect'
    topic5_new   =          '/sync/cmd_vel'
    # prevent filename suffix to be filled to "None" when no suffix arg is supplied
    if args.suffix==None:
        suffix=""
    else:
        suffix="_%s" % args.suffix

    print("\nExtract data from the directory %s into the directory %s" %(source_dir,target_dir))


    # iterature through the start to the end rosbag, dependent on the argument "start" and "end"
    for i in range(start,end+1):
        name_outbag = "%s_exp_%06i_edit_delay_%i_nosync.bag" % (args.date,i,args.sync_delay)
        name_inbag  = "%s_exp_%06i%s.bag"    % (args.date,i,suffix)
        outbagpath=os.path.join(target_dir,name_outbag)
        # print("Outbagpath is: \n",outbagpath)
        if os.path.exists(outbagpath):
            print("\nOutbagpath exists, skipping ", name_outbag)
        else:
            # check if sourge bagfile exists first before creating a target bagfile object
            filepath = os.path.join(source_dir,name_inbag)
            if not os.path.exists(filepath):
                print("\nInbagpath does not exist, skipping ",name_inbag)
            else:
                print('Converting ',name_inbag,' to ', name_outbag)
                with rosbag.Bag(outbagpath,'w') as outbag:
                    #with rosbag.Bag(os.path.join(target_dir,"%s_exp_edit.bag" % args.date), 'w') as outbag:
                    # print("Inbagpath",filepath)
                    inbag=rosbag.Bag(filepath)
                    # walk through messages
                    for topic, msg, t in inbag.read_messages():
                        #if topic == topic1 or topic == topic2 or topic.__contains__(topic3) or topic == topic4 or topic == topic5:
                        if topic == topic1 or topic == topic2:
                            #print(topic) #'topic ',topic,' contains ', topic3) #msg.header.stamp.nsecs)
                            # Rewrite correct header stamps to the 'faulty' Husky stamps, since that was the main machine on which was recorded, so that is easier.
                            #if not args.sync_delay: # or topic.__contains__(topic3):
                            #print('writing normal msg clock stamp to topic ', topic)
                            #msg.header.stamp = t # will keep the stamp of the Husky clock which was used to record the bagfile on
                                # the modification below will keep the msg.header timestamps, uncomment whichever suits your needs
                                # t=msg.header.stamp
                            #else:
                            #    print('writing advanced msg clock stamp to topic ', topic,'\noriginal clock t:',t)
                            msg.header.stamp = t + rospy.Duration(args.sync_delay / 1000000)
                            #    print('header stamp is now:',msg.header.stamp)

                            if topic == topic1:
                                topic_new = topic1_new
                            if topic == topic2:
                                topic_new =topic2_new
                            outbag.write(topic_new,msg,t)
                        if topic.__contains__(topic3) or topic == topic4 or topic == topic5:
                            if topic == topic3_1:
                                topic_new = topic3_1_new
                            if topic == topic3_2:
                                topic_new = topic3_2_new
                            if topic == topic3_3:
                                topic_new = topic3_3_new
                            if topic == topic3_4:
                                topic_new = topic3_4_new
                            if topic == topic4:
                                topic_new = topic4
                            if topic == topic5:
                                topic_new =topic5_new
                            outbag.write(topic_new,msg,t)

if __name__ == "__main__":
    main()
