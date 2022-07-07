#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2021 Delft University of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2
import rosbag
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

def main():
    """Extract an image sequence from a rosbag to be used in SC-SfM-learner.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")

    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--output_dir", help="Output directory.")
    parser.add_argument("--image_topic", help="Image topic.")
    # parser.add_argument("--intrinsics_topic", help="Topic containing CameraInfo.")

    args = parser.parse_args()

    print("Extract images from %s on topic %s into %s" % (args.bag_file,args.image_topic, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    if os.access(args.output_dir,os.W_OK)==True:
        for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # intrinsics =
            cv2.imwrite(os.path.join(args.output_dir, "%06i.png" % count), cv_image)
            print("Wrote image %i" % count)
            count+=1
            #if count==2:
            #    break
    else:
	    print('target dir not writable')


    bag.close()

    return

if __name__ == '__main__':
    main()
