#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo
# from object_pose_module import common_ops
import cv2
from cv_bridge import CvBridge



def rosmsgs_to_cv2(msg):
    bridge          = CvBridge()
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')



# cv_image = bridge.imgmsg_to_cv2(current_img_ros_msg, desired_encoding="bgra8")
                
def callback(msg, args):
    if args == 'image':
        # latest_ros_image_msg = msg

        cv2im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        print(type(cv2im))


        # # Convert the ROS Image message to a NumPy array
        # np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # assert np_array.shape[0]==720 and np_array.shape[1]==1280 and np_array.shape[2]==4
        # # prev_img = latest_img
        # # print("latest_img['timestamp']",latest_img['timestamp'])
        # # print('image incoming, shape, center pixel RGB values', latest_img['image'].shape, latest_img['image'][360,640,:])

        # latest_img['image'] = np_array.copy()
        # latest_img['timestamp'] = msg.header.stamp
        # return msg
    else:
        latest_caminfo       = msg
        
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    sub_image              = rospy.Subscriber("/zed_node/left/image_rect_color", Image, callback=callback, callback_args='image', queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
