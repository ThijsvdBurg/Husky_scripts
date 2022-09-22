#!/usr/bin/env python
import rospy

#import pnp
#import os
#import sys
# Because of transformations
import tf_conversions
import utils
import math
import tf2_ros
#import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


def handle_pose(msg):
    parent_name =   rospy.get_param('~parentname')
    child_name =    rospy.get_param('~childname')
    #print('child_name datatype is: ',type(child_name))
    #print('child_name is: ',(child_name))
    if child_name == 'Husky_zed_left_sensor_link':
        #print('child name contains <sensor>', child_name, 'retrieving focal length from rosparam')
        focal_length =  rospy.get_param('~focal_length')

    if child_name == 'Husky_zed_left_mk_link':
        cam_angle    =  rospy.get_param('~campitch')
    #else:
    #    focal_length =  0

    #print('focal_length is: ', focal_length)    #rospy.init_node('tf2_sub_pub_%s' %child_name )

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    #t = msg
    t.header.stamp = msg.header.stamp #rospy.Time.now()
    t.header.frame_id = parent_name
    t.header.seq = msg.header.seq
    t.child_frame_id = child_name
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1.0

    if child_name == 'Husky_base_link':
        t.transform.translation.x = -0.013881934
        t.transform.translation.y =  0.009032268
        t.transform.translation.z = -0.10728907

    if child_name == 'Husky_zed_left_mk_link':
        t.transform.translation.x = -0.1553935
        t.transform.translation.y =  0.013355832
        t.transform.translation.z =  0.35680165
        # cam_angle = 23.3

        # camera optitrack to opencv convention (converted from deg to radians)
        camroll = (-90-cam_angle) / 360 * 2 * math.pi
        campitch=    0            / 360 * 2 * math.pi
        camyaw  =  -90            / 360 * 2 * math.pi
        # TODO werkt nog niet, op de een of andere stomme manier
        quats = utils.euler_to_quaternion(camroll, campitch, camyaw)
        t.transform.rotation.x = quats[0]
        t.transform.rotation.y = quats[1]
        t.transform.rotation.z = quats[2]
        t.transform.rotation.w = quats[3]
        #t.transform.rotation.x = -0.5906660440898404
        #t.transform.rotation.y = 0.5906660440898404
        #t.transform.rotation.z = -0.38873335894833977
        #t.transform.rotation.w = 0.3887333589483398

    if child_name == 'Husky_zed_right_mk_link':
        # values relative to the left sensor
        t.transform.translation.x = 0.12000
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # values from optitrack calibration
        #t.transform.translation.x = -0.1593366
        #t.transform.translation.y = -0.1050237
        #t.transform.translation.z =  0.3558767

    if child_name == 'Husky_zed_left_sensor_link':
        t.transform.translation.x =  0.0
        t.transform.translation.y =  0.02181
        t.transform.translation.z =  0.0162-focal_length

    if child_name == 'Husky_zed_right_sensor_link':
        t.transform.translation.x =  0.12000
        t.transform.translation.y =  0.0
        t.transform.translation.z =  0.0

    if child_name == 'MMbox_base_link':
        t.transform.translation.x = -0.0091251
        t.transform.translation.y =  0.003978
        t.transform.translation.z = -0.183673
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 1
        t.transform.rotation.w = 6.123233996e-17

        #-0.1593366 -0.1050237 0.3558767  0 0 0 1 Husky_base_link Husky_zed_right2

    #<!-- static transforms below to publish the different coordinate frames (husky base_plate and zed sensors and box centre) -->
    #name="husky_opti_to_base_static_broadcaster" args="-.013881934 .009032268 -.10728907 0 0 0 1 Husky_opti_link Husky_base_link $(arg static_ms)" />
    #name="husky_base_to_zed_right_static_broadcaster" args="-0.1593366 -.1050237 0.3558767  0 0 0 1 Husky_base_link Husky_zed_right2 $(arg static_ms)" />
    #name="husky_base_to_zed_left_static_broadcaster" args="-0.1553935 .013355832 .35680165  0 0 0 1 Husky_base_link Husky_zed_left2 $(arg static_ms)" />
    #name="box_opti_to_top_ctr_static_broadcaster" args="-0.0091251 0.003978 -0.001673 0 0 0 1 MMbox_opti_link MMbox_top_ctr_2 $(arg static_ms)" />
    #name="box_top_ctr_to_ctr_static_broadcaster" args="0 0 -0.182 0 0 0 1 MMbox_top_ctr_2 MMbox_base_link $(arg static_ms)" />


    #msg.pose.pose.position.y
    # = msg.pose.pose.position.z

    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    #if child_name == 'Husky_base_link'
        #br.sendTransform(msg)
    #print('t: ',t.header, t.child_frame_id)
    br.sendTransform(t)

def main():
    rospy.init_node('tf2_sub_pub')
    topic_name =    rospy.get_param('~topicname')

    #rospy.Subscriber('/sync/robot/pose' %topic_name,
    # don't put more than one extra argument for callback in rospy.Subscriber, since it can't differentiate well between positional and named arguments so it will detect positional arguments for the callback as queue size for subscriber
    rospy.Subscriber(topic_name ,
                     Odometry   , # 1st arg msg type which is also parsed to the callback
                     handle_pose # Callback function name
                     )
    rospy.spin()

if __name__ == '__main__':
    main()
