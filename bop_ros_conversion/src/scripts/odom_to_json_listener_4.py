#!/usr/bin/env python
import rospy
import os
import json
import tf
import tf2_ros
from bop_toolkit_lib import inout
import time
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

def between_two_numbers(num,a,b):
    if b < a:
        a, b = b, a
    if num in range(a,b):
        return True
    else:
        return False

def check_discrepancy(current_tf,last_tf):
    disc_x  = current_tf.transform.translation.x - last_tf.transform.translation.x
    disc_y  = current_tf.transform.translation.y - last_tf.transform.translation.y
    disc_z  = current_tf.transform.translation.z - last_tf.transform.translation.z
    disc_qx = current_tf.transform.rotation.x    - last_tf.transform.rotation.x
    disc_qy = current_tf.transform.rotation.y    - last_tf.transform.rotation.y
    disc_qz = current_tf.transform.rotation.z    - last_tf.transform.rotation.z
    disc_qw = current_tf.transform.rotation.w    - last_tf.transform.rotation.w
    trans   = np.array([abs(disc_x),abs(disc_y),abs(disc_z)])
    rot     = np.array([abs(disc_qx),abs(disc_qy),abs(disc_qz),abs(disc_qw)])
    discrepancy = math.sqrt( np.dot(trans,trans) + np.dot(rot,rot) ) * 1000
    return discrepancy

def transform_init():
    tf = TransformStamped()
    tf.transform.translation.x = 0
    tf.transform.translation.y = 0
    tf.transform.translation.z = 0
    tf.transform.rotation.x    = 0
    tf.transform.rotation.y    = 0
    tf.transform.rotation.z    = 0
    tf.transform.rotation.w    = 1
    return tf

def mkdir(path):
    os.mkdir(path)

def main():
    rospy.init_node('json_listener_tt')

    #ROSPARAMETER parsing
    base_frame_id = rospy.get_param('~parent_frame')
    target_frame_id = rospy.get_param('~child_frame')
    scene_num = rospy.get_param('~sequence_number')
    lookup_rate = rospy.get_param('~lookup_rate')
    sleep_before =  rospy.get_param('~sleeptime')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfl = tf.TransformListener()
    rate=rospy.Rate(lookup_rate)
    rospy.sleep(sleep_before)
    obj_id = 1

    #TODO change this when actually converting
    scenes_directory = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/'
    scenes_path =  os.path.join(scenes_directory, f"{scene_num:06}")
    json_6d_path = os.path.join(scenes_path, "scene_gt_tt.json")
    #json_6d_filter_path = os.path.join(scenes_path, "scene_gt_tt_filter.json")
    scene_gt = {}
    scene_eigen_tf = {}
    #scene_gt_filter = {}
    image_num = 0
    gt_6d_pose_data = {}
    discrepancy_threshold_upper = .001 # to filter the static transforms
    discrepancy_threshold = .0001      # to filter the duplicate transforms

    if os.path.exists(json_6d_path):
        with open(json_6d_path, "r") as gt_scene:
            gt_6d_pose_data = json.load(gt_scene)
    elif not os.path.exists(scenes_path):
        print('folder does not exist yet...\ncreating')
        mkdir(scenes_path)

    with open(json_6d_path, 'w+') as gt_path:
        last_stamp = rospy.Time()
        last_transform = transform_init()
        dummy_tf       = transform_init()

        while not rospy.is_shutdown():
            try:
                #print('trying...')
                t = tfBuffer.get_latest_common_time(base_frame_id, target_frame_id)
                #past = t - rospy.Duration(.2)
                past = rospy.Time.now() - rospy.Duration(.2)
                rate.sleep()
                transform = tfBuffer.lookup_transform_full(
                        target_frame=base_frame_id,
                        target_time=t,
                        source_frame=base_frame_id,
                        #source_time=past,
                        source_time=last_stamp, #t-rospy.Duration(.2),
                        #source_time=last_stamp,
                        fixed_frame='world',
                        timeout=rospy.Duration(1.0)
                        )
                #print(transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
                rate.sleep()
                print(e)

            stampdiff = transform.header.stamp - last_stamp
            stampdiff_sec = stampdiff.to_sec() # floating point
            #stampdiff_nsec = stampdiff.to_nsec()
            #print('type of stampdiff is : ',type(stampdiff_sec),type(stampdiff_nsec))
            #print('stampdiff is : ',stampdiff_sec,stampdiff_nsec)

            stampdiff_int = stampdiff_sec * 1000000000 #convert to nsecs for readability)

            if last_stamp == transform.header.stamp or stampdiff_int < 10000000:
                #print('continued...')
                continue

            transform_mag = check_discrepancy(transform, dummy_tf)
            scene_eigen_tf[image_num] = [{
                'header_stamp': str(transform.header.stamp),
                'header_diff': stampdiff_int, #str(transform.header.stamp - last_stamp),
                'transform_magnitude': str(transform_mag),
                 }]

            #scene_gt[image_num] = [{
            #    'header_stamp': str(trans.header.stamp),
            #    'cam_R_m2c': rotation,
            #    'cam_t_m2c': translation,
            #    'obj_id': int(obj_id)
            #     }]
            #print('komt ie nog hier langs na een "continued..."? Alleen als de bagfile draait...')

            image_num+=1
            last_stamp = transform.header.stamp

            #print('timestamp updated')
            rospy.loginfo_throttle(
                    1, "Recorded {} transformations.".format(image_num))

    print('end main')
    rospy.loginfo("Recorded {} transformations.".format(image_num))
    inout.save_scene_gt_list(json_6d_path, scene_eigen_tf)


if __name__ == '__main__':
    main()
