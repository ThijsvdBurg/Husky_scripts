#!/usr/bin/env python
import rospy
import os

import json
import tf
import tf2_ros
from bop_toolkit_lib import inout
import time

#def save_gt(rot, trans, id):
#    scene_gt={}
#    for i in range(0,10):
#        scene_gt[i] = [{
#                    'cam_R_m2c': rot,
#                    'cam_t_m2c': trans,
#                    'obj_id': int(id)
#                }]
#    #print(scene_gt)
#    return scene_gt

def main():
    #TODO:
    #		- Code cleanup
    #		- fix todos below
    #		- FIXED extract row-wise values and check if they are correct
    #           - make timestamp based if construct functioning
    rospy.init_node('json_listener')

    #ROSPARAMETER parsing
    base_frame_id = rospy.get_param('~parent_frame')
    target_frame_id = rospy.get_param('~child_frame')

    #TODO welke werkt (beter?)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfl = tf.TransformListener()
    rate=rospy.Rate(200.0)
    rospy.sleep(1.0)

    scene_num = 2
    obj_id = 2
    image_num = 0

    #TODO change this when actually converting
    scenes_path = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/'
    json_6d_path = os.path.join(scenes_path, f"{scene_num:06}", "scene_gt.json")
    scene_gt = {}

    if os.path.exists(json_6d_path):
        with open(json_6d_path, "r") as gt_scene:
            gt_6d_pose_data = json.load(gt_scene)
    else:
        gt_6d_pose_data = {}
    #gt_6d_pose_data = {}

    with open(json_6d_path, 'w+') as gt_path:
    #if 2>1:
        last_stamp = rospy.Time()
        while not rospy.is_shutdown():
            try:
                now = rospy.Time(0) #rospy.Time.now()
                trans = tfBuffer.lookup_transform(base_frame_id, target_frame_id, now, timeout=rospy.Duration(10.0))
                current_time = rospy.Time.from_sec(time.time())
                nanosec = current_time.to_nsec()

                #if seq==0:
                #    break
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
                rate.sleep()
                print(e)
            #print('Success') #    tf.publish(trans)
            #print('dir data is: ', dir(data))
            if last_stamp == trans.header.stamp:
                #print('continued...')
                continue

            print('trans :\n',trans.header.stamp)

            # bop toolkit annotation lines
            #seq = trans.header.seq
            tf_cam_to_object = tfl.asMatrix(target_frame_id,trans.header)


            translation = list(tf_cam_to_object[0:3, 3 ]*1000   )  # convert meter to mm
            rotation    =      tf_cam_to_object[0:3,0:3].tolist()  # rotation matrix
            #TODO change name
            #scene_gt2 = save_gt(rotation, translation, obj_id)
            scene_gt[image_num] = [{
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
            #print('komt ie nog hier langs na een "continued..."? Alleen als de bagfile draait...')
            image_num+=1
            last_stamp = trans.header.stamp
            #print('timestamp updated')
            rospy.loginfo_throttle(
                    2, "Recorded {} transformations.".format(image_num))

            #TODO waarom publishen?
            #tfpub.publish(trans)
    print('end main')
    inout.save_scene_gt_list(json_6d_path, scene_gt)

if __name__ == '__main__':
    main()
