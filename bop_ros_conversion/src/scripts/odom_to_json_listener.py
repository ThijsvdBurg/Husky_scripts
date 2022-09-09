#!/usr/bin/env python
import rospy
import os

import json
import tf
import tf2_ros
from bop_toolkit_lib import inout
import time

class ROSTransformerHandler (object):
    tform = tf.TransformerROS()
    #static_tfs = {}

    def get_transform_matrix(self, frame_a, frame_b, tfb):
        #t = rospy.Time(0)
        t = self.tform.getLatestCommonTime(frame_a, frame_b)
        #print('latestcommontime  =',t2)
        #trans, rot = self.tform.lookupTransform(frame_a, frame_b, t)
        trans = tfb.lookup_transform(frame_a, frame_b, t, timeout=rospy.Duration(10.0))
        #mat = self.tform.fromTranslationRotation(trans, rot) # only for TransformerROS
        #mat = mat.tolist()
        #except (tf.LookupException, tf.Exception):
        #    return []
        #print(mat)
        #print(trans)
        return trans

def mkdir(path):
    os.mkdir(path)

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
    scene_num = rospy.get_param('~sequence_number')
    lookup_rate = rospy.get_param('~lookup_rate')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfl = tf.TransformListener()
    rate=rospy.Rate(lookup_rate)
    rospy.sleep(1.0)
    obj_id = 1



    #TODO change this when actually converting
    scenes_directory = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/'
    scenes_path =  os.path.join(scenes_directory, f"{scene_num:06}")
    json_6d_path = os.path.join(scenes_path, "scene_gt.json")
    scene_gt = {}
    ROS_TRANSFORMER_HANDLER = ROSTransformerHandler()
    image_num = 0
    gt_6d_pose_data = {}

    if os.path.exists(json_6d_path):
        with open(json_6d_path, "r") as gt_scene:
            gt_6d_pose_data = json.load(gt_scene)
    elif not os.path.exists(scenes_path):
        print('folder does not exist yet...\ncreating')
        mkdir(scenes_path)

    with open(json_6d_path, 'w+') as gt_path:
    #if 2>1:
        last_stamp = rospy.Time()
        while not rospy.is_shutdown():
            try:
                #now = rospy.Time(0) #rospy.Time.now()
                t = tfBuffer.get_latest_common_time(base_frame_id, target_frame_id)
                #print(t)
                transform = tfBuffer.lookup_transform(base_frame_id, target_frame_id, t, timeout=rospy.Duration(10.0))
                #current_time = rospy.Time.from_sec(time.time())
                #nanosec = current_time.to_nsec()
                #trans = ROS_TRANSFORMER_HANDLER.get_transform_matrix(base_frame_id, target_frame_id, tfBuffer)
                #print(transform)
                #if seq==0:
                #    break
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
                rate.sleep()
                print(e)
            if last_stamp == transform.header.stamp:
                #print('continued...')
                continue

            # bop toolkit annotation lines
            #seq = trans.header.seq
            tf_cam_to_object = tfl.asMatrix(target_frame_id,transform.header)


            translation = list(tf_cam_to_object[0:3, 3 ]*1000   )  # convert meter to mm
            rotation    =      tf_cam_to_object[0:3,0:3].tolist()  # rotation matrix
            #TODO change name
            #scene_gt2 = save_gt(rotation, translation, obj_id)
            scene_gt[image_num] = [{
                'header_stamp': str(transform.header.stamp),
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
            #print('komt ie nog hier langs na een "continued..."? Alleen als de bagfile draait...')
            image_num+=1
            last_stamp = transform.header.stamp
            #print('timestamp updated')
            rospy.loginfo_throttle(
                    1, "Recorded {} transformations.".format(image_num))

            #TODO waarom publishen?
            #tfpub.publish(trans)
    print('end main')
    rospy.loginfo("Recorded {} transformations.".format(image_num))
    inout.save_scene_gt_list(json_6d_path, scene_gt)

if __name__ == '__main__':
    main()
