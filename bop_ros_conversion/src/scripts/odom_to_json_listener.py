#!/usr/bin/env python
import rospy
import os
from os import mkdir

import json
import tf
import tf2_ros

from bop_toolkit_lib import inout
from pybop_lib.stamp_utils import assertStamp
from pybop_lib.stamp_utils import getFirstStamp
from pybop_lib.debug_tools import printdebug
from pybop_lib import transform_tools as tt

import time
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

# TODO move to pybop utils
def filter_list(imnum, headerstamp,mag):
    scene_gt_filter = [{
                'image_number': imnum,
                'header_stamp': str(headerstamp),
                'tf_magnitude': str(mag)
                 }]
    return scene_gt_filter
def gt_list(tf, rotation, translation, obj_id):
    scene_gt = [{
                'header_stamp': str(tf.header.stamp),
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
    return scene_gt
def aux_list(tf_stamp, last_stamp, mag):
    scene_gt_aux = [{
                   'header_stamp': str(tf_stamp),
                   'header_diff': str(tf_stamp - last_stamp),
                   'transform_magnitude': str(mag),
                   }]
    return scene_gt_aux

def transformMagnitude(current_tf, last_tf, multiplier):
    disc_x  = current_tf.transform.translation.x - last_tf.transform.translation.x
    disc_y  = current_tf.transform.translation.y - last_tf.transform.translation.y
    disc_z  = current_tf.transform.translation.z - last_tf.transform.translation.z
    disc_qx = current_tf.transform.rotation.x    - last_tf.transform.rotation.x
    disc_qy = current_tf.transform.rotation.y    - last_tf.transform.rotation.y
    disc_qz = current_tf.transform.rotation.z    - last_tf.transform.rotation.z
    disc_qw = current_tf.transform.rotation.w    - last_tf.transform.rotation.w
    trans   = np.array([abs(disc_x) ,abs(disc_y) ,abs(disc_z)              ])
    rot     = np.array([abs(disc_qx),abs(disc_qy),abs(disc_qz),abs(disc_qw)])
    transform_mag = math.sqrt( np.dot(trans,trans) + np.dot(rot,rot) ) * multiplier
    return transform_mag

def transform_init():
    tf = TransformStamped()
    tf.transform.translation.x = 0
    tf.transform.translation.y = 0
    tf.transform.translation.z = 0
    tf.transform.rotation.x    = 0
    tf.transform.rotation.y    = 0
    tf.transform.rotation.z    = 0
    tf.transform.rotation.w    = 1.0
    return tf



def main():
    #TODO:
    #		- Code cleanup
    rospy.init_node('json_listener')

    #ROSPARAMETER parsing
    base_frame_id   = rospy.get_param('~parent_frame')
    target_frame_id = rospy.get_param('~child_frame')
    bagpath         = rospy.get_param('bagpath')
    scene_num       = rospy.get_param('sequence_number')
    lookup_rate     = rospy.get_param('~lookup_rate')
    sleep_before    = rospy.get_param('~sleeptime')
    DEBUG           = rospy.get_param('~debug')
    data_path       = rospy.get_param('dataset_path')
    split_type      = rospy.get_param('split_type')
    delay           = rospy.get_param('sync_delay')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfl = tf.TransformListener()
    rate=rospy.Rate(lookup_rate)
    rospy.sleep(sleep_before)
    obj_id = 1

    #TODO change this when actually converting
    scenes_directory = data_path #'/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220705'
    if delay:
        scenes_path =  os.path.join(scenes_directory, split_type, f"{scene_num:06}",'delay_{}'.format(delay))
    else:
        scenes_path =  os.path.join(scenes_directory, split_type, f"{scene_num:06}")
    json_6d_path = os.path.join(scenes_path, "scene_gt.json")
    json_6d_aux_path = os.path.join(scenes_path, "scene_gt_aux.json")
    json_6d_filter_path = os.path.join(scenes_path, "scene_gt_filter.json")
    scene_gt         = {}
    scene_filter     = {}
    scene_aux        = {}
    # gt_6d_pose_data  = {}

    # -2 accounts for the two dummy transforms which are expected to be recorded to kick-start the pipeline
    image_num = -2
    discrepancy_multiplier = 1000
    discrepancy_threshold_upper = 2.000 # max threshold to filter the static transforms

    topic_name='/sync/Husky/pose'
    # get first timestamp from bagfile to compare to the 0th recorded transform, also get total number of messages to use for assertion later
    first_stamp, num_msgs = getFirstStamp(bagpath, topic_name)

#    if os.path.exists(json_6d_path):
#        with open(json_6d_path, "r") as gt_scene:
#            gt_6d_pose_data = json.load(gt_scene)
#    elif not os.path.exists(scenes_path):
#        print('folder does not exist yet...\ncreating')
#        mkdir(scenes_path)

    if os.path.exists(json_6d_path):
        print('output file already existed, overwriting')
    if not os.path.exists(scenes_path):
        print('folder does not exist yet...\ncreating')
        mkdir(scenes_path)

    with open(json_6d_path, 'w+') as gt_path:
        last_stamp1 = rospy.Time()
        last_stamp2 = rospy.Time()
        dummy_tf       = transform_init()
        #print(last_transform)
        while not rospy.is_shutdown():
            try:
                t = tfBuffer.get_latest_common_time(base_frame_id, target_frame_id)
                #past = t - rospy.Duration(.2) #
                if DEBUG:
                    print('latest common time is:',t)

                transform_cam_object = tfBuffer.lookup_transform(
                    base_frame_id,
                    target_frame_id,
                    t,
                    timeout=rospy.Duration(1.0)
                )

                if DEBUG:
                    print('transform from cam to object',transform_cam_object)
                # transform from previous robot location to the current TODO check if these values are closely related to the excel values
                transform_base = tfBuffer.lookup_transform_full(
                    target_frame=base_frame_id,
                    target_time=t,
                    source_frame=base_frame_id,
                    source_time=last_stamp1,
                    fixed_frame='world',
                    timeout=rospy.Duration(1.0)
                )
                rate.sleep()
                current_stamp = transform_cam_object.header.stamp
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
                rate.sleep()
                print(e)

            stampdiff = current_stamp - last_stamp1
            stampdiff_sec = stampdiff.to_sec() # floating point
            # stampdiff_nsec = stampdiff.to_nsec()
            # print('type of stampdiff is : ',type(stampdiff_sec),type(stampdiff_nsec))
            # print('stampdiff is : ',stampdiff_sec,stampdiff_nsec)

            stampdiff_int = stampdiff_sec * 1000000000
            if DEBUG:
                    print('current stamp:',current_stamp)
                    print('stamp difference:',stampdiff)
                    print('tf base to prev timestep:',transform_base)

            if last_stamp1 == current_stamp or stampdiff_int < 50000000: # or current_stamp == 1655219049426688671:
                #print('continued...')
                continue

            transform_magnitude = transformMagnitude(transform_base, dummy_tf, discrepancy_multiplier)

            # updating iamge_num here is necessary since we wish to have a filtered list of static transformations
            # with unique values for each image id
            image_num+=1

            # gets updated already here since we need an updated last_stamp for when the script reaches past the first continue if construct
            last_stamp1 = current_stamp

            if transform_magnitude < discrepancy_threshold_upper and image_num > 0:
                print('skipped because of static robot, movement magnitude is ', transform_magnitude, ' which is lower than ',discrepancy_threshold_upper)
                scene_filter[image_num] = filter_list(image_num, current_stamp,transform_magnitude)
                continue
            # to automatically check if the first timestamp is corresponding with the timestamp found at image_num = 0,
            # we assert it here wrt an upper and lower bound, which is the original first timestamp ±25 million nanoseconds (rospy duration 0.025)
            if image_num == 0:
                # for 000077 we needed a slightly larger bound assertStamp(current_stamp, first_stamp, rospy.Duration(.045))
                assertStamp(current_stamp, first_stamp, rospy.Duration(.025))



            #printdebug('translation',translation)
            p,q = tt.transformStampedTopq(transform_cam_object) #convert to position and quaternion numpy arrays

            transform_matrix = tt.pqToRotationMatrix(p,q)
            #printdebug('transform_matrix',transform_matrix)
            translation = list(transform_matrix[0:3, 3 ]*1000   )  # convert meter to mm
            rotation    =      transform_matrix[0:3,0:3].tolist()  # rotation matrix
            #translation = list(trans_np*1000    )           # convert np array to list and meter to mm
            #printdebug('translation',translation)
            #rotation    =      list(rot_np       )           # rotation matrix
            #printdebug('rotation',rotation)


            # we need a second last_stamp to be able to generate valid stampdiff values for the auxiliary list (containing information to monitor the process)
            # laststamp2 gets updated after the construction of scene_aux to prevent stampdiff to return 0
            if image_num >= 0:
                #scene_filter[image_num] = filter_list(image_num, current_stamp, transform_magnitude)
                scene_gt    [image_num] = gt_list(transform_cam_object, rotation, translation, obj_id)
                scene_aux   [image_num] = aux_list(current_stamp, last_stamp2, transform_magnitude)

                last_stamp2 = current_stamp

            rospy.loginfo_throttle(
                    1, "Recorded {} transformations at last timestamp {}.".format(image_num, last_stamp2))

    print('end main')
    rospy.loginfo("Recorded {} transformations. We expected {}.".format(image_num+1, num_msgs))
    #if image_num+1 == num_msgs
    assert image_num+1 == num_msgs
    inout.save_scene_gt_list(json_6d_path       , scene_gt    )
    inout.save_scene_gt_list(json_6d_filter_path, scene_filter)
    inout.save_scene_gt_list(json_6d_aux_path   , scene_aux   )
    #inout.save_scene_gt_list(json_caminfo_path  , camera_scene)

if __name__ == '__main__':
    main()

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
