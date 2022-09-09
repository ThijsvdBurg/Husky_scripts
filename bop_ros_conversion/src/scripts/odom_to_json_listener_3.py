#!/usr/bin/env python
import rospy
import os

#import json
import tf
# import tf2_ros
# from bop_toolkit_lib import inout
# import time

class ROSTransformerHandler():
    print('init')
    tform = tf.Transformer()
    static_tfs = {}

    def get_transform_matrix(self, frame_a, frame_b):
        try:
            mat = []
            t = rospy.Time(0) #
            #t = self.tform.getLatestCommonTime(frame_a, frame_b)
            #print('latestcommontime  =',t)
            trans,rot = self.tform.lookupTransform(frame_a, frame_b, t)
            print('trans is: ',trans)
            mat = self.tform.fromTranslationRotation(trans, rot) # only for TransformerROS
            mat = mat.tolist()
            print(mat)
        except (tf.LookupException, tf.Exception) as e:
            print(e)
        return mat


def main():
    #TODO:
    #		- Code cleanup
    #		- fix todos below
    #		- FIXED extract row-wise values and check if they are correct
    #           - make timestamp based if construct functioning
    rospy.init_node('json_listener')

    #ROSPARAMETER parsing
    base_frame_id = 'world'
    target_frame_id = 'Husky/base_link'

    rate=rospy.Rate(30.0)
    rospy.sleep(0.50)

    ROS_TRANSFORMER_HANDLER = ROSTransformerHandler()

    while not rospy.is_shutdown():
        trans = ROS_TRANSFORMER_HANDLER.get_transform_matrix(base_frame_id, target_frame_id)
        rate.sleep()


    print('end main')
    #inout.save_scene_gt_list(json_6d_path, scene_gt)

if __name__ == '__main__':
    main()
