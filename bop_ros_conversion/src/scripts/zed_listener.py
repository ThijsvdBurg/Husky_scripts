#!/usr/bin/env python
import rospy

import time
import json
#import numpy as np
import os
# Because of transformations
import tf_conversions.posemath as pm
import tf as tfclassic
import math
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

#def _save(dictionary, raw_abs_filepath)
#    return json.dumps(y,indent=4)
def _save(dictionary, raw_abs_filepath):
    with open(raw_abs_filepath, 'w') as file:
        json.dump(dictionary, file, indent=2)

def main():
    #TODO:
    #		- Code cleanup
    #		- fix todos below
    #		- extract row-wise values and check if they are correct
    #		- 
    rospy.init_node('zed_listener')

    base_frame_id = rospy.get_param('~agentname1')
    target_frame_id = rospy.get_param('~agentname2')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tfl = tfclassic.TransformListener()
    #print(dir(tfBuffer))
    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('', t)
    rate=rospy.Rate(5.0)
    rospy.sleep(1.0)
    tfpub = rospy.Publisher('/sync/transform', geometry_msgs.msg.TransformStamped, queue_size=25)


    scene_num = 1
    obj_id = 1
    image_num = 1

    #TODO change this when actually converting
    scenes_path = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/'
    json_6d_path = os.path.join(scenes_path, f"{scene_num:06}", "scene_gt.json")

    if os.path.exists(json_6d_path):
        with open(json_6d_path, "r") as gt_scene:
            gt_6d_pose_data = json.load(gt_scene)
    else:
        gt_6d_pose_data = {}

    with open(json_6d_path, 'w+') as gt_scene:

        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(base_frame_id, target_frame_id, rospy.Time(0), timeout=rospy.Duration(10.0))
                current_time = rospy.Time.from_sec(time.time())
                nanosec = current_time.to_nsec()
                # EXAMPLE:
                # {  "0": [{"cam_R_m2c": [-0.6779589653015137, 0.6399039030075073, 0.36179420351982117, 0.33407020568847656, -0.17020763456821442, 0.9270527362823486, 0.6548046469688416, 0.7493681907653809, -0.09837926179170609], 
                           #"cam_t_m2c": [356.5428161621094, -45.313438415527344, 1177.91015625], 
                           #"obj_id": 3}, 
                          #{"cam_R_m2c": [-0.038782037794589996, -0.8341618776321411, 0.5501546263694763, -0.3616027235984802, 0.5249704718589783, 0.77048659324646, -0.9315254092216492, -0.16905643045902252, -0.3219946324825287], 
                           #"cam_t_m2c": [-112.53829193115234, 246.022705078125, 1009.3762817382812], 
                           #"obj_id": 1}, 
                          #{"cam_R_m2c": [-0.7954239845275879, 0.1994752138853073, -0.5722852349281311, -0.6053470969200134, -0.21592409908771515, 0.7661146521568298, 0.029250729829072952, 0.9558171629905701, 0.29250290989875793], 
                           #"cam_t_m2c": [5.740962296840735e-05, -0.00010618045780574903, 790.0726928710938], 
                           #"obj_id": 2}],
    	            #print(trans)
                np_tf_matrix = tfl.asMatrix('box_ctr',trans.header)
                #print(np_array)
                # npl = np_array.tolist()
                #print('npL type is:', type(npl))
                # print('nplist is:', npl)
                #print('np_matrix: \n', np_matrix)
                #print('rot:', rot)

                # bop toolkit annotation lines

                transform_cam_to_object = np_tf_matrix
                translation = list(transform_cam_to_object[0:3, 3 ]*1000   )  # convert meter to mm
                rotation    =      transform_cam_to_object[0:3,0:3].tolist()  # rotation matrix
                #TODO change name
                view_angle_data = list()
                obj_data = {
                    "cam_R_m2c": rotation,     # rotation
                    "cam_t_m2c": translation,  # translation
                    "obj_id": obj_id
                }
                view_angle_data.append(obj_data)
                #print(view_angle_data)
                gt_6d_pose_data[str(image_num)] = view_angle_data
                json.dump(gt_6d_pose_data, gt_scene)
                #data = {
                #    '_timestamp_ns': nanosec,
                #    'header': {
                #        'seq': 0,
                #        'stamp' : {
                #            'secs': 0,
                #            'nsecs': 0,
                #        },
                #        'frame_id': base_frame_id,
                #    },
                #    'transforms': npl,
                #}
                #print(data)
                #_save(data,'/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/001_synced.json')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rate.sleep()
                #continue
                print(e)
            #print('Success') #    tf.publish(trans)
            #print('dir data is: ', dir(data))
            rate.sleep()
            tfpub.publish(trans)



if __name__ == '__main__':
    main()
