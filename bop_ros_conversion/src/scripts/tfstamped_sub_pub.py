#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
#import geometry_msgs.msg
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


def handle_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    #t = msg

    if child_name == 'Husky_base_link':
        t.header.stamp = msg.header.stamp #rospy.Time.now()
        t.header.frame_id = "Husky_opti_link"
        t.child_frame_id = '%s' %child_name
        t.transform.translation.x = -.013881934
        t.transform.translation.y = 0.009032268
        t.transform.translation.z = -0.10728907
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0

    if child_name == 'Husky_zed_left_mk_link':
        t.header.stamp = msg.header.stamp #rospy.Time.now()
        t.header.frame_id = "Husky_base_link"
        t.child_frame_id = '%s' %child_name
        t.transform.translation.x = -0.1553935
        t.transform.translation.y = 0.013355832
        t.transform.translation.z = 0.35680165
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0

    if child_name == 'Husky_zed_right_mk_link':
        t.header.stamp = msg.header.stamp #rospy.Time.now()
        t.header.frame_id = "Husky_base_link"
        t.child_frame_id = '%s' %child_name
        t.transform.translation.x = -0.1593366
        t.transform.translation.y = -0.1050237
        t.transform.translation.z =  0.3558767
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1.0


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
    print('msg: ',msg,'\nt: ',t)
    br.sendTransform(t)

def main():
    child_name =    rospy.get_param('~childname')
    parent_name =   rospy.get_param('~parentname')
    rospy.init_node('tf2_sub_pub_%s'%child_name)
    rospy.Subscriber('/sync/robot/pose'
                     TransformStamped, # msg type which is also parsed to the callback
                     handle_pose,      # Callback
                     child_name        # 2nd argument supplied to callback
                     )
    rospy.spin()

if __name__ == '__main__':
    main()
