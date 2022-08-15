#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# box optitrack centre to desired geometrical centre broadcaster

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage,queue_size=100)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "object_optitrack"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "box_ctr"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = -0.2
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)


def main():
    rospy.init_node('fixed_tf2_box_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()


if __name__ == '__main__':
    main()
