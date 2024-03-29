#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# robot to zed transform broadcaster
# FIXME: 
# 	- add true translation from robot optitrack to child
#	- see if it's possible to perform this not 10Hz but callback style
class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage,queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "robot_optitrack"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "zed"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 2.0
            t.transform.translation.z = 0.5
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)


def main():
    rospy.init_node('fixed_tf2_broadcaster')
    # agent_name =     rospy.get_param('~agentname')
    tfb = FixedTFBroadcaster()

    rospy.spin()


if __name__ == '__main__':
    main()
