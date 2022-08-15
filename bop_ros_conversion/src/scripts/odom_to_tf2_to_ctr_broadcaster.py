#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_pose(msg, agent_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = '%s_optitrack' %agent_name
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform(t)

    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id= '%s_optitrack' %agent_name
    t2.child_frame_id = '%s_center' %agent_name
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = -0.2
    t2.transform.rotation.x = 0.0
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0
    #tfm = tf2_msgs.msg.TFMessage([t2])

    br.sendTransform(t2)

def main():
    rospy.init_node('odom_to_tf2_broadcaster')
    agent_name =     rospy.get_param('~agentname')
    rospy.Subscriber('/sync/%s/pose' %agent_name,
                     Odometry,       # msg type which is also parsed to the callback
                     handle_pose,    # Callback
                     agent_name      # 2nd argument supplied to callback
                     )
    rospy.spin()

if __name__ == '__main__':
    main()
