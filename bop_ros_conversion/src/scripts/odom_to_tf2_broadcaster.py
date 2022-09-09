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
    t.header.seq = msg.header.seq
    t.header.stamp = msg.header.stamp #rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = '%s_opti_link' %agent_name
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    #print('tf sent with child frame ',t.child_frame_id)
    br.sendTransform(t)
    #msg_no+=1

def main():
    rospy.init_node('odom_to_tf2_broadcaster')
    agent_name1 =     rospy.get_param('~agentname1')
    agent_name2 =     rospy.get_param('~agentname2')
    #msg_no=0
    rospy.Subscriber('/sync/%s/pose' %agent_name2,
                     Odometry,       # msg type which is also parsed to the callback
                     handle_pose,    # Callback
                     agent_name1     # 2nd argument supplied to callback
                     )
    rospy.spin()

if __name__ == '__main__':
    main()
