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
    t = msg

    #t.header.stamp = rospy.Time.now()
    #t.header.frame_id = "world"
    #t.child_frame_id = '%s_optitrack' %agent_name
    #t.transform.translation.x = msg.pose.pose.position.x
    #t.transform.translation.y = msg.pose.pose.position.y
    #t.transform.translation.z = msg.pose.pose.position.z

    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    #t.transform.rotation.x = msg.pose.pose.orientation.x
    #t.transform.rotation.y = msg.pose.pose.orientation.y
    #t.transform.rotation.z = msg.pose.pose.orientation.z
    #t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform(t)

def main():
    rospy.init_node('tf2_sub_pub')
    rospy.init_node('tfstamped_broadcaster')
    #agent_name =     rospy.get_param('~agentname')
    rospy.Subscriber('/tfstamped',
                     TransformStamped,  # msg type which is also parsed to the callback
                     handle_pose #,     # Callback
                     #agent_name        # 2nd argument supplied to callback
                     )
    rospy.spin()

if __name__ == '__main__':
    main()
