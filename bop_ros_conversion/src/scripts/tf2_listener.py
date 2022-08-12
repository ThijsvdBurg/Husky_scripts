#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import math
import turtlesim.srv

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def main():
    agent_name_1 = 'robot' #    rospy.get_param('~agentname_1')
    agent_name_2 = 'object' #    rospy.get_param('~agentname_2')

    rospy.init_node('tf2_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('', t)
    rate=rospy.Rate(10.0)

    tf = rospy.Publisher('/sync/transform', geometry_msgs.msg.TransformStamped, queue_size=25)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(agent_name_1, agent_name_2, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        tf.publish(trans)
        rate.sleep()

if __name__ == '__main__':
    main()
