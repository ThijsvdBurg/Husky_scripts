#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import math
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('zed_listener')

    #agentname1 = robot_optitrack  #
    agentname1 = rospy.get_param('~agentname1')
    agentname2 = rospy.get_param('~agentname2')
    # object_optitrack

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('', t)
    rate=rospy.Rate(10.0)

    tf = rospy.Publisher('/sync/transform', geometry_msgs.msg.TransformStamped, queue_size=25)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(agentname1, agentname2, rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        tf.publish(trans)
        rate.sleep()

if __name__ == '__main__':
    main()
