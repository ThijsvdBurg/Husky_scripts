#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import math
import turtlesim.srv
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_pose():
    

def main():
    rospy.init_node('tf2_turtle_broadcaster')
    agent_name =     rospy.get_param('~agentname')
    rospy.Subscriber('/sync/%s/pose' %agent_name,
                     Odometry,              # msg type which is also parsed to the callback
                     handle_pose,    # Callback
                     agent_name             # 2nd argument supplied to callback
                     )
    rospy.spin()


if __name__ == '__main__':
    main()
