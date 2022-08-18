#!/usr/bin/env python
import rospy

# Because of transformations
import tf

import math
#import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import nav_msgs.msg

class TransformerListener:

    def __init__(self):
        self.tl = tf.TransformListener() 
        #rospy.Subscriber("/sync/robot/pose",nav_msgs.msg.Odometry,self.callback)
        rospy.Subscriber("/tf",tf2_msgs.msg.TFMessage,self.callback)

    def callback(self,msg):
        print('callback function called, here is the message: \n')
        # do json dumping here
        print(msg)
        #matrix = self.tl.asMatrix('robot',msg.header)
        #print(dir(msg))

def main():
    rospy.init_node('tf_listener')

    #tl1 = TransformerListener(True,rospy.Duration(10.0))
    tl1 = TransformerListener()
    #listener = tf2_ros.TransformListener(tfBuffer)
    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('', t)
    rospy.spin()

if __name__ == '__main__':
    main()
