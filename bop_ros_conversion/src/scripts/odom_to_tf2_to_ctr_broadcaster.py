#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def handle_pose(msg, agent_name): #x, y, z, r, p, yw):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    #print('x is ',x)
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = '%s_optitrack' %agent_name
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform(t)

    #t2 = geometry_msgs.msg.TransformStamped()
    #t2.header.stamp = rospy.Time.now()
    #t2.header.frame_id= '%s_optitrack' %agent_name
    #t2.child_frame_id = '%s_center' %agent_name
    #TODO:
    #   - make arg based so we can change transforms depending on husky and box
    #q = tf_conversions.transformations.quaternion_from_euler(r, p, yw)
    #t2.transform.translation.x = x
    #t2.transform.translation.y = y
    #t2.transform.translation.z = z
    #t2.transform.rotation.x = q[0]
    #t2.transform.rotation.y = q[1]
    #t2.transform.rotation.z = q[2]
    #t2.transform.rotation.w = q[3]
    #tfm = tf2_msgs.msg.TFMessage([t2])

    #br.sendTransform(t2)

def main():
    rospy.init_node('odom_to_tf2_broadcaster')
    agent_name =     rospy.get_param('~agentname')
    #queue_size =  10 #rospy.get_param('~x')
    #translation_y =  rospy.get_param('~y')
    #translation_z =  rospy.get_param('~z')
    #rotation_roll =  rospy.get_param('~roll')
    #rotation_pitch = rospy.get_param('~pitch')
    #rotation_yaw =   rospy.get_param('~yaw')
    rospy.Subscriber('/sync/%s/pose' %agent_name,
                     Odometry,handle_pose,agent_name) #transx)#,     # 2nd argument supplied to callback
                     #translation_x)  # 3rd argument supplied to callback
                     #translation_y,  # 4rd argument supplied to callback
                     #translation_z,  # 5th argument supplied to callback
                     #rotation_roll,  # 6th argument supplied to callback
                     #rotation_pitch, # 7thrd argument supplied to callback
                     #rotation_yaw,   # 8th argument supplied to callback
                     #)
    rospy.spin()

if __name__ == '__main__':
    main()
