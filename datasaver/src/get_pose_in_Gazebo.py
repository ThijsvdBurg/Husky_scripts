#!/usr/bin/env python
import sys
import rospy

import tf
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import timeit

#rosservice call gazebo/get_model_state '{model_name: husky}'
def retrieve_pose_client():
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        retrieve_relative_pose = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        param_model_name = rospy.get_param("~model_name")
        param_relative_entity_name = rospy.get_param("~relative_entity_name")
        relative_pose = retrieve_relative_pose(model_name=param_model_name, relative_entity_name=param_relative_entity_name)
        return relative_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__=="__main__":
    
    rospy.init_node('box_tf_broadcaster')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('model_relative_poseTwist', Odometry, queue_size=1)

    # pub_twist = rospy.Publisher('model_relative_twist', Twist, queue_size=1)
    # pub_pose = rospy.Publisher('model_relative_pose', Pose, queue_size=1)
    rate = rospy.Rate(100.0)
    child_link = rospy.get_param("~child_link")
    parent_link = rospy.get_param("~parent_link")

    _odom_frame = 'world'
    _footprint_frame = rospy.get_param("~model_name")
    # lastTime =  rospy.Time.now() 

    while not rospy.is_shutdown():
        relative_pose = retrieve_pose_client()

        # timeDiff = rospy.Time.now() - lastTime
        # lastTime = rospy.Time.now() 
        # rospy.loginfo("time: ", rospy.Time.now())

        if relative_pose.success:
            # take the base_link frame the same as the robot_model frame
            br.sendTransform((relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z),
                            (relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w),
                            relative_pose.header.stamp,
                            child_link,#child
                            parent_link)#parent
            
            now = rospy.Time.now()
            odom = Odometry()
            odom.header.frame_id = _odom_frame
            odom.header.stamp = now
            odom.child_frame_id = _footprint_frame
            odom.pose.pose = relative_pose.pose
            odom.twist.twist = relative_pose.twist
            
            pub.publish(odom)
            # pub_twist.publish(relative_pose.twist)
            # pub_pose.publish(relative_pose.pose)

            # rospy.loginfo("The pose of the model W.R.T the reference frame: position: [%f, %f, %f], quaternion: [%f, %f, %f, %f]", 
            #     relative_pose.pose.position.x,
            #     relative_pose.pose.position.y,
            #     relative_pose.pose.position.z,
            #     relative_pose.pose.orientation.x,
            #     relative_pose.pose.orientation.y,
            #     relative_pose.pose.orientation.z,
            #     relative_pose.pose.orientation.w)
        else:
            rospy.loginfo("Failled to get the box pose.")
        rate.sleep()
