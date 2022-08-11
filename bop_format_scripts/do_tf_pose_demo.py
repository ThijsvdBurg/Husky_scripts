#!/usr/bin/python
# -*- coding: utf-8 -*-


# import modules
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

rospy.init_node('tf2_turtle_listener')
tf_buffer = tf2_ros.Buffer()  # tf buffer length
#Interface




def transform_pose(transformation, point_wrt_source):
    #PointStamped(point=point_wrt_source),transformation).point
    pose_stamped=transformation
    tf = point_wrt_source
    pose_wrt_target = tf2_geometry_msgs.do_transform_pose(pose_stamped, tf)
    print(pose_wrt_target)
    return [pose_wrt_target.x, pose_wrt_target.y, pose_wrt_target.z]

# def do_transform_pose(pose, transform):

world_coords = TransformStamped()
print(world_coords)
world_coords.header.stamp = rospy.Time(0)
world_coords.header.frame_id = 'world'
world_coords.child_frame_id = 'robot'


p1 = PoseStamped()
p1.header.stamp = rospy.Time(0)
p1.header.frame_id = 'world2'
#p1.child_frame_id = 'frame1'
p1.pose.position.y = 10.71828183
p1.pose.orientation.w = 0.5

p2 = PoseStamped()
p2.header.stamp = rospy.Time(0)
p2.header.frame_id = 'robot'
#p1.child_frame_id = 'frame1'
p2.pose.position.x = 10.71828183
p2.pose.orientation.w = -1.0

pose_stamped=p1
print(p2)

#point_wrt_target = transform_point(transformation, point_wrt_source)

transform = tf_buffer.lookup_transform(world_coords.header.frame_id,
                                       # source frame:
                                       p2.header.frame_id,
                                       # get the tf at the time the pose was valid
                                       # pose_stamped.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Time(0),rospy.Duration(0.8))

#print(dir(transform))
print(transform)
