#!/usr/bin/env python
import rospy
#import tf2
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
#import tf2_geometry_msgs

buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))

#print(dir(buffer_core))
#ts1 = Odometry()
ts1 = TransformStamped()
ts1.header.stamp = rospy.Time(0)
ts1.header.frame_id = 'map'
ts1.child_frame_id = 'frame1'

ts1.transform.translation.x = 2.71828183
ts1.transform.rotation.w = 1.0

ts2=Odometry()
ts2.header.stamp = rospy.Time(0)
ts2.header.frame_id = 'map'
ts2.child_frame_id = 'frame2'
ts2.pose.pose.position.x = 10.71828183
ts2.pose.pose.orientation.w = -1.0

print(type(ts2))

ts3=Odometry()
print("ts3 odometry type is: \n",type(ts3))
ts3_geom=ts3.pose.pose
print("ts3.pose.pose type is: \n",type(ts3_geom))
#print(type(ts3_geom))
#print(dir(ts3_geom))
# transform.rotation.w = 1.0
# TODO (thijsvdburg):
#	- try to convert geometry_msgs Pose message to tf2
#	using python wrapper for the c++ method tf2::fromMsg(const geometry_msgs::msg::Pose & in, tf2::Transform & out)

# ts3_tf=tf2_ros.to_msg(ts3_geom)

# create buffer core class from ts1 with the buffercore settransform
buffer_core.set_transform(ts1, "default_authority") # bool tf2::BufferCore::setTransform(const geometry_msgs::TransformStamped & transform, const std::string & authority,bool 	is_static = false)


# why no lookup_transform(
# a = buffer_core.lookup_transform_core('map', 'frame1', rospy.Time(0))
#print(a)
# ((2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
b = buffer_core.lookup_transform_core('frame1', 'map', rospy.Time(0))
#print("printing b: \n",b)
# ((-2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

ts2 = TransformStamped()
ts2.header.stamp = rospy.Time(0)
ts2.header.frame_id = 'frame1'
ts2.child_frame_id = 'frame2'
ts2.transform.translation.x = 0
ts2.transform.translation.y = 0.5
# TODO(lucasw) example rotation using transform3d/transformations.py
ts2.transform.rotation.w = 1.0
buffer_core.set_transform(ts2, "default_authority")

#print(buffer_core.lookup_transform_core('map', 'frame2', rospy.Time(0)))
