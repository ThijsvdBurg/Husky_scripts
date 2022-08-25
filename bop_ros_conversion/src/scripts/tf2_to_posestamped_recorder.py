#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
author: Michael Grupp

This file is part of evo (github.com/MichaelGrupp/evo).

evo is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

evo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with evo.  If not, see <http://www.gnu.org/licenses/>.
"""

import datetime
import os

import rosbag
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import copy

DESC = """Record a tf frame's trajectory to a geometry_msgs/PoseStamped bag."""


class Recorder(object):
    def __init__(self, parent_frame, child_frame, child_frame2, lookup_frequency, bagfile,
                 output_topic, append):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.child_frame2 = child_frame2
        self.bagfile = bagfile
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lookup_frequency = lookup_frequency
        self.output_topic = output_topic
        self.append = append

    def run(self):
        msg_count = 0
        time = rospy.get_rostime()
        time.secs = 1661344252
        time.nsecs= 765565395
        new_child_frame = 'Husky/opti_link'
        new_child_frame2= 'MMbox/opti_link'
        try:
            print('1')
            bag = rosbag.Bag(self.bagfile, mode='a' if self.append else 'w')
            # plaats deze erbuiten
            rate = rospy.Rate(self.lookup_frequency)
            last_stamp = rospy.Time()
            while not rospy.is_shutdown():
                #print('2')
                try:
                    #print('3')
                    transform = self.tf_buffer.lookup_transform(
                        self.parent_frame, self.child_frame, rospy.Time())
                    transform2 = self.tf_buffer.lookup_transform(
                        self.parent_frame, self.child_frame2, rospy.Time())
                    #print('directly after loopup transform, the transform header stamp is:\n',transform.header.stamp)
                    rate.sleep()
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    #print('4')
                    rate.sleep()
                    continue
                if last_stamp == transform.header.stamp:
                    #print('5')
                    #print('continued...')
                    continue
                pose = transformstamped_to_posestamped(transform,time,new_child_frame)
                pose2 = transformstamped_to_posestamped(transform2,time,new_child_frame2)

                #manually define child id
                #write to bag
                #bag.write(self.output_topic, pose, t=pose.header.stamp)
                bag.write(self.output_topic, pose, t=transform.header.stamp)
                bag.write(self.output_topic, pose2, t=transform2.header.stamp)
                #bag.write(self.output_topic, pose, t=time) # time as manually created time
                msg_count += 1
                #if msg_count>200:
                #    break
                time.secs+=1
                time.nsecs+= 102080
                #print(msg_count)
                last_stamp = transform.header.stamp
                rospy.loginfo_throttle(
                    2, "Recorded {} PoseStamped messages.".format(msg_count))

        except rospy.ROSInterruptException:
            print('7')
            pass
        finally:
            rospy.loginfo_throttle(
                2, "Recorded {} PoseStamped messages.".format(msg_count))
            bag.close()
            rospy.loginfo("Finished recording.")


def transformstamped_to_posestamped(transform_stamped,time,new_child_frame):
    #pose_stamped = PoseStamped()
    pose_stamped = TransformStamped()
    #print('1 transform_stamped.header.stamp in method is:', transform_stamped.header.stamp)
    #pose_stamped = transform_stamped
    pose_stamped = copy.deepcopy(transform_stamped)
    #print('2 transform_stamped.header.stamp in method is:', transform_stamped.header.stamp)
    pose_stamped.header.stamp = time
    pose_stamped.child_frame_id = new_child_frame
    print('3 transform_stamped.header.stamp in method is:', transform_stamped.header.stamp, pose_stamped.header.stamp)
    return pose_stamped

def timestamp_str():
    return str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

def main(parent_frame, child_frame, child_frame2, lookup_frequency, bagfile, output_topic,
         append):
    rospy.init_node("record_tf_as_posestamped_bag")
    recorder = Recorder(parent_frame, child_frame, child_frame2, lookup_frequency, bagfile,
                        output_topic, append)
    recorder.run()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument("parent_frame")
    parser.add_argument("child_frame")
    parser.add_argument("child_frame2")
    parser.add_argument(
        "--lookup_frequency", help="maximum frequency at which transforms "
        "are looked up", default=200.0, type=float)
    parser.add_argument("--output_topic", help="name of the output topic",
                        default=None)
    parser.add_argument(
        "--bagfile", help="output bagfile path", default=os.path.join(
            os.getcwd(),
            timestamp_str() + ".bag"))
    parser.add_argument("--append", action="store_true",
                        help="whether to append to an existing bagfile")

    args = parser.parse_args()

    if args.output_topic is None:
        output_topic = args.child_frame
    else:
        output_topic = args.output_topic

    main(args.parent_frame, args.child_frame, args.child_frame2, args.lookup_frequency,
         args.bagfile, output_topic, args.append)

