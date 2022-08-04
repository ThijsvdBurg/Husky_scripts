#!/usr/bin/env python

from rospy_message_converter import json_message_converter
from std_msgs.msg import String
message = String(data = 'Hello')
json_str = json_message_converter.convert_ros_message_to_json(message)

print(json_str)
