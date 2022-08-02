# Convert rosbags to BOP format

https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html

Information about CameraInfo topic

Information needs to be converted from ROS format to BOP format

## CameraInfo_to_BOP.py

https://github.com/thodan/bop_toolkit/blob/master/docs/bop_datasets_format.md

Steps:

1. 
''' rostopic echo -b 20220622_sequence_6.bag /zed_node/left/camera_info_throttle > camera_scene_00006.txt
'''
