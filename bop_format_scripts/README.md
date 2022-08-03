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

Example to rewrite the optitrack timestamps to match to the husky timestamps. This is necessary to match the location of the robot and object with the camera.
--suffix is used to differentiate between bagfiles which are left unedited or cropped beforehand. Use -h flag to see the options.
''' python ~/noetic-husky/bop_ros_ws/src/Husky_scripts/trial_package/src/scripts/ros_rewrite_header_timestamps.py --source_dir '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/' --date 20220705 --target_dir '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/' --start 108 --end 109 --suffix crop
'''