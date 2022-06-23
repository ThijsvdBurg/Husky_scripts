# Husky_scripts
Scripts for using the Husky robot for data collection, data converting etc.


# Experiment manual

## Mocap:
1. Login and start motive
2. Load the project and make sure all the rigid bodies are defined (robot is rigid body 1, object is rigid body 2)

## Robot connection
1. Connect the computer to the wifi base station (and turn wifi base on)
2. Connect an ethernet cable between the laptop and the robot itself + between the mocap wifi and laptop
3. Make sure this is in each command window, alternatively, add these to your ~/.bashrc file:
```
source /opt/ros/melodic/setup.bash
source <path_to_workspace>/devel/setup.bash
export ROS_MASTER_URI=http://192.168.131.1:11311
```
4. Check if everything works (the robot topics should appear)
```
rostopic list
```
5. Then launch files
```
roslaunch mocap_optitrack mocap_multidrone.launch  
roslaunch bebop2_state_estimator odometry_estimator_multiple.launch
```
6. Check if the messages of the mocap are ok by doing:
```
rostopic echo /Bebop1/pose
rostopic echo /Bebop1/position_velocity_orientation_estimation
```
7. For collecting rosbag and numpy array of mocap and cmd_vel command:
```
roslaunch datasaver data_collection.launch -> for launching the data collection node in datasaver folder
```
8. For subsampling the image data before recording it, move the launch file in datasaver/topic_tools_throttle_example.launch to the husky (see README in rosbag folder) and run it from the robot:
```
roslaunch topic_tools_throttle_example.launch
```
9. For recording the subsampled data, move the launch file in datasaver/data_collection_husky.launch to the husky (see README in rosbag folder) and run it from the robot:
```
roslaunch data_collection_husky.launch
```

