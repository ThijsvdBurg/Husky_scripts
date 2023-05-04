# Husky_scripts
Scripts for using the Husky robot for data collection, data converting etc.


# Experiment manual

## Mocap:
1. Login and start motive
2. Load the project and make sure all the rigid bodies are defined (robot is rigid body 1, object is rigid body 2)

## Robot connection
1. Connect the computer to the wifi base station (and turn wifi base on)
2. Connect an ethernet cable between the laptop and the robot itself + between the mocap wifi and laptop
3. Make sure this is in each command window:
```
source /opt/ros/melodic/setup.bash
source devel/setup.bash
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
8. For subsampling the image data before recording it, move the launch file in datasaver/topic_tools_throttle_example.launch to the husky and run it from the robot:
```
roslaunch topic_tools_throttle_example.launch
```
9. For recording the subsampled data, move the launch file in datasaver/data_collection_husky.launch to the husky and run it from the robot:
```
roslaunch data_collection_husky.launch
```



# Change clock on Husky to synchronize the OptiTrack clock with the Husky clock

## Connect Husky to different network to automatically retrieve the internet clock
1. Startup Husky
2. Connect Husky to base station (this will happen automatically if the base station is switched on)
3. Either connect to husky via ssh or connect husky to monitor using HDMI
4. Start terminal, type ```wicd-curses```
5. Select objective network using the up- and down-arrows (different than DUOT06_Base_station, we use an hp laptop with WiFi hotspot)
6. Right arrow to go to network settings, fill in password at the bottom of the screen
7. Set cursor to a different settings row, then save settings by typing shift+S (also adjust the WPA settings as needed, we did not need to change this)
8. You now end up in main screen, connect to new network by selecting is and then pressing the Enter key
9. If everything is set correctly, the Husky will connect and show the IP address at the bottom in the black status bar with red characters
10. Use this new IP address to ssh into husky and check if the clock is set correctly.

## Changing the Husky timezone

In our case, the clock was updated but a different timezone was selected. Follow the steps below to change this setting. We will change it to Amsterdam
1. ```sudo dpkg-reconfigure tzdata```
2. type the admin password
3. Select Europe and press Enter
4. Select Amsterdam and press Enter


