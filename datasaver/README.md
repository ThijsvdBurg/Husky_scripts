# Datasaver

Put this in your catkin_ws to collect data with rosbag record and ask the datasample every 0.1 sec

```
roslaunch datasaver data_collection.launch exp_nr:=0
```

The script that needs to be put on the husky and run via ssh (for downsampling the camera messages):

```
roslaunch topic_tools_throttle_example.launch
```

Script for launching a rosbag record that also records the camera feed:
```
roslaunch datasaver data_collection_husky.launch
```
