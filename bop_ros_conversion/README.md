# Husky_scripts
Scripts for converting rosbags to the BOP dataset format, as desribed in:
https://github.com/thodan/bop_toolkit/blob/master/docs/bop_datasets_format.md


# Sync timestamps in bags if misaligned
Our data was misaligned because of the clock of the Husky, which was not up to date with the world clock. Therefore, no match could be found between the optitrack datapoints (which had the 'correct' timestamps).

We therefore substituted the correct optitrack datapoints with the faulty Husky datapoints. For our case, this made a little more sense since the Husky was the 'main' computer on which everything was recorded, so the timestamps in all other topics were the 'wrong' Husky timestamps.

Step-by-step:
1. Record a new bagfile with the ```./src/scripts/rewrite_rosmsg_header_timestamps.py``` script <br />
example usage: <br />
```rosrun bop_ros_conversion rewrite_rosmsg_header_timestamps.py --source_dir '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220705' --target_dir '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220705' --start 100 --end 102 --date 20220705```
2. Record a synced bagfile with the ```./src/scripts/sync_husky_and_optitrack_topics.py``` script. 
Be sure to substitute any desired topic names in the scripts, also the script assumes that the format of the bagfile name is as following:
```<date in yyyymmdd>_exp_<exp number with 6 figures like 000124 for experiment 124>``` 
or
```<date in yyyymmdd>_sequence_<exp number with 6 figures like 000124 for experiment 124>```


# publish Odometry messages as transform messages

1. Use ```roslaunch bop_ros_conversion odom_to_tf2.launch```
2. Substitute values in the launchfile for your specific needs
3. 
