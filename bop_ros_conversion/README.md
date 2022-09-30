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

2. Record a synced bagfile with the ```./launch/record_synced_bags.launch``` script. 
Be sure to substitute the topic names in the scripts and the script assumes that the format of the bagfile name is as follows:
```<date in yyyymmdd>_exp_<exp number with 6 figures like 000124 for experiment 124>``` 
for example:
```20220705_exp_000098_edit.bag``` for a the 98th rosbag recorded on 05 07 2022 which was edited to have matching header stamps
or
```<date in yyyymmdd>_sequence_<exp number with 6 figures like 000124 for experiment 124>``` <br />

Example usage:
```
roslaunch bop_ros_conversion record_synced_bags.launch output_dir:='/media/pmvanderburg/T7/bagfiles/20220705' date:=20220705 bagrate:='1' sync_slop:='0.02' exp_nr:='000109'
```

# publish Odometry messages as transform messages and save the GT in a json BOP format

1. Use ```roslaunch bop_ros_conversion odom_to_tf2.launch``` 
or 
```export exp_nr=30 <br /> roslaunch bop_ros_conversion odom_to_tf2.launch json_frequency:=10 sequence_number:=$exp_nr``` 
2. Substitute values in the launchfile for your specific needs
3. 
