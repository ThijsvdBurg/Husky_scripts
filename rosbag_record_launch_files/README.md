# Rosbag record scripts, launchfles and information sources

Useful links:
* [Recording and playing rosbags tutorial](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data "Recording and playing bags")
* [Reading from bagfile tutorial](http://wiki.ros.org/rosbag/Tutorials/reading%20msgs%20from%20a%20bag%20file "Reading bagfile")
* [Filtered bagfiles tutorial](http://wiki.ros.org/rosbag/Tutorials/Producing%20filtered%20bag%20files "Filtered bagfiles")
* [Topic_tools throttle](http://wiki.ros.org/topic_tools/throttle "Throttle bagfiles")
# * [Filtered bagfiles tutorial](http://wiki.ros.org/rosbag/Tutorials/Producing%20filtered%20bag%20files "Filtered bagfiles")


If you wish to throttle certain nodes to limit their publish rates (for example, a camera which publishes at 30Hz, which can be too frequent in some use cases), use the topic_tools_throttle launchfile.
Run the launchfile locally on the robot. Otherwise, the raw unthrottled topics would need to travel from the robot to the computer, and only then get stored somewhere. This defeats the purpose of throttling the publishing rate.

Copy the file to the robot and edit it to throttle the desired topics. For us, using a rate of 7.0 Hz effected in a bagfile with the topics at roughly 5.0 Hz

Use the data_collection launchfile to record data. For any heavy data stream, we advice to save the bagfiles locally on the Husky, and frequently move them to another storage device, since the available space on the Husky is extremely limited.
Use

rosbag info <instert path to bagile>

To see what effect the throttling had
