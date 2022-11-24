import roslaunch
import rospy

process_generate_running = True

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #cli_args = ['bop_ros_conversion',launchfile,'sequence_number:=000055']
    cli_args = [launchfile,'sequence_number:=000055']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    print('uuid',uuid)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        roslaunch_file,
        process_listeners=[process_listener],
    )
    return launch


rospy.init_node("rospylauncher")
launch_file = "/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bop_ros_conversion/launch/odom_to_tf2.launch"
launch = init_launch(launch_file, ProcessListener())
launch.start()

while process_generate_running:
        rospy.sleep(10)

launch.shutdown()
