import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist, Pose
from gazebo_msgs.msg import ModelState, ContactState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry

from timeit import default_timer as timer

import sys
import os
import datetime

class Saver():

    def __init__(self):
        # get the nr of the experiment and the simulation mode
        self.exp_nr_param = rospy.get_param('exp_nr_param')
        self.sim_mode = rospy.get_param('sim_mode_param')
        
        # ROS subscriber
        if self.sim_mode == "simulation":
            self.sub_robot_state_gazebo = rospy.Subscriber("/poseTwist_of_husky_wrt_world", Odometry,
                                                       self.receive_robot_state)
        if self.sim_mode == "mocap":
            self.sub_robot_state_gazebo = rospy.Subscriber("/Bebop1/position_velocity_orientation_estimation",
                                                       Odometry, self.receive_robot_state)

        if self.sim_mode == "simulation":
            self.sub_object_state = rospy.Subscriber("/pose_of_box_wrt_world", Odometry,
                                                       self.receive_object_state)
        if self.sim_mode == "mocap":
            self.sub_object_state = rospy.Subscriber("/Bebop2/position_velocity_orientation_estimation", Odometry,
                                                       self.receive_object_state)


        # Subscriber for data collection with manual control
        self.sub_husky_control = rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, self.receive_control_command)

        self.sub_husky_control_joy = rospy.Subscriber("/joy_teleop/cmd_vel", Twist, self.receive_control_command_joy)

        self.robot_state = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.object_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.control_command = [0.0, 0.0]
        self.control_command_joy = [0.0, 0.0]

        self.control_command_list = []
        self.control_command_joy_list = []
        self.robot_state_list = []
        self.object_state_list = []
        self.timesteps_list = []

        self.path = None


    def receive_robot_state(self, odom_robot):

        r = R.from_quat([odom_robot.pose.pose.orientation.x, 
                            odom_robot.pose.pose.orientation.y, 
                            odom_robot.pose.pose.orientation.z, 
                            odom_robot.pose.pose.orientation.w])

        ypr = r.as_euler('zyx', degrees=False)[0]

        vel = np.sqrt(odom_robot.twist.twist.linear.x**2 + odom_robot.twist.twist.linear.y**2)
        omega = odom_robot.twist.twist.angular.z
        self.robot_state = [odom_robot.pose.pose.position.x, 
                                   odom_robot.pose.pose.position.y, ypr, vel, omega]

    def receive_object_state(self, obj_pos_msg):

        r = R.from_quat([obj_pos_msg.pose.pose.orientation.x, 
                            obj_pos_msg.pose.pose.orientation.y, 
                            obj_pos_msg.pose.pose.orientation.z, 
                            obj_pos_msg.pose.pose.orientation.w])

        ypr = r.as_euler('zyx', degrees=False)[0]

        vel_x = obj_pos_msg.twist.twist.linear.x
        vel_y = obj_pos_msg.twist.twist.linear.y
        omega = obj_pos_msg.twist.twist.angular.z
        self.object_state = [obj_pos_msg.pose.pose.position.x, obj_pos_msg.pose.pose.position.y, ypr, vel_x, vel_y, omega]

    def receive_control_command(self, twist_msg):
        vel = np.sqrt(twist_msg.linear.x**2 + twist_msg.linear.y**2)
        omega = twist_msg.angular.z 
        self.control_command = [vel, omega]

    def receive_control_command_joy(self, twist_msg):
        vel = np.sqrt(twist_msg.linear.x**2 + twist_msg.linear.y**2)
        omega = twist_msg.angular.z 
        self.control_command_joy = [vel, omega]

    def get_save_path(self):
        # make new directory and save data there
        # self.path = "/home/susan/Documents/mocap_experiment/data/exp_" + str(self.exp_nr_param) #os.path.join("./../data/exp_", str(self.exp_nr_param))
        self.path = "/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/20220819/20220819_exp_" + str(self.exp_nr_param) #os.path.join("./../data/exp_", str(self.exp_nr_param))
        os.mkdir(self.path)

    def save_run_info(self, shape, mass):
        dict_info = {}
        dict_info["box_shape"] = shape
        dict_info["box_mass"] = mass
        dict_info["current_timestamp"] = datetime.datetime.now().strftime("%H:%M:%S")

        print("dict_info", dict_info)

        with open(self.path + "/info_trajectory_" + str(self.exp_nr_param) + ".txt", 'w') as f:
            for key, value in dict_info.items():
                f.write(str(key) + ' >>> '+ str(value) + '\n\n')

    def save_data_sample(self, t):
        self.robot_state_list.append(self.robot_state)
        self.object_state_list.append(self.object_state)
        self.control_command_list.append(self.control_command)
        self.control_command_joy_list.append(self.control_command_joy)
        self.timesteps_list.append([t])

        self.robot_state_array = np.array(self.robot_state_list)
        self.object_state_array = np.array(self.object_state_list)
        self.control_command_array = np.array(self.control_command_list)
        self.control_command_joy_array = np.array(self.control_command_joy_list)
        self.timesteps_array = np.array(self.timesteps_list)

        # saving the data
        save_array = np.concatenate((self.robot_state_array, self.object_state_array, self.control_command_array, self.control_command_joy_array, self.timesteps_array), axis  = 1)

        np.save(self.path + "/trajectory_" + str(self.exp_nr_param) + ".npy", save_array)
        

if __name__ == '__main__':
    pass
