import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist, Pose
from gazebo_msgs.msg import ModelState, ContactState
from gazebo_msgs.srv import SetModelState
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry

from timeit import default_timer as timer

import sys
import os
import datetime

class Reader():

    def __init__(self, trajectory_nr):
        self.trajectory_nr = trajectory_nr

        self.path = "/home/susan/Documents/mocap_experiment/data/exp_" + str(self.trajectory_nr)

        self.robot_state = None
        self.control_command = None
        self.object_state = None


    def load_data_sample(self):

        arr = np.load(self.path + "/trajectory_" + str(self.trajectory_nr) + ".npy")

        print(arr.shape)

        self.robot_state = arr[:, 0:5]
        self.object_state = arr[:, 5:11]
        self.control_command = arr[:, 11:13]
        self.control_command_joy = arr[:, 13:15]
        self.time = arr[:, 15]
        print(self.robot_state.shape)
        print(self.control_command.shape)
        print(self.object_state.shape)

        self.plot_data()

    def plot_data(self):
        plt.figure(0)
        plt.plot(self.robot_state[:, 0], self.robot_state[:, 1], "bo", label="Robot path")
        plt.plot(self.object_state[:, 0], self.object_state[:, 1], "ro", label="Object path")
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        plt.legend()
        plt.savefig(self.path + "/xy_" + str(self.trajectory_nr) + ".png")

        timesteps = np.arange(0, self.control_command.shape[0])
        plt.figure(1)
        plt.plot(timesteps, self.control_command[:, 0], "b", label="Robot forward velocity action")
        plt.plot(timesteps, self.control_command[:, 1], "r", label="Robot angular velocity action")
        plt.legend()
        plt.savefig(self.path + "/action_" + str(self.trajectory_nr) + ".png")


        timesteps = np.arange(0, self.object_state.shape[0])
        plt.figure(2)
        plt.plot(self.time, self.object_state[:, 3], "b", label="Object x velocity")
        plt.plot(self.time, self.object_state[:, 4], "r", label="Object y velocity")
        plt.plot(self.time, self.object_state[:, 5], "m", label="Object angular velocity")
        plt.legend()
        plt.savefig(self.path + "/vel_object_" + str(self.trajectory_nr) + ".png")

        timesteps = np.arange(0, self.robot_state.shape[0])
        plt.figure(3)
        plt.plot(self.time, self.robot_state[:, 3], "b", label="Robot forward velocity")
        plt.plot(self.time, self.robot_state[:, 4], "m", label="Robot angular velocity")
        plt.legend()
        plt.savefig(self.path + "/vel_robot_" + str(self.trajectory_nr) + ".png")
if __name__ == '__main__':
    nr = 2
    reader = Reader(nr)
    reader.load_data_sample()
