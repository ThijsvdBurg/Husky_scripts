#!/usr/bin/env python
# new husky vis for mppi
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


class Husky_Visualization:
    def __init__(self):
        # Visualization info 
        self.robot_info = None  # robot size, current state, and goal pos
        self.robot_mpc_x_plan = None  # robot mpc_z_plan
        self.obs_info = None  # obs size and current pos
        self.box_info = None  # box size and current pos

        self.nx = 3
        self.N = 30#15
        self.K = 30

        self.d_box = {}
        self.d_husky = {}
        self.d_contact = {}

        # More information
        self.robot_size = [0.5, 0.4]
        self.robot_state = list(np.zeros(self.nx))
        self.robot_pos_goal = [5, 5]
        self.robot_mpc_path = np.tile(np.array([0.0, 0.0]).reshape((-1, 1)), (1, self.N))
        self.robot_mpc_path_box = np.tile(np.array([0.0, 0.0]).reshape((-1, 1)), (1, self.N))
        self.robot_last_state = [0.0, 0.0, 0.0]
        self.box_last_state = [0.0, 0.0, 0.0]

        # ROS subscribers 
        self.sub_robot_mpc_x_plan = rospy.Subscriber('/husky/mpc_x_plan', Float64MultiArray,
                                                     self.receive_robot_mpc_x_plan)
        self.sub_robot_info = rospy.Subscriber('/husky/robot_info', Float64MultiArray, self.receive_robot_info)

    def receive_robot_mpc_x_plan(self, mpc_x_plan_msg):
        # save the msg to robot_mpc_x_plan
        robot_mpc_x_plan = np.array(mpc_x_plan_msg.data)  # column vector
        self.robot_mpc_x_plan = robot_mpc_x_plan.reshape((self.N, self.K, self.nx)) #.reshape((self.nx, self.N), order='F')  # matrix   (self.horizon, self.N, self.cur_state.shape[0])
        
        for x in range(0, self.K):
            self.d_husky["husky_path_{0}".format(x)] = self.robot_mpc_x_plan[:, x, 0:2].T
            #self.d_box["box_path_{0}".format(x)] = self.robot_mpc_x_plan[:, x, 3:5].T
            #self.d_contact["contact_path_{0}".format(x)] = self.robot_mpc_x_plan[:, x, 6:8].T

    def receive_robot_info(self, robot_info_msg):
        # save the msg to robot_info
        robot_info = robot_info_msg.data
        self.robot_state = list(robot_info)

    def receive_obs_info(self, obs_info_msg):
        # save the msg to obs_info
        pass

    def receive_box_info(self, box_info_msg):
        # save the msg to box_info
        pass

def run_visualization_simple():
    print("Starting Visualization...")

    # Initialize a ros node
    rospy.init_node('husky_visualization_node', anonymous=False)

    # Specify the control node frequency
    hz = 20
    rate = rospy.Rate(hz)
    rospy.sleep(1.0)

    # Create a husky visualization object
    husky_vis = Husky_Visualization()

    # Prepare a figure for visualization 
    plt.ion()
    fig_main, ax_main = plt.subplots()
    ax_main.grid(visible=True, ls='-.')
    ax_main.set_aspect('equal')
    ax_main.set_xlim([-9, 11])
    ax_main.set_ylim([-9, 11])
    ax_main.set_xlabel('x [m]')
    ax_main.set_ylabel('y [m]')
    # plot obejects
    # robot current pos
    #robot_pos_ellipse = mpatches.Ellipse([0.0, 0.0], 2.0, 1.6, angle=0.0, fc=(0, 0, 1, 0.8), ec=(0, 0, 1, 0.8))
    #fig_robot_pos = ax_main.add_artist(robot_pos_ellipse)
 
    # robot goal location 
    fig_robot_goal = ax_main.plot(1.0, 1.0, marker='d', mec='r', mfc='r', ms=6)

    plt.draw()

    # Start running
    n_loop = 0
    while not rospy.is_shutdown():
        # clear the plot
        plt.cla()
        ax_main.grid(visible=True, ls='-.')
        ax_main.set_aspect('equal')
        ax_main.set_xlim([-9, 11])
        ax_main.set_ylim([-9, 11])
        ax_main.set_xlabel('x [m]')
        ax_main.set_ylabel('y [m]')

        # robot current pos

        # Update robot current pos 
        #fig_robot_pos.set_center(husky_vis.robot_state[0:2])
        #fig_robot_pos.set_angle(np.rad2deg(husky_vis.robot_state[2]))
        #fig_robot_pos.set_width(2.0 * husky_vis.robot_size[0])
        #fig_robot_pos.set_height(2.0 * husky_vis.robot_size[1])

        robot_pos_ellipse = mpatches.Ellipse([husky_vis.robot_state[0], husky_vis.robot_state[1]], 2.0 * husky_vis.robot_size[0], 2.0 * husky_vis.robot_size[1], angle=np.rad2deg(husky_vis.robot_state[2]), fc=(0, 0, 1, 0.8), ec=(0, 0, 1, 0.8))
        fig_robot_pos = ax_main.add_artist(robot_pos_ellipse)
     
        # Plot path
        robot_pos_path = mpatches.Ellipse(husky_vis.robot_state[0:2], 0.05, 0.05, angle=0.0, fc=(0, 0, 1, 0.2), ec=(0, 0, 1, 0.2))
        fig_robot_pos_path = ax_main.add_artist(robot_pos_path)

        # Update robot goal pos
        fig_robot_goal[0].set_data(husky_vis.robot_pos_goal[0], husky_vis.robot_pos_goal[1])

	# plot mppi path
        for key, value in husky_vis.d_husky.items():
            ax_main.plot(value[0], value[1], c = "c")

        fig_main.canvas.draw()
        fig_main.canvas.flush_events()

        n_loop += 1
        rate.sleep()

    plt.ioff()
    plt.show()
    


if __name__ == "__main__":
    run_visualization_simple()
