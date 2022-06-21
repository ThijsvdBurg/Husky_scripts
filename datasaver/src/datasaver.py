#!/usr/bin/env python
import numpy as np
import rospy
from timeit import default_timer as timer
from saver import Saver

import sys


def run_saver():
    print("Starting Data Saver Node...")

    # Initialize a ros node
    rospy.init_node('data_saver', anonymous=False)

    # Specify some things about the data trajectory
    # mass_box = 
    # ...
    shape_box = (48, 32, 33) 
    mass_box = 4.0

    # Create controller object
    saver = Saver()
    saver.get_save_path()
    saver.save_run_info(shape_box, mass_box)

    # Specify the control node frequency
    hz = 10
    rate = rospy.Rate(hz)
    rospy.sleep(1.0)

    t = 0

    while not rospy.is_shutdown():
        start = timer()

        # Compute control action 
        saver.save_data_sample(t)
        
        end = timer()
        t += 0.1
        rate.sleep()


if __name__ == '__main__':
    rospy.sleep(1.0)
    #sim_mode = "simulation" # mocap

    run_saver()


