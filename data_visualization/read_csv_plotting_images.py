import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from datetime import datetime
from matplotlib.patches import Rectangle
import time

# todo:
#	- visualise 3D bounding box and/or
#					3D object center with axis system
#	- 

def plotting_trajectories(states_array_robot, states_array_object, n):
	# plotting
	plt.figure(n*1)
	axis = plt.gca()
	plt.plot(states_array_robot[:, 0], states_array_robot[:, 1], "ro", markersize =1,  label="Robot path")
	plt.plot(states_array_object[:, 0], states_array_object[:, 1], "bo", markersize =1, label="Object path")
	w_box = 0.48
	h_box = 0.60
	r_husky = 0.99/2
	for i in range(0, states_array_object.shape[0]):
		# convert to object rectangle
		objf_x = states_array_object[i, 0] * np.cos(states_array_object[i, 2]) + states_array_object[i, 1] * np.sin(states_array_object[i, 2])
		objf_y = -states_array_object[i, 0] * np.sin(states_array_object[i, 2]) + states_array_object[i, 1] * np.cos(states_array_object[i, 2])
		angle_rot = np.rad2deg(np.arctan((w_box/2) / (h_box/2)))
		objf_x  = objf_x - np.sin(np.deg2rad(angle_rot)) * np.sqrt((w_box/2)**2 + (h_box/2)**2)
		objf_y  = objf_y - np.cos(np.deg2rad(angle_rot)) * np.sqrt((w_box/2)**2 + (h_box/2)**2)
		wf_x = objf_x * np.cos(states_array_object[i, 2]) - objf_y * np.sin(states_array_object[i, 2])
		wf_y = objf_x * np.sin(states_array_object[i, 2]) + objf_y * np.cos(states_array_object[i, 2])
		axis.add_patch(Rectangle((wf_x,  wf_y), w_box, h_box, angle = np.rad2deg(states_array_object[i, 2]), color="blue", fill=None, alpha=0.2))

	for i in range(0, states_array_robot.shape[0]):
		# convert to husky circle
		draw_circle = plt.Circle((states_array_robot[i, 0], states_array_robot[i, 1]), r_husky, color = "red", alpha=0.2, fill = False )
		axis.add_artist(draw_circle)
	

	plt.xlabel("$x$")
	plt.ylabel("$y$")
	plt.grid()
	# plt.xlim([-10, 12])
	# plt.ylim([-10, 12])
	axis.axis('equal')
	plt.legend(loc="upper left")
	plt.savefig("plots/trajectories"+str(n)+".png")


def subsample(timestamp, data_array):
	prev_millisecond = np.inf
	millisecond = np.inf

	lst_sampled = []
	for k in range(0, timestamp.shape[0]):
		time_ = timestamp[k][0]
		prev_millisecond = millisecond
		millisecond = str(time_).split(".")[1][0]
		datetime_ = datetime.utcfromtimestamp(time_)
		second = str(datetime_).split(".")[0][-2:]

		if prev_millisecond != millisecond:
			data_sample_lst = []
			
			for n in range(data_array.shape[1]):
				data_sample_lst.append(float(data_array[k, n]))

			data_sample_lst.append(float(second))
			data_sample_lst.append(float(millisecond))

			data_sample_arr = np.asarray(data_sample_lst)
			lst_sampled.append(data_sample_arr)

	arr_sampled = np.asarray(lst_sampled)

	return arr_sampled

def find_index(start_sec, end_sec, arr):
	same_sec = int(str(start_sec)[:-1])
	same_msec = int(str(start_sec)[-1:])
	# find index where it is
	ind_start = np.where(arr[:, -2] == same_sec )
	ind_m_start = np.where(arr[:, -1] == same_msec)
	final_start_index = np.intersect1d(ind_start, ind_m_start)[0]


	same_sec = int(str(end_sec)[:-1])
	same_msec = int(str(end_sec)[-1:])
	# find index where it is
	ind_end = np.where(arr[:, -2] == same_sec )
	ind_m_end = np.where(arr[:, -1] == same_msec)
	final_end_index = np.intersect1d(ind_end, ind_m_end)[0]

	return final_start_index, final_end_index


def plotting_vels(cmd_array_subs, n):
	t = np.arange(0, cmd_array_subs.shape[0])
	plt.figure(100*n)
	plt.plot(t, cmd_array_subs[:, 0], label="Linear velocity x")
	plt.plot(t, cmd_array_subs[:, 1], label="Angular velocity z")
	plt.legend(loc="upper left")
	plt.savefig("plots/vel"+str(n)+".png")

def plotting_twists_robot(twists_array_robot, n):
	plt.figure(1289*n)
	t = np.arange(0, twists_array_robot.shape[0])
	plt.plot(t, twists_array_robot[:, 0], label="Linear velocity x")
	plt.plot(t, twists_array_robot[:, 1], label="Linear velocity y")
	plt.plot(t, twists_array_robot[:, 2], label="Angular velocity z")
	plt.legend(loc="upper left")
	plt.savefig("plots/twists_robot"+str(n)+".png")

def plotting_twists_object(twists_array_object, n):
	plt.figure(99999*n)
	t = np.arange(0, twists_array_object.shape[0])
	plt.plot(t, twists_array_object[:, 0], label="Linear velocity x")
	plt.plot(t, twists_array_object[:, 1], label="Linear velocity y")
	plt.plot(t, twists_array_object[:, 2], label="Angular velocity z")
	plt.legend(loc="upper left")
	plt.savefig("plots/twists_object"+str(n)+".png")

def get_arrs(i, topics):
	# extract cmd vel and time
	path = "exp_data/exp" + str(i) +"/" + topics["cmd"] + ".csv"
	data_cmd = pd.read_csv(path)
	timestamp_cmd = data_cmd["Time"].to_numpy().reshape(-1,1)
	lin_x = data_cmd["linear.x"].to_numpy().reshape(-1,1)
	ang_z = data_cmd["angular.z"].to_numpy().reshape(-1,1)

		
	# Getting positions and velocities of robot
	path = "exp_data/exp" + str(i) +"/" + topics["robot_vel"] + ".csv"
	data_robot_vel = pd.read_csv(path)
	timestamp_robot_vel = data_robot_vel["Time"].to_numpy().reshape(-1,1)
	pose_x_robot = data_robot_vel["pose.pose.position.x"].to_numpy().reshape(-1,1)
	pose_y_robot = data_robot_vel["pose.pose.position.y"].to_numpy().reshape(-1,1)
	quat_robot = np.concatenate((data_robot_vel["pose.pose.orientation.x"].to_numpy().reshape(-1,1), data_robot_vel["pose.pose.orientation.y"].to_numpy().reshape(-1,1), 
			data_robot_vel["pose.pose.orientation.z"].to_numpy().reshape(-1,1), data_robot_vel["pose.pose.orientation.w"].to_numpy().reshape(-1,1)), axis=1)
	r_robot = R.from_quat(quat_robot)
	theta_robot = r_robot.as_euler('zyx', degrees=False)[:,0].reshape(-1,1)

	twist_robot_x = data_robot_vel["twist.twist.linear.x"].to_numpy().reshape(-1,1)
	twist_robot_y = data_robot_vel["twist.twist.linear.y"].to_numpy().reshape(-1,1)
	twist_robot_theta = data_robot_vel["twist.twist.angular.z"].to_numpy().reshape(-1,1)

	# Getting positions and velocities of object
	path = "exp_data/exp" + str(i) +"/" + topics["object_vel"] + ".csv"
	data_object_vel = pd.read_csv(path)
	timestamp_object_vel = data_object_vel["Time"].to_numpy().reshape(-1,1)
	twist_object_x = data_object_vel["twist.twist.linear.x"].to_numpy().reshape(-1,1)
	twist_object_y = data_object_vel["twist.twist.linear.y"].to_numpy().reshape(-1,1)
	twist_object_theta = data_object_vel["twist.twist.angular.z"].to_numpy().reshape(-1,1)

	pose_x_object = data_object_vel["pose.pose.position.x"].to_numpy().reshape(-1,1)
	pose_y_object = data_object_vel["pose.pose.position.y"].to_numpy().reshape(-1,1)
	quat_object = np.concatenate((data_object_vel["pose.pose.orientation.x"].to_numpy().reshape(-1,1), data_object_vel["pose.pose.orientation.y"].to_numpy().reshape(-1,1), 
			data_object_vel["pose.pose.orientation.z"].to_numpy().reshape(-1,1), data_object_vel["pose.pose.orientation.w"].to_numpy().reshape(-1,1)), axis=1)
	r_object = R.from_quat(quat_object)
	theta_object = r_object.as_euler('zyx', degrees=False)[:,0].reshape(-1,1)

	# concat into arrays
	states_array_robot = np.concatenate((pose_x_robot, pose_y_robot, theta_robot), axis = 1) #[::10]
	states_array_object = np.concatenate((pose_x_object, pose_y_object, theta_object), axis = 1) #[::10]

	twists_array_robot = np.concatenate((twist_robot_x, twist_robot_y, twist_robot_theta), axis = 1) #[::10]
	twists_array_object = np.concatenate((twist_object_x, twist_object_y, twist_object_theta), axis = 1) #[::10]

	cmd_array = np.concatenate((lin_x, ang_z), axis = 1)

	return states_array_robot, states_array_object, twists_array_robot, twists_array_object, cmd_array, timestamp_cmd, timestamp_robot_vel, timestamp_object_vel

def get_overlapping_indexes(states_array_robot_subs, cmd_array_subs):
	# get all indexes where the timestamps overlap
	lst = []
	for i in range(0, states_array_robot_subs.shape[0]):  
		for j in range(0, cmd_array_subs.shape[0]):
			sec_robot = states_array_robot_subs[i, -2]
			msec_robot = states_array_robot_subs[i, -1]
			sec_cmd = cmd_array_subs[j, -2]
			msec_cmd = cmd_array_subs[j, -1]

			if sec_robot == sec_cmd and msec_robot == msec_cmd:
				lst.append((i,j))

	end_ind = -1			
	# iterate through the list and see where there is a part with no data
	for i in range(0, len(lst)-1):
		if lst[i][0] != (lst[i+1][0] - 1) or lst[i][1] != (lst[i+1][1] - 1):
			print("End of rollout at :", i)
			end_ind = i
			break

	start_ind_rob, start_ind_cmd = lst[0][0], lst[0][1]
	end_ind_rob, end_ind_cmd = lst[end_ind][0], lst[end_ind][1]

	return start_ind_rob, start_ind_cmd, end_ind_rob, end_ind_cmd

def get_nonzero_indexes(cmd_array_subs_clip):

	# find where the robot control starts
	for m in range(0, cmd_array_subs_clip.shape[0]-1):
		if cmd_array_subs_clip[m, 0] != 0.0 or cmd_array_subs_clip[m, 1] != 0.0:
			ind_start_run = m
			break

	# find where the robot control ends
	for a in range(cmd_array_subs_clip.shape[0]-1, 0, -1):
		if cmd_array_subs_clip[a, 0] != 0.0 or cmd_array_subs_clip[a, 1] != 0.0:
			ind_end_run = a
			break

	return ind_start_run, ind_end_run

def fix_angles(states_array_robot, states_array_object):
	# if the velocity in the robot frame is negative, then we do += pi
	for i in range(0, states_array_robot.shape[0]):

		x_start_r = states_array_robot[0, 0]
		y_start_r = states_array_robot[0, 1]
		x_end_r = states_array_robot[-1, 0]
		y_end_r = states_array_robot[-1, 1]
		theta_r = np.arctan2(y_end_r - y_start_r, x_end_r - x_start_r)

	if np.abs(theta_r - states_array_robot[0, 2]) > np.deg2rad(90):
		states_array_robot[:, 2] += np.pi
			
	# if the velocity in the robot frame is negative, then we do += pi
	x_start_o = states_array_object[0, 0]
	y_start_o = states_array_object[0, 1]
	x_end_o = states_array_object[-1, 0]
	y_end_o = states_array_object[-1, 1]
	theta_o = np.arctan2(y_end_o - y_start_o, x_end_o - x_start_o)

	if np.abs(theta_o - states_array_object[0, 2]) > np.deg2rad(90):
		states_array_object[:, 2] += np.pi

	# make sure between -pi and pi
	for i in range(0, states_array_robot.shape[0]):
		if states_array_robot[i, 2] > np.pi:
			states_array_robot[i, 2] -= 2*np.pi
		# if states_array_robot[i, 2] < -2*np.pi:
		# 	states_array_robot[i, 2] += 2*np.pi

		if states_array_object[i, 2] > np.pi:
			states_array_object[i, 2] -= 2*np.pi
		# if states_array_object[i, 2] < -2*np.pi:
		# 	states_array_object[i, 2] += 2*np.pi

	return states_array_robot[:, 2], states_array_object[:, 2]

def main():
	max_nr = 18
	topics = {}
	topics["robot_pose"] = "Bebop1-pose"
	topics["robot_vel"] = "Bebop1-position_velocity_orientation_estimation"
	topics["object_pose"] = "Bebop2-pose"
	topics["object_vel"] = "Bebop2-position_velocity_orientation_estimation"
	topics["cmd"] = "husky_velocity_controller-cmd_vel"
	topics["cmd_joy"] = "joy_teleop-cmd_vel"

	traj_nr = 1
	total_data_points = 0
	for k in range(1, max_nr+1):
		print("Exp number: " + str(k) )

		# get arrays from datafiles
		states_array_robot, states_array_object, twists_array_robot, twists_array_object, cmd_array, timestamp_cmd, timestamp_robot_vel, timestamp_object_vel = get_arrs(k, topics)

		# subsampling
		states_array_robot_subs = subsample(timestamp_robot_vel, states_array_robot)
		twists_array_robot_subs = subsample(timestamp_robot_vel, twists_array_robot)

		# subsample object data
		states_array_object_subs = subsample(timestamp_object_vel, states_array_object)
		twists_array_object_subs = subsample(timestamp_object_vel, twists_array_object)

		# subsample cmd vel data
		cmd_array_subs = subsample(timestamp_cmd, cmd_array)

		print("-----------")
		# get all indexes where the timestamps overlap and then use the start and end ones to clip everything to the same size
		start_ind_rob, start_ind_cmd, end_ind_rob, end_ind_cmd = get_overlapping_indexes(states_array_robot_subs, cmd_array_subs)

		states_array_robot_subs_clip = states_array_robot_subs[start_ind_rob:end_ind_rob, :]
		states_array_object_subs_clip = states_array_object_subs[start_ind_rob:end_ind_rob, :]
		twists_array_robot_subs_clip = twists_array_robot_subs[start_ind_rob:end_ind_rob, :]
		twists_array_object_subs_clip = twists_array_object_subs[start_ind_rob:end_ind_rob, :]
		cmd_array_subs_clip = cmd_array_subs[start_ind_cmd:end_ind_cmd, :]

		# filter out points where there is no control command so no movement
		ind_start_run, ind_end_run = get_nonzero_indexes(cmd_array_subs_clip)

		states_array_robot_subs_clip2 = states_array_robot_subs_clip[ind_start_run:ind_end_run, :]
		states_array_object_subs_clip2 = states_array_object_subs_clip[ind_start_run:ind_end_run, :]
		twists_array_robot_subs_clip2 = twists_array_robot_subs_clip[ind_start_run:ind_end_run, :]
		twists_array_object_subs_clip2 = twists_array_object_subs_clip[ind_start_run:ind_end_run, :]
		cmd_array_subs_clip2 = cmd_array_subs_clip[ind_start_run:ind_end_run, :]

		# plotting subsampled and clipped data
		plotting_vels(cmd_array_subs_clip2, k)
		plotting_trajectories(states_array_robot_subs_clip2, states_array_object_subs_clip2, k)
		plotting_twists_robot(twists_array_robot_subs_clip2, k)
		plotting_twists_object(twists_array_object_subs_clip2, k)

		assert(states_array_object_subs_clip2.shape[0] == states_array_robot_subs_clip2.shape[0] == cmd_array_subs_clip2.shape[0])


		total_data_points += twists_array_object_subs_clip2.shape[0]
		print("total_data_points", total_data_points)

		print(cmd_array_subs_clip2.shape)
		print(states_array_robot_subs_clip2.shape)
		print(states_array_object_subs_clip2.shape)
		print(twists_array_robot_subs_clip2.shape)
		print(twists_array_object_subs_clip2.shape)

		
		# convert everything to a nice numpy array that can be saved!
		# data_arr = np.zeros((cmd_array_subs_clip2.shape[0], 13))
		data_arr = np.zeros((20, 14))

		save_path = "preprocessed_data2/"

		vv = 0
		traj_nr_curr = 0

		for v in range(0, states_array_robot_subs_clip2.shape[0]):
			# robot pose
			data_arr[vv, 0] = states_array_robot_subs_clip2[v, 0]
			data_arr[vv, 1] = states_array_robot_subs_clip2[v, 1]
			data_arr[vv, 2] = states_array_robot_subs_clip2[v, 2]

			# robot vel
			data_arr[vv, 3] = np.sqrt(twists_array_robot_subs_clip2[v, 0]**2 + twists_array_robot_subs_clip2[v, 1]**2)
			data_arr[vv, 4] = twists_array_robot_subs_clip2[v, 2]

			# object pose
			data_arr[vv, 5] = states_array_object_subs_clip2[v, 0]
			data_arr[vv, 6] = states_array_object_subs_clip2[v, 1]
			data_arr[vv, 7] = states_array_object_subs_clip2[v, 2]		

			# object vel
			data_arr[vv, 8] = twists_array_object_subs_clip2[v, 0]
			data_arr[vv, 9] = twists_array_object_subs_clip2[v, 1]
			data_arr[vv, 10] = twists_array_object_subs_clip2[v, 2]	

			# action
			data_arr[vv, 11] = cmd_array_subs_clip2[v, 0] #np.sqrt( cmd_array_subs_clip2[v, 0]**2 + cmd_array_subs_clip2[v, 1]**2 )
			data_arr[vv, 12] = cmd_array_subs_clip2[v, 1]

			data_arr[vv, 13] = vv

			vv +=1		
			if v % 20 == 0 and traj_nr_curr==0:
				vv = 0
				traj_nr_curr+=1
			elif v % 20 == 0 and traj_nr_curr>0:
				print(data_arr)
				data_arr_robot_angle, data_arr_obj_angle = fix_angles(data_arr[:, 0:3], data_arr[:, 5:8])
				data_arr[:, 2] = data_arr_robot_angle
				data_arr[:, 7] = data_arr_obj_angle
				np.save(save_path + "push_trajectory" + str(traj_nr), data_arr)
				vv = 0
				traj_nr +=1
				traj_nr_curr+=1
			else:
				pass




		print("data_arr", data_arr.shape)
		
		# np.save(save_path + "push_trajectory" + str(k), data_arr)

if __name__ == "__main__":
	main()