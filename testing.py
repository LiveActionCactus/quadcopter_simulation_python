# QuadTest extends the Quadcopter class with testing options
#
# By: Patrick Ledzian
# Date: 14 Apr 2020

"""
A testing class that will include different methods to test different aspects of the Quadcopter class. The goal here is
to automate the testing of many of the performance metrics, where does the current modelling break down, and is it the
modelling approach or is it the vehicle itself?
"""

# External Libraries
import numpy as np
import matplotlib.pyplot as plt
import random

# Project Libraries
from quadcopter import Quadcopter

class QuadTest(Quadcopter):
	"""
	Defines some testing metrics and inherits from the Quadcopter class in quadcopter.py.
	TODO: build out the quadcopter testing class
	"""
	_sim_params: dict

	def __init__(self, sim_params={}, initial_state=np.zeros(6), desired_state=np.zeros(3)):
		"""
		Class constructor, defines input parameters for the super construction of the Quadcopter class
		:param sim_params: user can specify, if not this is defined in the constructor
		:param initial_state: 1x6 quadcopter position and orientation
		:param desired_state: 1x3 desired quadcopter position
		"""
		super_initial_state = initial_state
		super_desired_state = desired_state
		super().__init__(initial_state=super_initial_state, desired_state=super_desired_state)
		random.seed(1) 			# for stochastic disturbances

		# TODO: create checking for the "sim_parameters" argument so that users fill in all needed values
		if not bool(sim_params):
			self._sim_params = {
				"sim_steps": 100,
				"tstep": 0.01,
				"start_time": 0,
				"end_time": 10
			}
		assert len(self._sim_params) == 4, "self._sim_params initialized to incorrect size!"

	def simple_line_traj_test(self):
		"""
		Plots the results of running the simple line trajectory. Visual inspection is important before asking the
		vehicle to perform trajectories.
		:return: NONE, outputs a plot of the trajectory (does not take into account quadcopter dynamics)
		"""
		# TODO: FIX THE TRAJECTORY
		# test the trajectory output, assume perfect tracking
		# TODO: include some sort of time analysis of the trajectory, velocity plots? overlay with acceleration?
		traj_prev = self._state[0:3]
		cur_time = 0
		pos_trackx = []
		pos_tracky = []
		pos_trackz = []
		# TODO: create more comprehensive tests with stochastic disturbances
		for itr in range(0, int((self._sim_params["end_time"]+self._sim_params["tstep"])/self._sim_params["tstep"])):
			cur_time = itr*self._sim_params["tstep"]
			traj = self.simple_line_trajectory(traj_prev, self._desired_state[0:3], 10, cur_time)

			# I don't currently understand what is happening with the stochastic disturbance plot
			traj_prev = traj[0:3] # + np.array([round(random.random()/10, 3), round(random.random()/10, 2), round(random.random()/10, 3)])
			pos_trackx.append(traj_prev[0])
			pos_tracky.append(traj_prev[1])
			pos_trackz.append(traj_prev[2])

		fig = plt.figure()
		ax = plt.axes(projection='3d')
		ax.plot3D(pos_trackx, pos_tracky, pos_trackz)
		ax.set_xlabel('X position')
		ax.set_ylabel('Y position')
		ax.set_zlabel('Z position')
		ax.set_title('Testing the "simple_line_trajectory" function')
		ax.scatter3D(pos_trackx[0], pos_tracky[0], pos_trackz[0], c="g", s=20)
		ax.scatter3D(pos_trackx[999], pos_tracky[999], pos_trackz[999], c="r", s=20)
		plt.show()