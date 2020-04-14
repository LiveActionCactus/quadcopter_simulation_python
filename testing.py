# QuadTest extends the Quadcopter class with testing options
#
# By: Patrick Ledzian
# Date: 14 Apr 2020

import numpy as np
import matplotlib.pyplot as plt
import random
from mpl_toolkits import mplot3d

from quadcopter import Quadcopter


class QuadTest(Quadcopter):
	_sim_params: dict

	def __init__(self, sim_params={}, initial_state=np.zeros(6), desired_state=np.zeros(3)):
		super_initial_state = initial_state
		super_desired_state = desired_state
		super().__init__(initial_state=super_initial_state, desired_state=super_desired_state)
		random.seed(1) 			# for stochastic disturbances

		if not bool(sim_params):
			self._sim_params = {
				"sim_steps": 100,
				"tstep": 0.01,
				"start_time": 0,
				"end_time": 10
			}
		assert len(self._sim_params) == 4, "self._sim_params initialized to incorrect size!"

	def simple_line_traj_test(self):
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