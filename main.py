# Quadcopter simulation
#
# By: Patrick Ledzian
# Date: 08 April 2020

# Resources
# https://cs231n.github.io/python-numpy-tutorial/
#
#

import numpy as np
import testing
import quadcopter

# TODO: build unit tests to make sure our asserts are catching errors
# TODO: build unit tests to make sure data of the correct bounds and dimension are going into each function

#
# Run simulation
#

# Set simulation parameters
sim_steps = 100
tstep = 0.01
cstep = 0.05
start_time = 0
time = start_time

num_quads = 1
quad_list = []

start_pos = np.array([0.0, 0.0, 1.0, 0, 0, 0])		# x, y, z, qx, qy, qz
end_pos = np.array([1.0, 0.0, 1.5])  				# x, y, z

quad = quadcopter.Quadcopter(start_pos, end_pos)

# timeint = np.arange(time, time + cstep, tstep)




# y =

# for itr in range(1,sim_steps):
# 	#timeint = time:tstep:time+cstep
# 	timeint = np.arange(time, time+cstep, tstep)
# 	print(timeint)
# 	time += cstep
# 	break
#
# 	# TODO: terminate check for simulation results
# 	# TODO: plotting


# # TESTING
# test = testing.QuadTest([], start_pos, end_pos)             # sim_params, start_pos, end_pos
# test.simple_line_traj_test()
