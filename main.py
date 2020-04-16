# Quadcopter simulation
#
# By: Patrick Ledzian
# Date: 08 April 2020

# Resources
# https://cs231n.github.io/python-numpy-tutorial/
#
#

import numpy as np
from scipy import integrate
import testing
import quadcopter
import matplotlib.pyplot as plt

# TODO: build unit tests to make sure our asserts are catching errors
# TODO: build unit tests to make sure data of the correct bounds and dimension are going into each function

#
# Run simulation
#

# Set simulation parameters
sim_steps = 1000
tstep = 0.01
cstep = 0.05
start_time = 0
time = start_time

num_quads = 1
quad_list = []

start_pos = np.array([0.0, 0.0, 1.0, 0, 0, 0])		# x, y, z, qx, qy, qz
end_pos = np.array([1.0, 0.0, 1.5])  				# x, y, z

quad = quadcopter.Quadcopter(start_pos, end_pos)

state_data = np.zeros((sim_steps, 13))        # rows = iterations in simulation, columns = state values
time_data = np.zeros(sim_steps)
err_data = np.zeros(sim_steps)
step_size = int(cstep/tstep)
assert int(cstep % tstep) == 0, "Step size is not evenly divisible"
# TODO: encapsulate plotting for quadcopter outputs, expand simulation loop to include multiple vehicles
# TODO: resizer utility function for logging arrays, break the simulation loop when the error converges (for each quad and total metric)
for itr in range(0, int(sim_steps/step_size)):
    timeint = np.arange(time, time + cstep, tstep)
    save_state = integrate.odeint(lambda s, t: quad.simulation_step(s, t, 10), quad._state, timeint, printmessg=1)          # func(y,t), y0, t,
    time += cstep

    print((itr*step_size)+5)
    state_data[(itr*step_size):(itr*step_size)+5] = save_state[0:5]
    time_data[itr] = time
    err_data[itr] = np.linalg.norm(end_pos - save_state[3, 0:3])

print(err_data)

fig = plt.figure()
ax = plt.axes()
ax.plot(time_data, err_data)
ax.set_xlabel("Time (s)")
ax.set_ylabel("State Error")
ax.set_title("State Error vs Time")
plt.show()

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
