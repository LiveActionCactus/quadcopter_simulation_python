# Quadcopter simulation
#
# By: Patrick Ledzian
# Date: 08 April 2020

# Resources
# https://cs231n.github.io/python-numpy-tutorial/
# https://github.com/yrlu/quadrotor
# https://github.com/hbd730/quadcopter-simulation
#

"""
A simulation of quadcopter(s) using actual vehicle parameters verified via system identification. The implementation is
modular allowing for new dynamics and individual vehicle control, and trajectory modules to be substituted in.
"""

# External Libraries
import numpy as np
from scipy import integrate
import testing                      # for testing trajectories, can uncomment below
import matplotlib.pyplot as plt

# Project Libraries
import quadcopter
import plotting

# TODO: build unit tests to make sure our asserts are catching errors, need better error catching as well
# TODO: build unit tests to make sure data of the correct bounds and dimension are going into each function
# TODO: Expand simulation loop to include multiple vehicles
# TODO: resizer utility function for logging arrays, break the simulation loop when the error converges (for each quad and total metric)
# TODO: ensure that the attitude control loop is running faster than the trajectory one (200-400Hz vs 100Hz), have this as a tunable parameter
# TODO: build class for quaternion operations
# TODO: have a swappable position solution (MoCap @ 300/400Hz, GPS @ 1-5Hz), maybe include noise as a parameter
# TODO: have each quadcopter object keep track of their states and attitude/position loop rates (main sets step size), pre-allocate array size in __init()__
# TODO: include an integration health checker to stop the simulation if one of the values is going out of bounds

# Set simulation parameters
sim_steps = 750             # total steps
tstep = 0.01                # seconds per step
cstep = 0.05                # controller integration window (sliding window, integration bounds)
start_time = 0
time = start_time

num_quads = 1                                       # only supports 1 vehicle as of now
quad_list = []

# start_pos = np.array([0.0, 0.0, 1.0, 0, 0, 0])		# x, y, z, qx, qy, qz       Straight Line Test
start_pos = np.array([0.5, 0.5, 0.0, 0, 0, 0])		# x, y, z, qx, qy, qz       Hover Test, YOU NEED TO SWAP TRAJECTORIES IN quadcopter.py simulation_step()

# TODO: fix the straight line trajectory
# TODO: fix bug in quat2rot line 59 if 800 sim steps and desired position is [0.5, 0.5, 1.5]
end_pos = np.array([0.5, 0.5, 1.5])  				# x, y, z
quad = quadcopter.Quadcopter(start_pos, end_pos)    # single quadcopter object, we are currently only running single vehicle simulations

state_data = np.zeros((sim_steps, 13))        # rows = iterations in simulation, columns = state values
time_data = np.zeros(sim_steps)
err_data = np.zeros(sim_steps)
step_size = int(cstep/tstep)
assert int(cstep % tstep) == 0, "Step size is not evenly divisible"

#
# Run simulation
#

for itr in range(0, int(sim_steps/step_size)):
    timeint = np.arange(time, time + cstep, tstep)
    save_state = integrate.odeint(lambda s, t: quad.simulation_step(s, t, 10), quad._state, timeint, printmessg=False)          # func(y,t), y0, t,
    time += cstep

    print((itr*step_size)+5)
    state_data[(itr*step_size):(itr*step_size)+5] = save_state[0:5]         # TODO: change "save_state[0:5] to "save_state[0:step_size]"
    time_data[itr] = time
    err = np.linalg.norm(end_pos - save_state[3, 0:3])
    err_data[itr] = err

    if err < 0.1:
        break

# Plot animation of simulation results
plot = plotting.QuadPlot(state_data)
plot.plot_quad_3d()

# Plot Frobenius norm of position error over time (need to uncomment function call right now)
# TODO: figure out what to do with the "analysis" functions, maybe have a new class for analysis
def plot_state_pos_err(err_data):
    """
    Plots a single value representing vehicle state error (position, or attitude, or etc...) over time

    :param err_data: 1xN vector of error values
    :return: NONE, produces a plot
    """
    fig = plt.figure()
    ax = plt.axes()
    ax.plot(time_data, err_data)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("State Error")
    ax.set_title("State Error vs Time")
    plt.show()

# print(err_data)
# plot_state_pos_err(err_data)


# Plot the simple straight-line trajectory, no quadcopter dynamics (shows if it's working)
# TODO: trajectory testing class, shows the trajectories plotting in an infinite acceleration perfectly holonomic world
# test = testing.QuadTest([], start_pos, end_pos)             # sim_params, start_pos, end_pos
# test.simple_line_traj_test()
