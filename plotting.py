import numpy as np
import matplotlib.pyplot as plt

states = open("states.csv", "r")
times = open("times.csv", "r")

state_data_test = np.genfromtxt(states, delimiter=",")
time_data_test = np.genfromtxt(times, delimiter=",")
data_points = np.shape(time_data_test)[0]

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot([], [], [], '-', c='cyan')[0]
ax.plot([], [], [], '-', c='red')[0]
ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]
ax.plot([], [], [], '.', c='red', markersize=4)[0]
ax.plot([], [], [], '.', c='red', markersize=2)[0]
set_limit((-1.0, 1.0), ())

def plot_waypoints(wpts):
    zx = plt.gca()
    lines = ax.get_lines()
    lines[-2].set_data(wpts[:, 0], wpts[:, 1])
    lines[-2].set_3d_properties(wpts[:, 2])


