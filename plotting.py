import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
import time

import quadcopter

states = open("states.csv", "r")
times = open("times.csv", "r")

# state_data_test = np.genfromtxt(states, delimiter=",")
# state_data_test = np.transpose(state_data_test[0:800, :])
# # TODO: set the plotting sample rate in code, need to think about this a bit (falls in category with pos/att asynchronicity)
# state_data_test = state_data_test[:, ::8]                       # down-sampling is important!
# time_data_test = np.genfromtxt(times, delimiter=",")
# num_data_points = np.shape(time_data_test)[0]


# TODO: have this class inherit from Quadcopter() to get it's properties
# TODO: do I need a quaternion class to encapsulate it's functionality? (maybe when I add the non-linear controller?) could have a bunch of static methods
# TODO: add waypointing
# TODO: add trajectory ghost line to track in gray (see how close we get) show error metric
# TODO: link the plotting functionality to the Quacopter class
# TODO: allow the QuadPlot class to plot multiple vehicles at once
class QuadPlot:
    def __init__(self, sim_state_data):
        if str(np.shape(sim_state_data)[0]) == "13":
            pass
        elif str(np.shape(sim_state_data)[1]) == "13":
            sim_state_data = np.transpose(sim_state_data)
        else:
            raise Exception("The dimensions of the input data is incorrect, check and try again")

        sim_state_data = sim_state_data[:, ::8]                 # TODO: handle the downsampling in the main.py file, it's a simulation spec

        fig = plt.figure()
        ax = p3.Axes3D(fig)
        data = np.array([[sim_state_data[0, :], sim_state_data[1, :], sim_state_data[2, :]]])

        # declare line objects
        ax.plot([], [], [], '-', c='cyan')[0]                                   # quad arm
        ax.plot([], [], [], '-', c='red')[0]                                    # quad arm
        ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]          # quad center
        ax.plot([], [], [], '.', c='red', markersize=3)[0]                      # waypoints
        ax.plot([], [], [], '.', c='blue', markersize=2)[0]                     # trailing line
        ax.view_init(elev=30.0, azim=285)
        ax.dist = 12.0

        self._q1_pos_obj = [ax.plot(dat[0:1, 0], dat[0:1, 1], dat[0:1, 2])[0] for dat in data]
        self.set_limits(1.0, 1.0, 3.0)

        self._fig = fig
        self._pos_data = data
        self._full_state = sim_state_data

    # main plotting function
    def plot_quad_3d(self):
        t = time.time()
        ani = animation.FuncAnimation(self._fig, self.update_quad, np.shape(self._pos_data)[2],
                                      fargs=(self._pos_data, self._q1_pos_obj),
                                      interval=1, blit=False, repeat=False)
        plt.show()
        elapsed = time.time() - t
        print(elapsed)

    # animation helper functions
    def set_limits(self, x, y, z):
        # Setting the axes properties
        ax = plt.gca()
        ax.set_xlim3d([0.0, x])  # x
        ax.set_ylim3d([0.0, y])  # y
        ax.set_zlim3d([0.0, z])  # z

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Altitude')
        ax.set_title('Quadcopter Simulation')

    def plot_waypoints(self, wpts):
        # TODO: pre-plot the points that the quadcopter is trying to maneuver over
        pass

    def update_quad(self, itr, pos_data, pos_obj):
        quad_pos_world = self.quad_pos_world(self._full_state[:, itr])

        ax = plt.gca()
        lines = ax.get_lines()
        lines_data = [quad_pos_world[:, [0, 2]], quad_pos_world[:, [1, 3]], quad_pos_world[:, [4, 5]]]

        # rotate quad arms
        for line_obj, line_data in zip(lines[0:3], lines_data):
            x, y, z = line_data
            line_obj.set_data(x, y)
            line_obj.set_3d_properties(z)

        # trailing line/history update
        lines[4].set_data(pos_data[0, 0:2, :itr])
        lines[4].set_3d_properties(pos_data[0, 2, :itr])

    def quad_pos_world(self, state, L=0.046, H=0.05):
        pos = state[0:3]
        q = state[6:10]
        rot = quadcopter.Quadcopter.quat2rot(q)        # express quaternion rotation as a rotation matrix, ZXY rotation type

        # homogeneous transform from body to world frame
        padding = np.array([0, 0, 0, 1])                    # allows rigid body rotation about a point wrt SO(3) definition
        wHb = np.concatenate((np.concatenate((rot, pos[:, None]), axis=1), padding[None, :]), axis=0)

        body_frame = np.transpose(np.array([
            [L, 0, 0, 1],
            [0, L, 0, 1],
            [-L, 0, 0, 1],
            [0, -L, 0, 1],
            [0, 0, 0, 1],
            [0, 0, H, 1]
        ]))
        world_frame = np.matmul(wHb, body_frame)
        quad_pos_world = world_frame[0:3, :]

        return quad_pos_world



# plot = QuadPlot()
# plot.plot_quad_3d()




# def __init__(self, state_data_test):
#     self._state_data = state_data_test
#     self._fig = plt.figure()
#     ax = self._fig.add_axes([0, 0, 1, 1], projection='3d', label='quad_behavior')
#     ax.view_init(elev=45.0, azim=70)
#     ax.dist = 12.0
#     self.set_limit((-3.0, 3.0), (-3.0, 3.0), (0.0, 5.0))
#
#     ax.plot([], [], [], '-', c='cyan')[0]
#     ax.plot([], [], [], '-', c='red')[0]
#     ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)[0]
#     ax.plot([], [], [], '.', c='red', markersize=4)[0]
#     ax.plot([], [], [], '.', c='red', markersize=2)[0]
